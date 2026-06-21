#include "gggs_tile.h"

#include <gdal_priv.h>

#include <QOpenGLTexture>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace camp
{
namespace raster
{

GggsTile::GggsTile(const QString& path):
  path_(path)
{
  if(GDALGetDriverCount() == 0)
    GDALAllRegister();

  auto dataset = GDALDataset::FromHandle(GDALOpen(path.toUtf8().constData(), GA_ReadOnly));
  if(!dataset)
    return;

  if(dataset->GetGeoTransform(geo_transform_) != CE_None || dataset->GetRasterCount() < 1)
  {
    GDALClose(dataset);
    return;
  }

  const int width = dataset->GetRasterXSize();
  const int height = dataset->GetRasterYSize();
  auto band = dataset->GetRasterBand(1);

  // Geographic extent from the geotransform. The tiles are north-up WGS84
  // (geo[2] == geo[4] == 0, geo[1] > 0, geo[5] < 0): row 0 is the north edge.
  const double x0 = geo_transform_[0];
  const double y0 = geo_transform_[3];
  const double x1 = x0 + width * geo_transform_[1] + height * geo_transform_[2];
  const double y1 = y0 + width * geo_transform_[4] + height * geo_transform_[5];
  min_lon_ = std::min(x0, x1);
  max_lon_ = std::max(x0, x1);
  min_lat_ = std::min(y0, y1);
  max_lat_ = std::max(y0, y1);

  int has_nodata = 0;
  nodata_ = band->GetNoDataValue(&has_nodata);
  has_nodata_ = has_nodata != 0;

  GDALClose(dataset);

  // [camp#102] Extent/metadata only — NO band RasterIO here. The pixel read is
  // deferred to loadPixels() so a large store opens without blocking the GUI
  // thread (the layer drives loadPixels() off a QtConcurrent worker). dataMin/
  // dataMax stay at the crossed sentinel until loadPixels() runs.
  width_ = width;
  height_ = height;
}

bool GggsTile::loadPixels()
{
  if(!valid())
    return false;
  if(pixels_loaded_.load(std::memory_order_acquire))
    return true;

  // [camp#102] Pure GDAL read — safe off the GUI thread (no GL touched here).
  auto dataset = GDALDataset::FromHandle(GDALOpen(path_.toUtf8().constData(), GA_ReadOnly));
  if(!dataset)
    return false;
  if(dataset->GetRasterCount() < 1)
  {
    GDALClose(dataset);
    return false;
  }
  auto band = dataset->GetRasterBand(1);

  std::vector<float> values(static_cast<size_t>(width_) * height_);
  if(band->RasterIO(GF_Read, 0, 0, width_, height_, values.data(),
                    width_, height_, GDT_Float32, 0, 0) != CE_None)
  {
    GDALClose(dataset);
    return false;
  }
  GDALClose(dataset);

  // Range over valid samples (for the auto-ranged grayscale ramp).
  double min_value = std::numeric_limits<double>::max();
  double max_value = std::numeric_limits<double>::lowest();
  for(float v : values)
  {
    if(!std::isfinite(v) || (has_nodata_ && v == nodata_))
      continue;
    min_value = std::min(min_value, double(v));
    max_value = std::max(max_value, double(v));
  }
  data_min_ = min_value;
  data_max_ = max_value;

  data_ = std::move(values);
  // [camp#102] RELEASE store AFTER all of data_/data_min_/data_max_ are written.
  // Pairs with the ACQUIRE load in pixelsLoaded() so the paint thread, once it
  // sees this flag true, is guaranteed to observe the completed buffer/range —
  // closing the worker-vs-paint race on both first load and the rescan() re-kick.
  pixels_loaded_.store(true, std::memory_order_release);
  return true;
}

GggsTile::~GggsTile()
{
  // texture_ (QOpenGLTexture) must be destroyed with its context current; the
  // layer calls releaseGL() under a current context before dropping the tile.
  // If it survives to here (e.g. context already gone at shutdown), reset() may
  // warn and leak the GL object — harmless on teardown.
}

QOpenGLTexture* GggsTile::texture()
{
  if(!texture_ && valid() && !data_.empty())
  {
    texture_ = std::make_unique<QOpenGLTexture>(QOpenGLTexture::Target2D);
    texture_->setFormat(QOpenGLTexture::R32F);
    texture_->setSize(width_, height_);
    texture_->setMipLevels(1);
    texture_->allocateStorage(QOpenGLTexture::Red, QOpenGLTexture::Float32);
    texture_->setData(QOpenGLTexture::Red, QOpenGLTexture::Float32, data_.data());
    texture_->setMinMagFilters(QOpenGLTexture::Linear, QOpenGLTexture::Linear);
    texture_->setWrapMode(QOpenGLTexture::ClampToEdge);
    // Free the CPU copy once it's on the GPU — the texture persists for the
    // tile's lifetime, so we never re-upload (releaseGL = teardown). Halves
    // resident memory per tile. dataMin/dataMax were captured at load.
    data_ = std::vector<float>();
  }
  return texture_.get();
}

void GggsTile::releaseGL()
{
  texture_.reset();
}

}  // namespace raster
}  // namespace camp
