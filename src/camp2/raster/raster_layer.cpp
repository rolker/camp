#include "raster_layer.h"
#include <gdal_priv.h>
#include <gdalwarper.h>
#include "../map_view/web_mercator.h"
#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QtConcurrent>
#include <cmath>
#include <limits>
#include <QMenu>
#include <QAction>
#include <QSettings>
#include <QFileInfo>

#include <QDebug>

namespace camp
{

namespace raster
{

RasterLayer::RasterLayer(map::MapItem* parentItem, const QString& filename):
  map::Layer(parentItem, QFileInfo(filename).fileName()),
  filename_(filename)
{
  if(GDALGetDriverCount() == 0)
    GDALAllRegister();
  connect(&future_watcher_, &QFutureWatcher<LoadResult>::finished, this, &RasterLayer::imageReady);
  // [#59 ADR-0003] Establish the scene extent + world transform synchronously,
  // before kicking off the async pixel load, so the layer knows where it is
  // (valid boundingRect/scenePos) immediately — fit-to-extent / zoom-on-open
  // works at load time instead of only after the warp completes.
  initExtent(filename);
  loadFile(filename);
}

RasterLayer::~RasterLayer()
{
  abort_flag_mutex_.lock();
  abort_flag_ = true;
  abort_flag_mutex_.unlock();
  future_watcher_.waitForFinished();
}

QRectF RasterLayer::boundingRect() const
{
  // [#59 ADR-0003] Prefer the synchronously-known reprojected dimensions so the
  // extent is valid before pixels load. The mipmap rect (same dimensions) is the
  // fallback for the failed-initExtent / not-yet-set case.
  if(reprojected_width_ > 0 && reprojected_height_ > 0)
    return QRectF(0, 0, reprojected_width_, reprojected_height_);
  if(!mipmaps_.empty())
    return mipmaps_.begin()->second.rect();
  return QRectF();
}


void RasterLayer::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
  if(!mipmaps_.empty())
  {
    auto lod = QStyleOptionGraphicsItem::levelOfDetailFromTransform(painter->worldTransform());
    auto level = mipmaps_.lower_bound( std::min(255, int(1.0/lod)) );
    if(level == mipmaps_.end())
      level--;
    painter->save();
    painter->setRenderHint(QPainter::SmoothPixmapTransform);
    painter->scale(level->first, level->first);
    painter->drawPixmap(0,0, level->second);
    painter->restore();
  }

}


void RasterLayer::initExtent(const QString& filename)
{
  // [#59 ADR-0003] Read only the reprojected geotransform + dimensions (no
  // pixels) and apply the world transform/position synchronously, so the layer
  // has a valid extent and scene position before the async pixel load. Mirrors
  // the placement loadAndReprojectFile/imageReady would set, just earlier and
  // pixel-free. GDALAutoCreateWarpedVRT is metadata-only and effectively instant.
  auto dataset = GDALDataset::FromHandle(GDALOpen(filename.toLatin1(), GA_ReadOnly));
  if(!dataset)
    return;
  auto reprojected = GDALDataset::FromHandle(
    GDALAutoCreateWarpedVRT(dataset, nullptr, web_mercator::wkt, GRA_Bilinear, 0.0, nullptr));
  if(reprojected)
  {
    double geo_transform[6] = {0.0};
    reprojected->GetGeoTransform(geo_transform);
    reprojected_width_ = reprojected->GetRasterXSize();
    reprojected_height_ = reprojected->GetRasterYSize();
    prepareGeometryChange();
    setTransform(QTransform::fromScale(geo_transform[1], geo_transform[5]), true);
    setPos(geo_transform[0], geo_transform[3]);
    GDALClose(reprojected);
  }
  GDALClose(dataset);
}

void RasterLayer::loadFile(const QString& filename)
{
  // Abort and join any in-flight load before starting a new one (e.g. a rapid
  // colormap change). setFuture() only tracks the latest future, so a replaced
  // job would otherwise become untracked: it would keep running, race the new
  // job on shared state, and — if the layer is destroyed first — outlive `this`
  // (use-after-free), since the dtor only waits on the watched future.
  if(future_watcher_.isRunning())
  {
    abort_flag_mutex_.lock();
    abort_flag_ = true;
    abort_flag_mutex_.unlock();
    future_watcher_.waitForFinished();
  }
  abort_flag_mutex_.lock();
  abort_flag_ = false;   // re-arm for the new job
  abort_flag_mutex_.unlock();

  setStatus("(loading...)");
  future_watcher_.setFuture(QtConcurrent::run(this, &RasterLayer::loadAndReprojectFile, filename));
}

RasterLayer::LoadResult RasterLayer::loadAndReprojectFile(const QString& filename)
{
  LoadResult result;

  auto dataset = GDALDataset::FromHandle(GDALOpen(filename.toLatin1(), GA_ReadOnly));

  if(!dataset)
  {
    qDebug("RasterLayer::loadFile null dataset");
    return result;
  }

  auto reprojected_dataset = GDALDataset::FromHandle(GDALAutoCreateWarpedVRT(dataset, nullptr, web_mercator::wkt, GRA_Bilinear, 0.0, nullptr));

  if(!reprojected_dataset)
  {
    qDebug("RasterLayer::loadFile error creating reprojected dataset");
    return result;
  }

  double reprojected_geo_transform[6] = {0.0};
  reprojected_dataset->GetGeoTransform(reprojected_geo_transform);

  result.world_x = reprojected_geo_transform[0];
  result.world_y = reprojected_geo_transform[3];

  result.scale_x = reprojected_geo_transform[1];
  result.scale_y = reprojected_geo_transform[5];

  auto width = reprojected_dataset->GetRasterXSize();
  auto height = reprojected_dataset->GetRasterYSize();

  QImage image(width, height, QImage::Format_ARGB32);
  image.fill(Qt::black);

  auto first_band = reprojected_dataset->GetRasterBand(1);
  if(reprojected_dataset->GetRasterCount() == 1 &&
     first_band->GetRasterDataType() == GDT_Float32)
  {
    result.is_scalar = true;
    // [camp#63/#59 PR3c] Single-band scalar field (e.g. a depth raster): shade
    // through the ColorMap over the data range instead of the UInt32 RGB path
    // (which renders Float32 values as near-black). NoData / NaN -> transparent.
    image.fill(Qt::transparent);
    int has_nodata = 0;
    const double nodata = first_band->GetNoDataValue(&has_nodata);
    std::vector<float> values(static_cast<size_t>(width) * height);
    if(first_band->RasterIO(GF_Read, 0, 0, width, height, values.data(), width, height, GDT_Float32, 0, 0) == CE_None)
    {
      double min_value = std::numeric_limits<double>::max();
      double max_value = std::numeric_limits<double>::lowest();
      for(float v : values)
      {
        if(std::isnan(v) || (has_nodata && v == nodata))
          continue;
        min_value = std::min(min_value, double(v));
        max_value = std::max(max_value, double(v));
      }
      // A constant-valued raster (all valid samples equal) would otherwise hit
      // ColorMap::color()'s degenerate max<=min guard and render fully
      // transparent — the raster would vanish. Widen the range so the single
      // value maps to the top of the ramp (matches grid_map's handling). If
      // there were no valid samples at all, min/max stay crossed and pixels
      // correctly stay transparent.
      if(min_value == max_value)
        min_value -= 1.0;
      const map::ColorMap cm = colormap_;
      for(int j = 0; j < height; ++j)
      {
        uchar* scanline = image.scanLine(j);
        for(int i = 0; i < width; ++i)
        {
          const float v = values[static_cast<size_t>(j) * width + i];
          const QColor c = (std::isnan(v) || (has_nodata && v == nodata))
                             ? QColor(0, 0, 0, 0) : cm.color(v, min_value, max_value);
          scanline[i*4+0] = c.blue();
          scanline[i*4+1] = c.green();
          scanline[i*4+2] = c.red();
          scanline[i*4+3] = c.alpha();
        }
        QMutexLocker lock(&abort_flag_mutex_);
        if(abort_flag_)
          return {};
      }
    }
  }
  else
  for(auto&& band: reprojected_dataset->GetBands())
  {
    auto color_table = band->GetColorTable();
    std::vector<uint32_t> buffer(width);
    for(int j = 0; j<height; ++j)
    {
      if(band->RasterIO(GF_Read, 0, j, width, 1, &buffer.front(), width, 1, GDT_UInt32, 0, 0)== CE_None)
      {
        uchar *scanline = image.scanLine(j);
        for(int i = 0; i < width; ++i)
        {
          if(color_table)
          {
            GDALColorEntry const *ce = color_table->GetColorEntry(buffer[i]);
            scanline[i*4] = ce->c3;
            scanline[i*4+1] = ce->c2;
            scanline[i*4+2] = ce->c1;
            scanline[i*4+3] = ce->c4;
          }
          else
          {
            if(band->GetColorInterpretation() == GCI_GrayIndex)
            {
                scanline[i*4+0] = buffer[i];
                scanline[i*4+1] = buffer[i];
                scanline[i*4+2] = buffer[i];
            }
            if(band->GetColorInterpretation() == GCI_RedBand)
                scanline[i*4+2] = buffer[i];
            if(band->GetColorInterpretation() == GCI_GreenBand)
                scanline[i*4+1] = buffer[i];
            if(band->GetColorInterpretation() == GCI_BlueBand)
                scanline[i*4+0] = buffer[i];
            if(band->GetColorInterpretation() == GCI_AlphaBand)
                scanline[i*4+3] = buffer[i];
          }
        }
      }
      QMutexLocker lock(&abort_flag_mutex_);
      if(abort_flag_)
        return {};
    }
  }

  result.mipmaps[1] = QPixmap::fromImage(image);
  for(int i = 2; i < 128; i*=2)
  {
    result.mipmaps[i] = QPixmap::fromImage(image.scaledToWidth(width/float(i),Qt::SmoothTransformation));
  }
  return result;
}

void RasterLayer::imageReady()
{
  auto result = future_watcher_.result();
  if(result.mipmaps.empty())   // failed load (null/unreprojectable dataset);
  {                            // don't apply the zero-filled transform/pos
    setStatus("(load failed)");
    return;
  }
  is_scalar_ = result.is_scalar;
  mipmaps_ = result.mipmaps;

  // [#59 ADR-0003] The world transform + position were already applied
  // synchronously in initExtent() (the warped geotransform is identical here),
  // and boundingRect() comes from the reprojected dimensions, so the geometry
  // does not change when the pixels arrive — only the painted content does.
  // Defensive fallback: if initExtent() did not establish the extent (e.g. a
  // transient open failure) but the async load nonetheless succeeded, apply the
  // placement here so the layer is still positioned correctly.
  if(reprojected_width_ <= 0 || reprojected_height_ <= 0)
  {
    prepareGeometryChange();
    reprojected_width_ = result.mipmaps.begin()->second.width();
    reprojected_height_ = result.mipmaps.begin()->second.height();
    setTransform(QTransform::fromScale(result.scale_x, result.scale_y), true);
    setPos(result.world_x, result.world_y);
  }
  update(boundingRect());
  setStatus("");
}

void RasterLayer::setColormap(map::ColorMap::Type type)
{
  if(type == colormap_.type())
    return;
  colormap_.setType(type);
  writeSettings();
  if(!filename_.isEmpty())     // re-warp + re-shade with the new ramp
    loadFile(filename_);
}

void RasterLayer::contextMenu(QMenu* menu)
{
  map::Layer::contextMenu(menu);
  if(!is_scalar_)              // colormap only applies to scalar (depth) rasters
    return;
  QMenu* colormap_menu = menu->addMenu("Colormap");
  for(auto type : map::ColorMap::allTypes())
  {
    QAction* action = colormap_menu->addAction(map::ColorMap::name(type));
    action->setCheckable(true);
    action->setChecked(type == colormap_.type());
    connect(action, &QAction::triggered, this, [this, type]() { setColormap(type); });
  }
}

void RasterLayer::readSettings()
{
  map::Layer::readSettings();
  QSettings settings;
  settings.beginGroup("MapItem");
  settings.beginGroup(itemID());
  const map::ColorMap::Type type = map::ColorMap::typeFromName(
    settings.value("colormap", map::ColorMap::name(colormap_.type())).toString());
  settings.endGroup();
  settings.endGroup();
  // Apply the persisted ramp (re-render if it differs from the default used by
  // the initial load); don't re-persist here.
  if(type != colormap_.type())
  {
    colormap_.setType(type);
    if(!filename_.isEmpty())
      loadFile(filename_);
  }
}

void RasterLayer::writeSettings()
{
  map::Layer::writeSettings();
  QSettings settings;
  settings.beginGroup("MapItem");
  settings.beginGroup(itemID());
  settings.setValue("colormap", map::ColorMap::name(colormap_.type()));
  settings.endGroup();
  settings.endGroup();
}

} // namespace raster

} // namespace camp
