#include "gggs_tile.h"
#include "gggs_tile_util.h"

#include <gdal_priv.h>

#include <QDateTime>
#include <QFileInfo>
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
  path_(path),
  // [camp#103/#180] Level parsed ONCE from the filename
  // (`<level>_<row>_<col>.tif`) — a store's fine tiles and its overviews/
  // sidecar (uma ADR-0011) differ only by this, and getElevation() reads the
  // cached value per cursor move. Independent of the GDAL open below, so it
  // is set even for a tile that fails to open (-1 only on a non-value name).
  level_(tileLevel(QFileInfo(path).fileName()))
{
  readMetadata();
}

void GggsTile::readMetadata()
{
  // [camp#194 review] Re-runnable metadata read (constructor + refreshFromFile).
  // Zero the dimensions/band count first so a failed re-read leaves valid() ==
  // false (and loadPixels() re-latching the failure) rather than reporting the
  // PREVIOUS file's shape. The geographic extent members are deliberately NOT
  // zeroed: they are the layer's scene-bounds/spatial-filter input for every
  // tile regardless of valid(), and a GGGS tile's extent is fixed by its
  // `<level>_<row>_<col>` grid cell, so keeping the last known extent is both
  // correct and stops a failed re-read from collapsing the layer's footprint.
  width_ = 0;
  height_ = 0;
  band_count_ = 0;

  // [camp#194 review] Record the file identity the change detector compares
  // against. Taken BEFORE the read so a producer swapping the file mid-read is
  // seen as changed by the next rescan (the stat then predates the bytes we
  // actually got) rather than missed.
  const QFileInfo info(path_);
  const bool exists = info.exists() && info.isFile();
  file_size_ = exists ? info.size() : -1;
  file_mtime_ms_ = exists ? info.lastModified().toMSecsSinceEpoch() : -1;

  const QString& path = path_;
  if(GDALGetDriverCount() == 0)
    GDALAllRegister();

  auto dataset = GDALDataset::FromHandle(GDALOpen(path.toUtf8().constData(), GA_ReadOnly));
  if(!dataset)
    return;

  const int band_count = dataset->GetRasterCount();
  if(dataset->GetGeoTransform(geo_transform_) != CE_None || band_count < 1)
  {
    GDALClose(dataset);
    return;
  }
  band_count_ = band_count;   // [camp#108] retained so the layer can offer a picker

  const int width = dataset->GetRasterXSize();
  const int height = dataset->GetRasterYSize();

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

  GDALClose(dataset);

  // [camp#108] NoData is NOT read here: loadPixels() re-queries it for whichever
  // band is selected (the ctor cached only band 1's value, which setBand() made
  // stale), so has_nodata_/nodata_ stay at their defaults until the first
  // loadPixels(). No path reads them before then (texture()/range are gated on
  // pixelsLoaded()), so the ctor read was dead — removed.
  //
  // [camp#102] Extent/metadata only — NO band RasterIO here. The pixel read is
  // deferred to loadPixels() so a large store opens without blocking the GUI
  // thread (the layer drives loadPixels() off a QtConcurrent worker). dataMin/
  // dataMax stay at the crossed sentinel until loadPixels() runs.
  width_ = width;
  height_ = height;
}

bool GggsTile::fileChangedOnDisk() const
{
  // [camp#194 review] Cheap stat compare — see the header contract. A file that
  // vanished reports changed (-1/-1 vs the recorded stat), so a tile removed and
  // rewritten by a producer swap is refreshed rather than left latched-failed.
  const QFileInfo info(path_);
  const bool exists = info.exists() && info.isFile();
  const qint64 size = exists ? info.size() : -1;
  const qint64 mtime = exists ? info.lastModified().toMSecsSinceEpoch() : -1;
  return size != file_size_ || mtime != file_mtime_ms_;
}

bool GggsTile::refreshFromFile()
{
  // [camp#194 review] The explicit same-band retry for a REPLACED file: drop the
  // stale pixels/range, clear the sticky failure (the premise of the failure —
  // the bytes at this path — has changed), and re-read the metadata so the
  // extent/dimensions follow the new file. Ordering: clear the failure BEFORE
  // the re-read, so a re-read that itself fails leaves valid() false and the
  // next loadPixels() re-latches honestly.
  // The caller must have aborted + joined the load worker first (it mutates
  // state the worker reads) and released the GL texture under a current context
  // (the resetPixels() pairing INVARIANT) — GggsTileLayer::rescan() does both.
  resetPixels();
  load_failed_.store(false, std::memory_order_release);
  readMetadata();
  return valid();
}

bool GggsTile::loadPixels()
{
  // [camp#194] Every failure exit below latches load_failed_ (RELEASE, pairing
  // with the GUI thread's ACQUIRE in loadFailed()) so a tile that passed the
  // constructor's cheap metadata scan but cannot actually be read stops being
  // retried on every kick — and, more importantly, stops holding the layer's
  // hasUnloadedVisibleTiles() predicate true for the rest of the session (see
  // the loadFailed() contract in gggs_tile.h). Failures here are terminal for
  // the current band: the causes are a vanished/truncated file, a missing band,
  // or a GDAL read error, none of which a retry with identical parameters
  // resolves.
  if(!valid())
  {
    load_failed_.store(true, std::memory_order_release);
    return false;
  }
  if(pixels_loaded_.load(std::memory_order_acquire))
    return true;

  // [camp#102] Pure GDAL read — safe off the GUI thread (no GL touched here).
  auto dataset = GDALDataset::FromHandle(GDALOpen(path_.toUtf8().constData(), GA_ReadOnly));
  if(!dataset)
  {
    load_failed_.store(true, std::memory_order_release);
    return false;
  }
  if(dataset->GetRasterCount() < band_)
  {
    GDALClose(dataset);
    load_failed_.store(true, std::memory_order_release);
    return false;
  }
  auto band = dataset->GetRasterBand(band_);

  // [camp#108] Re-query NoData for the band actually being read. The constructor
  // cached band 1's value, but each band can declare its own NoData; setBand()
  // clears the buffer and routes back through here, so reading it from `band`
  // keeps the range filter below correct for the selected band.
  int has_nodata = 0;
  nodata_ = band->GetNoDataValue(&has_nodata);
  has_nodata_ = has_nodata != 0;

  std::vector<float> values(static_cast<size_t>(width_) * height_);
  if(band->RasterIO(GF_Read, 0, 0, width_, height_, values.data(),
                    width_, height_, GDT_Float32, 0, 0) != CE_None)
  {
    GDALClose(dataset);
    load_failed_.store(true, std::memory_order_release);
    return false;
  }
  GDALClose(dataset);

  // Range over valid samples (for the auto-ranged grayscale ramp).
  double min_value = std::numeric_limits<double>::max();
  double max_value = std::numeric_limits<double>::lowest();
  for(float v : values)
  {
    // [camp#122] Compare NoData in float to match the shader, which discards in
    // float (v == float(u_nodata)). For a Float32 band GetNoDataValue() already
    // returns the float-rounded sentinel, so float(nodata_) == nodata_ here and
    // the two compares usually agree; the float cast just guarantees CPU and GPU
    // stay on the same footing regardless of how the sentinel was stored.
    // [camp#122] Known theoretical edge: a NaN NoData sentinel never matches
    // v == u_nodata on the GPU (NaN != NaN), so the shader would discard nothing,
    // while the !isfinite check below still excludes it from the CPU auto-range —
    // the two paths diverge. In practice GGGS NoData is a finite sentinel (e.g.
    // 9999 or 0), so this does not arise.
    if(!std::isfinite(v) || (has_nodata_ && v == float(nodata_)))
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

float GggsTile::sampleAt(double lon, double lat) const
{
  // [camp#180] ACQUIRE gate, pairing with loadPixels()'s RELEASE store: false
  // until the worker has published BOTH data_ and the deferred NoData members
  // (nodata_/has_nodata_), so neither is ever read while unset (Plan Review #1).
  // A tile whose pixels were dropped (setBand()) also fails this gate.
  if(!pixelsLoaded() || data_.empty())
    return std::nanf("");

  // North-up geotransform (geo[2] == geo[4] == 0): invert to a pixel index.
  // geo[1] > 0 (lon/px), geo[5] < 0 (lat/px). A degenerate zero pixel size can't
  // index — treat as a miss rather than divide by zero.
  if(geo_transform_[1] == 0.0 || geo_transform_[5] == 0.0)
    return std::nanf("");
  // [camp#180] The diagonal-only inversion below assumes no rotation/shear
  // (geo[2] == geo[4] == 0), which holds for every GGGS tile (north-up WGS84 by
  // construction — see the ctor's extent math). Guard the assumption rather than
  // silently return a mis-indexed sample if a sheared tile ever slips through.
  if(geo_transform_[2] != 0.0 || geo_transform_[4] != 0.0)
    return std::nanf("");
  const int col = int(std::floor((lon - geo_transform_[0]) / geo_transform_[1]));
  const int row = int(std::floor((lat - geo_transform_[3]) / geo_transform_[5]));
  if(col < 0 || col >= width_ || row < 0 || row >= height_)
    return std::nanf("");

  const float v = data_[static_cast<size_t>(row) * width_ + col];
  // [camp#122] Same value filter as loadPixels()'s range loop: exclude non-finite
  // and the float-compared NoData sentinel so a masked pixel reads as "no value"
  // rather than a spurious elevation.
  if(!std::isfinite(v) || (has_nodata_ && v == float(nodata_)))
    return std::nanf("");
  return v;
}

void GggsTile::setBand(int band)
{
  // [camp#108] Switch which band loadPixels() reads. Out-of-range is a no-op so
  // a bad persisted/menu value can't strand the tile. The GL texture is left
  // alone here — the layer releases it under its own context (this runs on the
  // GUI thread but must not assume a current context).
  if(band < 1 || band > band_count_ || band == band_)
    return;
  band_ = band;
  // [camp#194] An explicit band switch is a genuine retry: the new band may be
  // readable where the old one was not (e.g. a band the file never carried), so
  // clear the sticky failure marker before the reload. Cleared HERE rather than
  // in resetPixels(), which is also the LOD release path — a tile released as a
  // stale finer-level backdrop is re-read with identical parameters, so its
  // failure must survive that.
  load_failed_.store(false, std::memory_order_release);
  // [camp#103] The clear body is shared with the LOD level-switch path.
  resetPixels();
}

void GggsTile::resetPixels()
{
  // Drop the loaded pixels + range so the next loadPixels() re-reads from
  // scratch (NoData is re-queried there, not here, since this path does not
  // open the dataset). See the header INVARIANT: callers pair this with
  // releaseGL() so a post-upload tile can't keep serving its stale texture.
  data_ = std::vector<float>();
  data_min_ = 1.0;   // crossed sentinel => no valid samples (range unknown)
  data_max_ = 0.0;
  // Release ordering kept for symmetry with the true-store in loadPixels(); the
  // actual cross-thread sync on this path is the worker abort+join the caller
  // (applyBand/rescan/LOD switch) performs first, so no paint thread observes
  // this store mid-flight. relaxed would be equally correct here.
  pixels_loaded_.store(false, std::memory_order_release);
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
    // [camp#134] The Nearest min/mag filter on this scalar value texture is set at
    // draw time by RasterGlRenderer (per-item, once per frame) — see camp#122 for the
    // rationale (Nearest avoids the NoData-sentinel halo the shader's exact-equality
    // discard would otherwise blend across). No upload-time setMinMagFilters() here.
    texture_->setWrapMode(QOpenGLTexture::ClampToEdge);
    // [camp#180] The CPU copy is intentionally RETAINED past the GPU upload (it
    // was freed here pre-#180 to halve resident memory). sampleAt() indexes this
    // resident buffer for the depth-at-cursor readout, so a point query stays a
    // cheap in-memory lookup with no synchronous GDAL re-open on the GUI thread
    // (Plan Review #4). Note the CPU buffer is actually held by EVERY loaded tile
    // (it is filled in loadPixels() and never freed), not just painted ones — so
    // the resident-memory footprint scales with the number of LOADED tiles. A
    // painted tile additionally holds the GL texture created here (kept for the
    // tile's lifetime; we never re-upload), so it alone carries both copies.
  }
  return texture_.get();
}

void GggsTile::releaseGL()
{
  texture_.reset();
}

}  // namespace raster
}  // namespace camp
