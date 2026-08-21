#ifndef RASTER_GGGS_TILE_H
#define RASTER_GGGS_TILE_H

#include <QString>
#include <atomic>
#include <memory>
#include <vector>

class QOpenGLTexture;

namespace camp
{
namespace raster
{

/// [camp#90 / I4 / camp#102] One native-geographic GGGS raster tile loaded from a
/// WGS84 GeoTIFF (`<level>_<row>_<col>.tif`). The constructor reads only the
/// geographic extent + dimensions + band count from the GDAL geotransform/metadata
/// (no `RasterIO`) so `valid()` becomes true cheaply on the GUI thread; the band
/// pixels (and that band's NoData) are read later by `loadPixels()` (typically
/// off-thread, per camp#102 /
/// ADR-0003 §3) and then lazily uploaded as a single-channel float (R32F) GL
/// texture for the GPU display-time warp. Unlike RasterLayer, the tile is NOT
/// reprojected on load — it stays in lat/lon and is warped to Web-Mercator in the
/// shader at display time (unh_marine_autonomy ADR-0002 §D2).
class GggsTile
{
public:
  /// Open a north-up WGS84 GeoTIFF and read its geotransform → extent, dimensions
  /// and band count ONLY (no pixel `RasterIO`, and no NoData — both are deferred to
  /// `loadPixels()`, which queries NoData for the selected band).
  /// valid() is false if the file can't be opened / has no geotransform.
  explicit GggsTile(const QString& path);
  ~GggsTile();

  bool valid() const { return width_ > 0 && height_ > 0; }

  /// Read the selected band (`band()`, 1-indexed) as Float32 and compute the data
  /// range. Safe to call off the GUI thread (pure GDAL `RasterIO`, no GL). No-op
  /// if the tile is invalid or pixels are already loaded. Returns true if pixels
  /// are present after the call.
  bool loadPixels();

  /// [camp#108] Number of raster bands in the GeoTIFF (>= 1 for a valid tile).
  /// Read from GDAL in the constructor.
  int bandCount() const { return band_count_; }

  /// [camp#108] The 1-indexed band `loadPixels()` reads (default 1).
  int band() const { return band_; }

  /// [camp#108] Select which 1-indexed band `loadPixels()` reads. Clears any
  /// loaded CPU pixels + range so the next loadPixels() re-reads the new band,
  /// re-queries that band's NoData, and marks the tile not-loaded. Does NOT touch
  /// the GL texture — releasing/recreating it is the layer's responsibility (it
  /// owns the GL context). No-op if @p band is out of [1, bandCount()].
  void setBand(int band);

  /// [camp#103 / ADR-0013 + camp#180] The GGGS level parsed once from the tile
  /// filename (`<level>_<row>_<col>.tif`, via camp::raster::tileLevel) in the
  /// constructor; -1 if the name doesn't carry one. Immutable for the tile's
  /// lifetime. Fine tiles and `overviews/` sidecar tiles are distinguished
  /// only by this (camp#103 LOD selection), and getElevation() reads the
  /// cached value per cursor move instead of re-running the parse (camp#180).
  int level() const { return level_; }

  /// [camp#194] True for a tile loaded from the derived `overviews/` sidecar
  /// (uma ADR-0011) rather than the store's main directory. An overview tile
  /// is padded to its (coarse) GGGS grid cell, so the layer's scene-bounds
  /// union excludes it; NATIVE tiles — at any level, since a region-disjoint
  /// ENC ladder (uma ADR-0010 D7) puts several native levels in the main
  /// directory — are the true data footprint. Set once by the layer while
  /// scanning (loadDirectory()); default false (rescan() only scans the main
  /// directory, so its tiles are native).
  bool isOverview() const { return is_overview_; }
  void setOverview(bool overview) { is_overview_ = overview; }

  /// [camp#103] Drop the loaded CPU pixels + range and mark the tile not-loaded,
  /// so a later loadPixels() re-reads from scratch — the CPU half of releasing a
  /// tile when the LOD switches away from its level. Does NOT touch the GL
  /// texture. INVARIANT: every call site must pair this with releaseGL() under a
  /// current context — after a texture() upload, data_ is already freed while
  /// texture_ is non-null, so a CPU-only clear leaves a stale texture that
  /// shadows any re-loaded pixels (texture() returns the old texture and never
  /// consumes the new data_).
  void resetPixels();

  /// True once `loadPixels()` has read the band into CPU memory (or freed it into
  /// the GL texture). dataMin/dataMax and texture() are only meaningful once true.
  ///
  /// [camp#102] `pixels_loaded_` is `std::atomic<bool>` and this is an ACQUIRE
  /// load. It pairs with the RELEASE store in `loadPixels()` (issued AFTER the
  /// `data_` move) to establish happens-before: when the paint thread observes
  /// `pixelsLoaded() == true` it is guaranteed to see the worker's completed
  /// `data_`/range writes. No other path may read `data_`/`texture()` without
  /// first passing this acquire check (see `texture()`, gated below).
  bool pixelsLoaded() const
  {
    return pixels_loaded_.load(std::memory_order_acquire);
  }

  const QString& path() const { return path_; }
  int width() const { return width_; }
  int height() const { return height_; }

  /// Geographic extent in degrees, pixel-edge aligned (north-up: row 0 = maxLat).
  double minLon() const { return min_lon_; }
  double maxLon() const { return max_lon_; }
  double minLat() const { return min_lat_; }
  double maxLat() const { return max_lat_; }

  bool hasNoData() const { return has_nodata_; }
  double noData() const { return nodata_; }

  /// [camp#180] Sample the loaded band at a geographic point — the value at
  /// (@p lon, @p lat) degrees, or NaN if the point is outside the tile, the
  /// sample is NoData / non-finite, or the pixels are not resident. A pure
  /// in-memory index into the resident CPU buffer (no GDAL I/O), cheap enough to
  /// call per cursor move on the GUI thread. Gated on the same pixelsLoaded()
  /// ACQUIRE the paint path uses; that gate is set by loadPixels() AFTER the
  /// deferred NoData members are populated, so this never reads nodata_/
  /// has_nodata_ while unset. Safe against the load worker for the same
  /// happens-before reason (see pixelsLoaded()).
  float sampleAt(double lon, double lat) const;

  /// Min / max over valid (non-NoData, finite) samples. min_ > max_ if the tile
  /// has no valid samples. Used by the layer to auto-range the grayscale ramp.
  double dataMin() const { return data_min_; }
  double dataMax() const { return data_max_; }

  /// Ensure the R32F texture exists in the current GL context and return it.
  /// Must be called with a current context (i.e. from within paint()).
  QOpenGLTexture* texture();

  /// Free the GL texture. Must be called with the owning context current.
  void releaseGL();

private:
  QString path_;
  int width_ = 0;
  int height_ = 0;
  int band_count_ = 0;   // [camp#108] GDAL raster-band count (0 until valid)
  int band_ = 1;         // [camp#108] selected 1-indexed band loadPixels() reads
  int level_ = -1;       // [camp#103/#180] level from the filename (-1 unknown)
  bool is_overview_ = false;   // [camp#194] from the overviews/ sidecar
  double geo_transform_[6] = {0.0};
  double min_lon_ = 0.0, max_lon_ = 0.0, min_lat_ = 0.0, max_lat_ = 0.0;
  bool has_nodata_ = false;
  double nodata_ = 0.0;
  // [camp#102] Cross-thread publication flag: stored with release in loadPixels()
  // (worker thread) AFTER data_/range are written, loaded with acquire in
  // pixelsLoaded() (paint thread) before data_/texture() are read. Atomic so the
  // rescan() re-kick (range already valid → paint's crossed-range gate is open)
  // can't race the worker's mid-write.
  std::atomic<bool> pixels_loaded_{false};   // set once loadPixels() has run
  double data_min_ = 1.0, data_max_ = 0.0;   // crossed => no valid samples
  std::vector<float> data_;                  // row-major, height_ * width_
  std::unique_ptr<QOpenGLTexture> texture_;
};

}  // namespace raster
}  // namespace camp

#endif
