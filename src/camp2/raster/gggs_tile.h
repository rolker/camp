#ifndef RASTER_GGGS_TILE_H
#define RASTER_GGGS_TILE_H

#include <QString>
#include <memory>
#include <vector>

class QOpenGLTexture;

namespace camp
{
namespace raster
{

/// [camp#90 / I4] One native-geographic GGGS raster tile loaded from a WGS84
/// GeoTIFF (`<level>_<row>_<col>.tif`). Holds the CPU sample data + geographic
/// extent (from the GDAL geotransform) and lazily uploads a single-channel
/// float (R32F) GL texture for the GPU display-time warp. Unlike RasterLayer,
/// the tile is NOT reprojected on load — it stays in lat/lon and is warped to
/// Web-Mercator in the shader at display time (unh_marine_autonomy ADR-0002 §D2).
class GggsTile
{
public:
  /// Open a single-band-or-more north-up WGS84 GeoTIFF and read band 1 as
  /// Float32. valid() is false if the file can't be opened / has no geotransform.
  explicit GggsTile(const QString& path);
  ~GggsTile();

  bool valid() const { return width_ > 0 && height_ > 0; }

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
  double geo_transform_[6] = {0.0};
  double min_lon_ = 0.0, max_lon_ = 0.0, min_lat_ = 0.0, max_lat_ = 0.0;
  bool has_nodata_ = false;
  double nodata_ = 0.0;
  double data_min_ = 1.0, data_max_ = 0.0;   // crossed => no valid samples
  std::vector<float> data_;                  // row-major, height_ * width_
  std::unique_ptr<QOpenGLTexture> texture_;
};

}  // namespace raster
}  // namespace camp

#endif
