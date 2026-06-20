// [camp#90 / I4] Tests for the GGGS GPU-warp tile layer's non-GL logic:
//  - GggsTile geographic-extent recovery from the GDAL geotransform,
//  - NoData / data-range handling (the mosaicker reserves 0 for NoData and
//    floors real returns to >= 1), and
//  - geoToMap parity: the vertex shader warps lon/lat -> Web-Mercator with
//    x = R*lambda, y = R*asinh(tan phi), expanded as log(t + sqrt(t^2+1)) to
//    avoid the asinh builtin. This pins that formula against the C++ reference
//    web_mercator::geoToMap so the shader can't silently drift. (Full offscreen
//    GPU/CPU render parity is the rqt#48-style harness deferred to camp#63.)

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <gdal_priv.h>

#include <QGeoCoordinate>
#include <QPointF>
#include <QTemporaryDir>

#include "raster/gggs_tile.h"
#include "map_view/web_mercator.h"

using camp::raster::GggsTile;

namespace
{

// Write a single-band UInt16 north-up WGS84-ish GeoTIFF with the given
// geotransform, NoData = 0, and row-major samples. Returns the file path.
QString writeTile(const QTemporaryDir& dir, const QString& name,
                  int width, int height, const double geo[6],
                  const std::vector<uint16_t>& samples)
{
  if(GDALGetDriverCount() == 0)
    GDALAllRegister();
  const QString path = dir.filePath(name);
  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GTiff");
  GDALDataset* ds = driver->Create(path.toUtf8().constData(), width, height, 1,
                                   GDT_UInt16, nullptr);
  ds->SetGeoTransform(const_cast<double*>(geo));
  GDALRasterBand* band = ds->GetRasterBand(1);
  band->SetNoDataValue(0);
  const CPLErr err = band->RasterIO(GF_Write, 0, 0, width, height,
                                    const_cast<uint16_t*>(samples.data()),
                                    width, height, GDT_UInt16, 0, 0);
  GDALClose(ds);
  if(err != CE_None)
    return QString();
  return path;
}

}  // namespace

// Extent corners come straight from the geotransform: north-up tile, row 0 is
// the north edge (maxLat).
TEST(GggsTileTest, ExtentFromGeotransform)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const int w = 4, h = 4;
  const double dlon = 0.001, dlat = 0.001;
  const double min_lon = -71.4, max_lat = 43.0;
  const double geo[6] = {min_lon, dlon, 0.0, max_lat, 0.0, -dlat};
  std::vector<uint16_t> samples(w * h, 100);
  const QString path = writeTile(dir, "13_10_20.tif", w, h, geo, samples);

  GggsTile tile(path);
  ASSERT_TRUE(tile.valid());
  EXPECT_EQ(tile.width(), w);
  EXPECT_EQ(tile.height(), h);
  EXPECT_NEAR(tile.minLon(), min_lon, 1e-12);
  EXPECT_NEAR(tile.maxLon(), min_lon + w * dlon, 1e-12);
  EXPECT_NEAR(tile.maxLat(), max_lat, 1e-12);
  EXPECT_NEAR(tile.minLat(), max_lat - h * dlat, 1e-12);
}

// NoData (0) is excluded from the data range; real samples set min/max.
TEST(GggsTileTest, NoDataExcludedFromRange)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const int w = 2, h = 2;
  const double geo[6] = {-71.4, 0.001, 0.0, 43.0, 0.0, -0.001};
  // one NoData (0), the rest spanning [5, 50000]
  std::vector<uint16_t> samples = {0, 5, 12345, 50000};
  const QString path = writeTile(dir, "13_0_0.tif", w, h, geo, samples);

  GggsTile tile(path);
  ASSERT_TRUE(tile.valid());
  EXPECT_TRUE(tile.hasNoData());
  EXPECT_DOUBLE_EQ(tile.noData(), 0.0);
  EXPECT_DOUBLE_EQ(tile.dataMin(), 5.0);
  EXPECT_DOUBLE_EQ(tile.dataMax(), 50000.0);
}

// An all-NoData tile has a crossed range (min > max) so the layer can skip it.
TEST(GggsTileTest, AllNoDataHasCrossedRange)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const int w = 2, h = 2;
  const double geo[6] = {-71.4, 0.001, 0.0, 43.0, 0.0, -0.001};
  std::vector<uint16_t> samples(w * h, 0);
  const QString path = writeTile(dir, "13_1_1.tif", w, h, geo, samples);

  GggsTile tile(path);
  ASSERT_TRUE(tile.valid());           // dimensions known
  EXPECT_GT(tile.dataMin(), tile.dataMax());   // no valid samples
}

// A missing file degrades to invalid (no crash), like RasterLayer.
TEST(GggsTileTest, MissingFileInvalid)
{
  GggsTile tile("/nonexistent/path/13_0_0.tif");
  EXPECT_FALSE(tile.valid());
}

// The layer warps each mesh vertex via web_mercator::geoToMap on the CPU (in
// double precision — the warp is intentionally NOT done in the GPU shader, whose
// transcendentals x R caused a ~50 m latitude error). This pins geoToMap against
// the analytic EPSG:3857 formula: x = R*lambda (linear); y = R*asinh(tan phi).
TEST(GggsTileTest, GeoToMapMatchesWebMercator)
{
  const double R = web_mercator::earth_radius_at_equator;
  const double deg2rad = M_PI / 180.0;
  for(double lat : {-85.0, -43.0, -10.0, 0.0, 10.0, 42.99, 43.0, 85.0})
  {
    for(double lon : {-180.0, -71.39, 0.0, 71.39, 179.0})
    {
      // shader formula (see gggs_tile_layer.cpp kVertexShader)
      const double x = lon * deg2rad * R;
      const double t = std::tan(lat * deg2rad);
      const double y = std::log(t + std::sqrt(t * t + 1.0)) * R;

      const QPointF ref = web_mercator::geoToMap(QGeoCoordinate(lat, lon));
      // relative tolerance over the (large, ~1e7 m) Web-Mercator magnitudes
      const double scale = std::max(1.0, std::abs(ref.x()) + std::abs(ref.y()));
      EXPECT_NEAR(x, ref.x(), 1e-6 * scale) << "lon=" << lon;
      EXPECT_NEAR(y, ref.y(), 1e-6 * scale) << "lat=" << lat;
    }
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
