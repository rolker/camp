// [camp#180] Headless tests for GggsTileLayer::getElevation() — the store-layer
// depth-at-cursor query wired into ProjectView via
// AutonomousVehicleProject::getStoreElevation().
//
// Non-GL by construction (mirrors test_gggs_rescan.cpp): the layer is built and
// its pixels are loaded via waitForLoad() (pure GDAL RasterIO — no GL context,
// no paint()). Because waitForLoad() never calls texture(), the tiles' CPU
// buffers stay resident, which is exactly the state sampleAt() indexes — so
// getElevation() is exercised end-to-end without a window.
//
// Covered: the extent filter (a point outside every tile -> NaN), the
// descending-level sort (when two tiles overlap the point, the finest / highest
// level wins regardless of the QDir::Name load order), and the all-NoData case
// (a covering tile with no valid sample -> NaN).

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include <gdal_priv.h>

#include <QApplication>
#include <QGeoCoordinate>
#include <QTemporaryDir>

#include "map/map.h"
#include "map/layer_list.h"
#include "raster/gggs_tile_layer.h"

namespace
{

// Write a north-up Float32 GeoTIFF tile filled with a single @p value (NoData
// 9999), named @p name (`<level>_<row>_<col>.tif`) with NW corner (@p lon0,
// @p lat0). A uniform fill makes the sampled value identify which tile answered.
QString writeUniformTile(const QTemporaryDir& dir, const QString& name,
                         double lon0, double lat0, float value)
{
  if(GDALGetDriverCount() == 0)
    GDALAllRegister();
  const int w = 16, h = 16;
  const double geo[6] = {lon0, 0.0001, 0.0, lat0, 0.0, -0.0001};
  const QString path = dir.filePath(name);
  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GTiff");
  EXPECT_NE(driver, nullptr);
  if(!driver)
    return QString();
  GDALDataset* ds = driver->Create(path.toUtf8().constData(), w, h, 1,
                                   GDT_Float32, nullptr);
  EXPECT_NE(ds, nullptr);
  if(!ds)
    return QString();
  ds->SetGeoTransform(const_cast<double*>(geo));
  GDALRasterBand* band = ds->GetRasterBand(1);
  band->SetNoDataValue(9999.0);
  std::vector<float> samples(static_cast<size_t>(w) * h, value);
  const CPLErr err = band->RasterIO(GF_Write, 0, 0, w, h, samples.data(),
                                    w, h, GDT_Float32, 0, 0);
  GDALClose(ds);
  return err == CE_None ? path : QString();
}

// A geographic point comfortably inside the tile whose NW corner is (lon0, lat0)
// (the 16x16, 0.0001-deg tiles span lon0..lon0+0.0016, lat0-0.0016..lat0).
QGeoCoordinate insidePoint(double lon0, double lat0)
{
  return QGeoCoordinate(lat0 - 0.0008, lon0 + 0.0008);
}

}  // namespace

// The finest (highest-level) tile covering the point wins even though QDir::Name
// loads the coarser "10_..." tile first: without the descending-level sort the
// walk would return the coarse value.
TEST(GggsElevationTest, FinestCoveringTileWins)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  // Two tiles at the SAME extent, distinct levels + values. "10_0_0" sorts before
  // "13_0_0" by name, so load order alone would answer with the coarse value.
  ASSERT_FALSE(writeUniformTile(dir, "10_0_0.tif", -71.40, 43.00, 100.0f).isEmpty());
  ASSERT_FALSE(writeUniformTile(dir, "13_0_0.tif", -71.40, 43.00, 200.0f).isEmpty());

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());
  layer->waitForLoad();

  const float e = layer->getElevation(insidePoint(-71.40, 43.00));
  EXPECT_FLOAT_EQ(e, 200.0f) << "finest (level 13) tile must win over level 10";
}

// A point outside every tile's extent -> NaN (the extent filter drops all tiles).
TEST(GggsElevationTest, OutOfExtentIsNaN)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  ASSERT_FALSE(writeUniformTile(dir, "13_0_0.tif", -71.40, 43.00, 200.0f).isEmpty());

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());
  layer->waitForLoad();

  // Far from the tile (a whole degree east) -> no covering tile.
  EXPECT_TRUE(std::isnan(layer->getElevation(QGeoCoordinate(43.00, -70.40))));
}

// A covering tile whose samples are all NoData yields no value -> NaN.
TEST(GggsElevationTest, AllNoDataCoveringTileIsNaN)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  // Every sample equals the NoData sentinel (9999) -> masked everywhere.
  ASSERT_FALSE(writeUniformTile(dir, "13_0_0.tif", -71.40, 43.00, 9999.0f).isEmpty());

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());
  layer->waitForLoad();

  EXPECT_TRUE(std::isnan(layer->getElevation(insidePoint(-71.40, 43.00))));
}

int main(int argc, char** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  // [camp#117] A test org/app name keeps Map's ctor QSettings seed out of the
  // developer's real camp settings.
  QCoreApplication::setOrganizationName("camp_test");
  QCoreApplication::setApplicationName("test_gggs_elevation");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
