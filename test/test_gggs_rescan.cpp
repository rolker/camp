// [camp#104] Regression tests for GggsTileLayer::rescan() — the manual refresh
// wired to the "Rescan" context-menu action (the stopgap for the retired
// QFileSystemWatcher; ADR-0005).
//
// These pin down rescan()'s add / no-op CONTRACT, the surface a Round-2 review
// must-fix reshaped: rescan() now computes the new-tile set FIRST and only
// disturbs the layer (and any in-flight async pixel load) when there is at least
// one tile to add; a rescan that finds nothing is a non-destructive no-op
// returning false.
//
// Non-GL / non-async by construction: rescan()'s directory scan + extent merge
// run synchronously on the calling thread (the pixel load is only kicked from
// paint()/waitForLoad(), neither of which runs here), so these assert
// deterministically without an offscreen GL context or a running load worker.
//
// The ONE behavior these cannot pin down is the in-flight-load case the must-fix
// directly targets — rescan-finds-nothing must leave a still-running pixel-load
// worker alone. Forcing a deterministically in-flight worker would need a
// worker-pause seam plus a pixelsLoaded() accessor the production class
// deliberately does not expose; a timing-based variant would only detect the bug
// on the rounds where tiles happen to load slowly (false assurance otherwise).
// That path is covered by code review (see progress.md). What IS guarded here is
// the contract the fix preserves: no-new -> false + extent untouched; new ->
// true + extent grows.

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include <gdal_priv.h>

#include <QApplication>
#include <QTemporaryDir>

#include "map/map.h"
#include "map/layer_list.h"
#include "raster/gggs_tile_layer.h"

namespace
{

// Write a north-up WGS84 GeoTIFF tile so GggsTile::valid() is true and its extent
// comes from the geotransform. lon0/lat0 place the NW corner, so successive tiles
// can be given disjoint extents (to observe the layer extent grow on rescan).
QString writeTile(const QTemporaryDir& dir, const QString& name,
                  double lon0, double lat0)
{
  if(GDALGetDriverCount() == 0)
    GDALAllRegister();
  const int w = 16, h = 16;
  const double geo[6] = {lon0, 0.0001, 0.0, lat0, 0.0, -0.0001};
  const QString path = dir.filePath(name);
  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GTiff");
  GDALDataset* ds = driver->Create(path.toUtf8().constData(), w, h, 1, GDT_UInt16, nullptr);
  ds->SetGeoTransform(const_cast<double*>(geo));
  GDALRasterBand* band = ds->GetRasterBand(1);
  band->SetNoDataValue(0);
  std::vector<uint16_t> samples(static_cast<size_t>(w) * h, 8000);
  const CPLErr err = band->RasterIO(GF_Write, 0, 0, w, h, samples.data(),
                                    w, h, GDT_UInt16, 0, 0);
  GDALClose(ds);
  return err == CE_None ? path : QString();
}

}  // namespace

// rescan() with nothing new is a non-destructive no-op: returns false and leaves
// the layer's extent untouched. This is the path the Round-2 must-fix made safe —
// it must NOT abort/disturb the layer (or a live load) when there is nothing to
// add. Idempotent: a second rescan with no change also returns false.
TEST(GggsRescanTest, NoNewTilesIsNoOp)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  ASSERT_FALSE(writeTile(dir, "13_0_0.tif", -71.40, 43.00).isEmpty());

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());
  const QRectF bounds_before = layer->sceneBounds();

  EXPECT_FALSE(layer->rescan());                  // nothing new
  EXPECT_EQ(layer->sceneBounds(), bounds_before); // extent untouched
  EXPECT_FALSE(layer->rescan());                  // idempotent
  EXPECT_EQ(layer->sceneBounds(), bounds_before);
}

// rescan() picks up a newly-landed tile: returns true and unions its extent into
// the layer bounds (west edge unchanged since the original tile is westmost; the
// extent widens eastward). A subsequent rescan with nothing new returns false.
TEST(GggsRescanTest, PicksUpNewlyLandedTile)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  ASSERT_FALSE(writeTile(dir, "13_0_0.tif", -71.40, 43.00).isEmpty());

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());
  const QRectF bounds_before = layer->sceneBounds();

  // A second tile to the east (disjoint extent) lands after the layer loaded.
  ASSERT_FALSE(writeTile(dir, "13_0_1.tif", -71.39, 43.00).isEmpty());

  EXPECT_TRUE(layer->rescan());                   // tile added
  EXPECT_DOUBLE_EQ(layer->sceneBounds().left(), bounds_before.left());  // westmost unchanged
  EXPECT_GT(layer->sceneBounds().width(), bounds_before.width());       // grew eastward
  EXPECT_FALSE(layer->rescan());                  // now nothing new
}

// [camp#112] loadDirectory() filters companion tiles (`_time`/`_source`): a tile
// directory holding the base value tile plus its companions enumerates EXACTLY
// the base tile. The companions are written at a disjoint (eastward) extent, so
// if they were (wrongly) loaded as tiles the layer's sceneBounds would widen —
// the layer instead matches a base-only layer's extent exactly.
TEST(GggsRescanTest, CompanionTilesAreNotLoaded)
{
  QTemporaryDir base_only;
  ASSERT_TRUE(base_only.isValid());
  ASSERT_FALSE(writeTile(base_only, "13_0_0.tif", -71.40, 43.00).isEmpty());

  camp::map::Map base_map;
  auto* base_layer =
    new camp::raster::GggsTileLayer(base_map.topLevelLayers(), base_only.path());
  ASSERT_TRUE(base_layer->valid());
  const QRectF base_bounds = base_layer->sceneBounds();

  QTemporaryDir with_companions;
  ASSERT_TRUE(with_companions.isValid());
  ASSERT_FALSE(writeTile(with_companions, "13_0_0.tif", -71.40, 43.00).isEmpty());
  // Companions at a disjoint eastward extent: a real GeoTIFF, excluded by NAME.
  ASSERT_FALSE(writeTile(with_companions, "13_0_0_time.tif", -71.39, 43.00).isEmpty());
  ASSERT_FALSE(writeTile(with_companions, "13_0_0_source.tif", -71.38, 43.00).isEmpty());

  camp::map::Map map;
  auto* layer =
    new camp::raster::GggsTileLayer(map.topLevelLayers(), with_companions.path());
  ASSERT_TRUE(layer->valid());
  // Only the base tile counted: same extent as the base-only layer (companions'
  // eastward extents did not widen it).
  EXPECT_EQ(layer->sceneBounds(), base_bounds)
      << "companion tiles must not be loaded as renderable tiles";
}

// [camp#112] rescan() filters companion tiles too: companions landing after the
// initial load are not treated as new renderable tiles, so rescan() no-ops
// (returns false) and leaves the extent untouched.
TEST(GggsRescanTest, RescanIgnoresCompanionTiles)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  ASSERT_FALSE(writeTile(dir, "13_0_0.tif", -71.40, 43.00).isEmpty());

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());
  const QRectF bounds_before = layer->sceneBounds();

  // Companions for the existing tile land afterward (disjoint eastward extents).
  ASSERT_FALSE(writeTile(dir, "13_0_0_time.tif", -71.39, 43.00).isEmpty());
  ASSERT_FALSE(writeTile(dir, "13_0_0_source.tif", -71.38, 43.00).isEmpty());

  EXPECT_FALSE(layer->rescan());                  // companions are not new tiles
  EXPECT_EQ(layer->sceneBounds(), bounds_before); // extent untouched
}

int main(int argc, char** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
