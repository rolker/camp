// [camp#195 / uma-ADR-0013 D4] Headless tests for GggsTileLayer's
// viewport-scoped retention and residency budget.
//
// The defect these guard is an OOM after prolonged panning: before camp#195 the
// only code path that ever freed a GGGS tile was tilesReady()'s level-switch
// release, so at a fixed zoom every tile ever scrolled across stayed resident
// for the session. The portable assertion is a BOUNDED RESIDENT COUNT while the
// visited count grows — the same shape as the in-repo
// test/test_map_tiles_eviction.cpp (camp#98) and cube_bathymetry's
// test_tile_eviction_rss.cpp.
//
// GL-free by construction (like test_gggs_rescan/test_gggs_elevation): the layer
// is driven through setLodForTest() + waitForLoad() (pure GDAL RasterIO) and
// refreshResidencyForTest(), never through paint(). Consequence, and a
// deliberate coverage gap recorded in camp-ADR-0014: with no GL context
// releaseTiles() exercises only the CPU half of the release (releaseGL() is a
// no-op because no texture can exist), so the GL-texture half of the ~7 MiB
// per-tile cost is not covered here.
//
// Residency is observed through getElevation() (a point sample of the resident
// CPU buffer -> NaN once the tile is released) and pixelsLoadedCount()/
// residentTileCount().

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

#include <gdal_priv.h>

#include <QApplication>
#include <QFile>
#include <QGeoCoordinate>
#include <QRectF>
#include <QTemporaryDir>

#include "map/map.h"
#include "map/layer_list.h"
#include "map_view/web_mercator.h"
#include "raster/gggs_tile_layer.h"

namespace
{

// Tile geometry: 16x16 cells of 0.0001 deg, so a tile spans 0.0016 deg.
constexpr int kTileEdge = 16;
constexpr double kCell = 0.0001;
constexpr double kTileSpan = kCell * kTileEdge;   // 0.0016 deg
constexpr double kLat0 = 43.0;                    // north edge of every tile
constexpr double kLon0 = -71.4;                   // west edge of tile 0

// Per-tile resident cost as the layer accounts it: the Float32 CPU buffer plus
// an R32F texture of the same extent (gggs_tile.cpp retains the CPU copy past
// the GPU upload for camp#180). Tests express their budget in these units so
// they do not depend on the 960x960 production tile size.
constexpr std::size_t kTileBytes =
  std::size_t(kTileEdge) * kTileEdge * sizeof(float) * 2;

// Write a north-up Float32 GeoTIFF filled with @p value (NoData 9999) at the
// given NW corner. @p name carries the GGGS `<level>_<row>_<col>` convention.
QString writeTile(const QTemporaryDir& dir, const QString& name, double lon0,
                  double lat0, double cell, int edge, float value)
{
  if(GDALGetDriverCount() == 0)
    GDALAllRegister();
  const double geo[6] = {lon0, cell, 0.0, lat0, 0.0, -cell};
  const QString path = dir.filePath(name);
  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GTiff");
  EXPECT_NE(driver, nullptr);
  if(!driver)
    return QString();
  GDALDataset* ds = driver->Create(path.toUtf8().constData(), edge, edge, 1,
                                   GDT_Float32, nullptr);
  EXPECT_NE(ds, nullptr);
  if(!ds)
    return QString();
  ds->SetGeoTransform(const_cast<double*>(geo));
  GDALRasterBand* band = ds->GetRasterBand(1);
  band->SetNoDataValue(9999.0);
  std::vector<float> samples(static_cast<size_t>(edge) * edge, value);
  const CPLErr err = band->RasterIO(GF_Write, 0, 0, edge, edge, samples.data(),
                                    edge, edge, GDT_Float32, 0, 0);
  GDALClose(ds);
  return err == CE_None ? path : QString();
}

// A west-to-east strip of @p count level-13 tiles, tile k carrying value k+1 so
// a sample identifies which tile answered.
void writeStrip(const QTemporaryDir& dir, int count)
{
  for(int k = 0; k < count; ++k)
    ASSERT_FALSE(writeTile(dir, QString("13_0_%1.tif").arg(k),
                           kLon0 + k * kTileSpan, kLat0, kCell, kTileEdge,
                           float(k + 1))
                   .isEmpty());
}

// A point comfortably inside strip tile @p k.
QGeoCoordinate insideTile(int k)
{
  return QGeoCoordinate(kLat0 - kTileSpan / 2,
                        kLon0 + k * kTileSpan + kTileSpan / 2);
}

// Web-Mercator scene rect spanning strip tiles [first, last] — the load/paint
// viewport the layer filters and protects against.
QRectF viewportOverTiles(int first, int last)
{
  const QPointF lo = web_mercator::geoToMap(
    QGeoCoordinate(kLat0 - kTileSpan, kLon0 + first * kTileSpan));
  const QPointF hi = web_mercator::geoToMap(
    QGeoCoordinate(kLat0, kLon0 + (last + 1) * kTileSpan));
  return QRectF(lo, hi).normalized();
}

// One pan step: move the viewport, let the demand-driven loader fill it, then
// run the residency pass (paint() would schedule it; a headless test has no
// event-loop turn, so it drives the pass directly).
void panTo(camp::raster::GggsTileLayer* layer, int first, int last)
{
  layer->setLodForTest(13, viewportOverTiles(first, last));
  layer->waitForLoad();
  layer->refreshResidencyForTest();
}

}  // namespace

// THE regression: panning a small viewport across a strip far wider than the
// budget must keep residency bounded, while the visited footprint grows. Before
// camp#195 every visited tile stayed resident for the session.
TEST(GggsEvictionTest, PanAcrossStripStaysWithinBudget)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const int kTiles = 12;
  writeStrip(dir, kTiles);

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());
  layer->setResidentBudgetBytesForTest(4 * kTileBytes);   // 4 tiles

  for(int k = 0; k + 1 < kTiles; ++k)
  {
    panTo(layer, k, k + 1);
    EXPECT_LE(layer->residentTileCount(), 4u)
      << "residency exceeded the budget at pan step " << k;
    // The two tiles under the viewport are the current working set and must be
    // present at every step — a budget that evicts what it is drawing thrashes.
    EXPECT_FALSE(std::isnan(layer->getElevation(insideTile(k))))
      << "an in-viewport tile was evicted at pan step " << k;
    EXPECT_FALSE(std::isnan(layer->getElevation(insideTile(k + 1))))
      << "an in-viewport tile was evicted at pan step " << k;
  }

  EXPECT_LT(layer->pixelsLoadedCount(13), kTiles)
    << "every visited tile is still resident — nothing was ever evicted";
}

// The far end of the strip is released once the view has moved on, and the
// released tile's depth-at-cursor readout degrades to NaN. This is the
// consequence camp#195's plan named and camp-ADR-0014 records: getElevation()
// answers over the resident set, and the cursor is always in the viewport, so
// the tile under it is protected.
TEST(GggsEvictionTest, PannedAwayAreaIsReleasedAndReadsNaN)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  writeStrip(dir, 10);

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());
  layer->setResidentBudgetBytesForTest(3 * kTileBytes);

  panTo(layer, 0, 1);
  ASSERT_FALSE(std::isnan(layer->getElevation(insideTile(0))));

  for(int k = 1; k + 1 < 10; ++k)
    panTo(layer, k, k + 1);

  EXPECT_TRUE(std::isnan(layer->getElevation(insideTile(0))))
    << "the far end of the pan track is still resident";
  EXPECT_FALSE(std::isnan(layer->getElevation(insideTile(9))))
    << "the tile under the current viewport must never be evicted";
}

// Eviction is only affordable because the reload path is free: the tiles are
// files on disk, and the existing demand-driven loader re-reads them when the
// viewport comes back (which is why camp#195 drops rather than folding to a
// parent, unlike camp-ADR-0010's SonarLiveCacheLayer).
TEST(GggsEvictionTest, PanBackReloadsAnEvictedTile)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  writeStrip(dir, 8);

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());
  layer->setResidentBudgetBytesForTest(3 * kTileBytes);

  panTo(layer, 0, 1);
  ASSERT_FLOAT_EQ(layer->getElevation(insideTile(0)), 1.0f);

  for(int k = 1; k + 1 < 8; ++k)
    panTo(layer, k, k + 1);
  ASSERT_TRUE(std::isnan(layer->getElevation(insideTile(0))));

  // Pan back over the released tile.
  for(int k = 6; k >= 0; --k)
    panTo(layer, k, k + 1);
  EXPECT_FLOAT_EQ(layer->getElevation(insideTile(0)), 1.0f)
    << "an evicted tile did not reload when the viewport returned to it";
}

// `GggsTileLayers/max_resident_bytes = 0` reproduces the pre-camp#195 behaviour
// exactly — the escape hatch camp-ADR-0006 D2 / camp#117 require for any
// operator-facing default.
TEST(GggsEvictionTest, ZeroBudgetDisablesEviction)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const int kTiles = 8;
  writeStrip(dir, kTiles);

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());
  layer->setResidentBudgetBytesForTest(0);

  for(int k = 0; k + 1 < kTiles; ++k)
    panTo(layer, k, k + 1);

  EXPECT_EQ(layer->pixelsLoadedCount(13), kTiles)
    << "eviction ran with the budget disabled";
}

// The zoom-out floor: on a real ladder the coarsest available level is exempt,
// so panning away from the region it covers does not leave a zoomed-out view
// with nothing to draw. The coarse tile here covers only the WEST half of the
// strip, so once the view is over the east half it is off-viewport and would
// otherwise be the farthest, stalest candidate of all.
TEST(GggsEvictionTest, CoarsestLadderLevelIsExemptFromEviction)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  writeStrip(dir, 10);
  // Level 0 tile spanning strip tiles 0..3 (4 x kTileSpan wide).
  ASSERT_FALSE(writeTile(dir, "0_0_0.tif", kLon0, kLat0, kCell * 4, kTileEdge,
                         500.0f)
                 .isEmpty());

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());
  ASSERT_EQ(layer->availableLevels().size(), 2u);
  layer->setResidentBudgetBytesForTest(3 * kTileBytes);

  panTo(layer, 0, 1);
  ASSERT_EQ(layer->pixelsLoadedCount(0), 1) << "the coarse tile never loaded";

  for(int k = 1; k + 1 < 10; ++k)
    panTo(layer, k, k + 1);

  EXPECT_EQ(layer->pixelsLoadedCount(0), 1)
    << "the coarsest ladder level was evicted — a zoom-out would have nothing "
       "to draw";
  EXPECT_LE(layer->pixelsLoadedCount(13), 3)
    << "the fine level is not bounded by the budget";
}

// camp#194's hole-coverage rule survives the budget: a finer tile that is the
// only usable coverage over a footprint whose selected-or-coarser tile failed to
// read must not be evicted out from under the operator while it is on screen. It
// is doubly load-bearing here because such a tile cannot reload —
// loadTilesWorker() skips levels finer than the selection.
TEST(GggsEvictionTest, InViewHoleCoverageIsProtected)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  writeStrip(dir, 6);
  // A coarse (level 10) tile over strip tiles 0..3, which we then break.
  const QString coarse_path =
    writeTile(dir, "10_0_0.tif", kLon0, kLat0, kCell * 4, kTileEdge, 500.0f);
  ASSERT_FALSE(coarse_path.isEmpty());

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());
  layer->setResidentBudgetBytesForTest(2 * kTileBytes);

  // Break the coarse tile AFTER its metadata scan: it stays valid() but its
  // pixels can never be read, so loadPixels() latches the sticky failure.
  {
    QFile file(coarse_path);
    ASSERT_TRUE(file.open(QIODevice::WriteOnly | QIODevice::Truncate));
    ASSERT_TRUE(file.resize(0));
  }

  // Load fine tiles 0..3 over the coarse footprint at a fine selection.
  panTo(layer, 0, 3);
  ASSERT_FLOAT_EQ(layer->getElevation(insideTile(0)), 1.0f);
  ASSERT_TRUE(layer->status().contains("failed")) << layer->status().toStdString();

  // Zoom OUT to the coarse level over tiles 0..1: the coarse tile is a hole, and
  // the resident fine tiles are the only coverage over it. Tiles 0..1 are on
  // screen; tiles 2..3 are not, and residency is over budget.
  layer->setLodForTest(10, viewportOverTiles(0, 1));
  layer->waitForLoad();
  layer->refreshResidencyForTest();

  EXPECT_FALSE(std::isnan(layer->getElevation(insideTile(0))))
    << "the only coverage over a failed tile was evicted while on screen";
  EXPECT_FALSE(std::isnan(layer->getElevation(insideTile(1))))
    << "the only coverage over a failed tile was evicted while on screen";
  EXPECT_LE(layer->residentTileCount(), 2u)
    << "the off-screen hole coverage was exempted from the budget rather than "
       "evicted last";
  // An evicted hole coverer cannot reload (loadTilesWorker skips levels finer
  // than the selection), so the loss must be reported with its recovery action
  // rather than disappearing silently.
  EXPECT_TRUE(layer->status().contains("Rescan"))
    << "released hole coverage was not reported: "
    << layer->status().toStdString();
}

// uma-ADR-0013 D4: "the budget must exceed [the current-frame set] or the system
// thrashes by construction". When one viewport's own working set is larger than
// the byte target, the cap is FLOORED at that set — the layer keeps drawing and
// reports the over-budget state instead of evicting what it is rendering.
// (Relaxing the quality target under this pressure is camp#197.)
TEST(GggsEvictionTest, WorkingSetAboveBudgetFloorsTheCapAndReportsIt)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  writeStrip(dir, 6);

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());
  layer->setResidentBudgetBytesForTest(kTileBytes);   // 1 tile

  panTo(layer, 0, 3);   // four tiles in view, budget of one

  EXPECT_GE(layer->residentTileCount(), 4u)
    << "the visible working set was evicted — the cap is not floored at it";
  for(int k = 0; k < 4; ++k)
    EXPECT_FALSE(std::isnan(layer->getElevation(insideTile(k))));
  EXPECT_TRUE(layer->status().contains("over the tile budget"))
    << "the over-budget state was not reported: " << layer->status().toStdString();

  // Zooming back to a set that fits clears the report — the status is composed
  // from live state, not latched.
  panTo(layer, 0, 0);
  EXPECT_FALSE(layer->status().contains("over the tile budget"))
    << layer->status().toStdString();
}

int main(int argc, char** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  // [camp#117] A test org/app name keeps Map's ctor QSettings seed — and the
  // layer's GggsTileLayers/max_resident_bytes read — out of the developer's real
  // camp settings.
  QCoreApplication::setOrganizationName("camp_test");
  QCoreApplication::setApplicationName("test_gggs_eviction");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
