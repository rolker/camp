// [camp#172] Headless layer-level test for on-demand reload of evicted fine tiles (the
// ADR-0013 §"camp#172 hook", implemented with the ADR-0010 D6 reload-hysteresis guard).
// Pre-populate the on-disk cache, enable the layer against a small budget so some fine
// tiles are folded to overviews and evicted, then drive the reload seam
// (waitForReload(), the headless analogue of paint()'s demand-driven kick) and assert
// the evicted tiles come back at full resolution — and that the hysteresis gate blocks
// a reload when there is no budget headroom (no reload<->evict ping-pong). Reuses the
// offscreen-QApplication + Map harness; GL-free and ROS-free (null Node).

#include <gtest/gtest.h>

#include <cstdint>

#include <QApplication>
#include <QDir>
#include <QRectF>
#include <QSettings>
#include <QTemporaryDir>
#include <QUrl>

#include "marine_autonomy/gggs.h"
#include "marine_interfaces/msg/sonar_visualization_tile.hpp"
#include "marine_interfaces/msg/visualization_band.hpp"

#include "map/map.h"
#include "map/layer_list.h"
#include "ros/live_coverage/sonar_live_cache_layer.h"
#include "ros/live_coverage/sonar_live_tile.h"

namespace mi = marine_interfaces::msg;
using camp::map::Map;
using camp::ros::live_coverage::SonarLiveCacheLayer;
using camp::ros::live_coverage::SonarLiveTile;
using camp::ros::live_coverage::tileIndexFromGridIndex;

namespace
{

constexpr int kLevel = 10;
constexpr int kEdge = 8;

QString sanitizeNamespace(const QString& ns)
{
  return QString::fromLatin1(QUrl::toPercentEncoding(ns));
}

mi::SonarVisualizationTile makeUniformDepth(const gggs::GridIndex& grid, std::int16_t base)
{
  mi::SonarVisualizationTile msg;
  msg.header.stamp.sec = 100;
  msg.header.frame_id = "gggs";
  msg.index = tileIndexFromGridIndex(grid);
  msg.width = kEdge;
  msg.height = kEdge;
  msg.window_col = 0;
  msg.window_row = 0;
  msg.window_width = kEdge;
  msg.window_height = kEdge;

  mi::VisualizationBand band;
  band.name = "depth";
  band.dtype = mi::VisualizationBand::INT16;
  band.scale = 0.01;
  band.offset = 0.0;
  band.nodata = -32768.0;
  for(int i = 0; i < kEdge * kEdge; ++i)
  {
    band.data.push_back(static_cast<std::uint8_t>(base & 0xff));
    band.data.push_back(static_cast<std::uint8_t>((base >> 8) & 0xff));
  }
  msg.bands.push_back(band);
  return msg;
}

// Seed @p count fine tiles (a lat-varying column) into the layer's cache directory and
// return the layer, enabled and already trimmed to @p budget by warm-load eviction.
SonarLiveCacheLayer* seedAndEnable(const QString& cache_path, const QString& ns,
                                   std::size_t count, std::size_t budget,
                                   camp::map::LayerList* layers)
{
  QSettings().setValue("LiveTileCache/cache_dir", cache_path);
  QSettings().setValue("LiveTileCache/max_vram_bytes", qulonglong(budget));

  const QString tile_dir = QDir(cache_path).filePath(sanitizeNamespace(ns));
  QDir().mkpath(tile_dir);
  const gggs::Level level(kLevel);
  for(std::size_t i = 0; i < count; ++i)
  {
    const gggs::GridIndex grid = level.gridIndex(43.0 + 0.02 * i, -70.5);
    SonarLiveTile tile(grid, kEdge, kEdge);
    tile.applyPatch(makeUniformDepth(grid, static_cast<std::int16_t>(300 + i)));
    const QString path =
      QDir(tile_dir).filePath(QString("%1_%2_%3.tif")
                                .arg(static_cast<int>(grid.level()))
                                .arg(grid.row())
                                .arg(grid.column()));
    tile.writeToGeoTiff(path.toStdString());
  }

  auto* layer = new SonarLiveCacheLayer(layers, nullptr, ns);
  layer->enableLiveCoverage();   // incremental warm-load + evict-to-budget
  return layer;
}

// Seed a CONTIGUOUS @p rows x @p cols block of fine tiles (adjacent row/col) so parents
// actually share 4->1 up the pyramid — the overview pool stays geometrically bounded
// (unlike a sparse lat scatter, where each fine tile gets its own parent chain and the
// pyramid bloats). Used by the per-kick-cap test, which needs a predictable resident
// footprint to reason about the quarter-budget cap.
SonarLiveCacheLayer* seedContiguousAndEnable(const QString& cache_path, const QString& ns,
                                             int rows, int cols, std::size_t budget,
                                             camp::map::LayerList* layers)
{
  QSettings().setValue("LiveTileCache/cache_dir", cache_path);
  QSettings().setValue("LiveTileCache/max_vram_bytes", qulonglong(budget));

  const QString tile_dir = QDir(cache_path).filePath(sanitizeNamespace(ns));
  QDir().mkpath(tile_dir);
  const gggs::Level level(kLevel);
  const gggs::GridIndex anchor = level.gridIndex(43.0, -70.5);
  std::size_t n = 0;
  for(int r = 0; r < rows; ++r)
    for(int c = 0; c < cols; ++c)
    {
      marine_interfaces::msg::TileIndex ti;
      ti.level = static_cast<std::uint8_t>(kLevel);
      ti.row = anchor.row() + r;
      ti.col = anchor.column() + c;
      const gggs::GridIndex grid = camp::ros::live_coverage::gridIndexFromTileIndex(ti);
      SonarLiveTile tile(grid, kEdge, kEdge);
      tile.applyPatch(makeUniformDepth(grid, static_cast<std::int16_t>(300 + n)));
      const QString path =
        QDir(tile_dir).filePath(QString("%1_%2_%3.tif")
                                  .arg(static_cast<int>(grid.level()))
                                  .arg(grid.row())
                                  .arg(grid.column()));
      tile.writeToGeoTiff(path.toStdString());
      ++n;
    }

  auto* layer = new SonarLiveCacheLayer(layers, nullptr, ns);
  layer->enableLiveCoverage();
  return layer;
}

}  // namespace

// The whole point of #172: an evicted fine tile is reloaded from disk when the viewport
// re-enters its area, restoring full resolution (the coarse overview covered it in the
// meantime). Reload adds fine tiles, not overviews.
TEST(SonarLiveReload, ReloadEvictedTilesOnViewportChange)
{
  QSettings().clear();
  QTemporaryDir cache;
  ASSERT_TRUE(cache.isValid());

  constexpr std::size_t kTiles = 24;
  const std::size_t per_tile = static_cast<std::size_t>(kEdge) * kEdge * sizeof(float);
  const std::size_t budget = 4 * per_tile;

  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  auto* layer = seedAndEnable(cache.path(), "/reload_test", kTiles, budget, layers);

  // Warm-load eviction shed most fine tiles into the overview pyramid, recording them
  // as reload candidates.
  ASSERT_LT(layer->residentTileCount(), kTiles);
  ASSERT_GT(layer->evictedFineCount(), std::size_t(0));
  ASSERT_GT(layer->overviewTileCount(), std::size_t(0));
  const std::size_t resident_before = layer->residentTileCount();
  const std::size_t overview_before = layer->overviewTileCount();

  // Give the layer budget headroom so the D6 hysteresis gate permits the reload (and so
  // the reloaded tiles are not immediately re-evicted), then drive the reload for a
  // viewport covering the whole (evicted) survey extent.
  layer->setResidentBudgetForTest(std::size_t(1) << 30);
  const QRectF viewport = layer->sceneBounds();
  ASSERT_FALSE(viewport.isNull());
  layer->waitForReload(viewport);

  // The evicted fine tiles are resident again ...
  EXPECT_GT(layer->residentTileCount(), resident_before);
  // ... every attempted index cleared from the reload set ...
  EXPECT_EQ(layer->evictedFineCount(), std::size_t(0));
  // ... and reload added fine tiles, not overviews (the pyramid is unchanged; with
  // headroom no new eviction ran).
  EXPECT_EQ(layer->overviewTileCount(), overview_before);
}

// ADR-0010 D6: reload fires only with budget headroom (< 0.75x budget), so a reloaded
// tile can't immediately re-trigger eviction. With residency at the budget, the gate
// blocks the reload entirely — no ping-pong; raising the budget re-opens it.
TEST(SonarLiveReload, ReloadHysteresisPreventsPingPong)
{
  QSettings().clear();
  QTemporaryDir cache;
  ASSERT_TRUE(cache.isValid());

  constexpr std::size_t kTiles = 24;
  const std::size_t per_tile = static_cast<std::size_t>(kEdge) * kEdge * sizeof(float);
  const std::size_t budget = 4 * per_tile;

  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  auto* layer = seedAndEnable(cache.path(), "/hysteresis_test", kTiles, budget, layers);

  ASSERT_GT(layer->evictedFineCount(), std::size_t(0));
  const QRectF viewport = layer->sceneBounds();
  ASSERT_FALSE(viewport.isNull());

  // Pin the budget to the current residency: accountedBytes() == budget, so
  // accountedBytes() < 0.75*budget is false — the hysteresis gate must block the reload.
  const std::size_t no_headroom = layer->accountedBytes();
  ASSERT_GT(no_headroom, std::size_t(0));
  layer->setResidentBudgetForTest(no_headroom);
  const std::size_t resident_before = layer->residentTileCount();
  const std::size_t evicted_before = layer->evictedFineCount();

  layer->waitForReload(viewport);

  // No reload fired (no headroom) — residency and the evicted set are untouched: no
  // reload<->evict ping-pong.
  EXPECT_EQ(layer->residentTileCount(), resident_before);
  EXPECT_EQ(layer->evictedFineCount(), evicted_before);

  // Now grant headroom: the same viewport re-kicks (it was never recorded, since the
  // gate blocked the earlier kick) and the reload proceeds.
  layer->setResidentBudgetForTest(std::size_t(1) << 30);
  layer->waitForReload(viewport);
  EXPECT_GT(layer->residentTileCount(), resident_before);
  EXPECT_EQ(layer->evictedFineCount(), std::size_t(0));
  // The permitted reload stays within budget (here trivially — the headroom is vast; the
  // meaningful budget-vs-reload interaction is BudgetBoundedReloadCapsPerKickVolume).
  EXPECT_LE(layer->accountedBytes(), std::size_t(1) << 30);
}

// [camp#172 / ADR-0010 D6] A permitted reload never blows the budget, because kickReload
// caps each batch to a quarter-budget of fine tiles (the headroom the hysteresis gate
// guarantees) — a wide re-entry can't reload the whole survey in one over-budget spike.
// With a moderate budget (headroom to reload, but well below the whole evicted set) one
// waitForReload() brings SOME tiles back yet leaves the rest evicted, and residency stays
// within budget. NOTE: this drives the deterministic waitForReload() seam (paint()'s
// headless analogue, same gate); the live paint()->queued-`finished` path and its re-kick
// race are guarded by the reload_attempted_ gate exercised there.
TEST(SonarLiveReload, BudgetBoundedReloadCapsPerKickVolume)
{
  QSettings().clear();
  QTemporaryDir cache;
  ASSERT_TRUE(cache.isValid());

  // A contiguous 8x8 block so the overview pyramid stays geometrically bounded and the
  // resident footprint is predictable. Small initial budget so warm-load evicts most of
  // the 64 fine tiles into overviews (a large reload candidate set).
  const std::size_t per_tile = static_cast<std::size_t>(kEdge) * kEdge * sizeof(float);
  const std::size_t small_budget = 8 * per_tile;

  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  auto* layer =
    seedContiguousAndEnable(cache.path(), "/cap_test", 8, 8, small_budget, layers);

  const std::size_t evicted_before = layer->evictedFineCount();
  ASSERT_GT(evicted_before, std::size_t(4));   // a real reload candidate set
  const std::size_t resident_before = layer->residentTileCount();
  const QRectF viewport = layer->sceneBounds();
  ASSERT_FALSE(viewport.isNull());

  // Derive the reload budget from the measured post-eviction footprint so the test does
  // not hard-code (fragile) pyramid sizes. budget = 2*A: the hysteresis gate (A < 0.75*2A)
  // always opens, while the per-kick cap (0.25*2A = 0.5A worth of fine tiles) is smaller
  // than the evicted set — so one reload brings back a bounded batch, not the whole block.
  const std::size_t accounted_after_evict = layer->accountedBytes();
  ASSERT_GT(accounted_after_evict, std::size_t(0));
  const std::size_t reload_budget = 2 * accounted_after_evict;
  layer->setResidentBudgetForTest(reload_budget);
  layer->waitForReload(viewport);

  // Some tiles reloaded ...
  EXPECT_GT(layer->residentTileCount(), resident_before);
  // ... but NOT all of them — the cap bounded the batch, so evicted tiles remain for a
  // later frame (the whole survey did not reload in one over-budget kick).
  EXPECT_GT(layer->evictedFineCount(), std::size_t(0));
  EXPECT_LT(layer->evictedFineCount(), evicted_before);
  // ... and residency stayed within budget across the permitted reload (no spike).
  EXPECT_LE(layer->accountedBytes(), reload_budget);
}

int main(int argc, char** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  QCoreApplication::setOrganizationName("camp_test");
  QCoreApplication::setApplicationName("test_sonar_live_reload");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
