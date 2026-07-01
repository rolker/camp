// [camp#160] Headless layer-level test for bounded eviction + the overview pyramid.
// Pre-populate the on-disk cache with more fine tiles than the resident budget,
// enable the layer (null ROS node, no GL), and assert the resident fine-tile set is
// trimmed to the budget while the evicted tiles are folded into resident overview
// tiles (so coverage is never lost). Reuses the offscreen-QApplication + Map harness
// from test_range_persist; GL-free (renderImage() is never called) and ROS-free
// (null Node -> no subscription).

#include <gtest/gtest.h>

#include <cstdint>

#include <QApplication>
#include <QDir>
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

// Percent-encode a namespace exactly as SonarLiveCacheLayer::sanitize() does, so the
// test writes tiles into the same cache sub-directory the layer will read.
QString sanitizeNamespace(const QString& ns)
{
  return QString::fromLatin1(QUrl::toPercentEncoding(ns));
}

// A uniform full-tile INT16 "depth" patch (value = base * 0.01) for @p grid.
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

}  // namespace

TEST(SonarLiveEviction, WarmLoadTrimsToBudgetAndBuildsOverviews)
{
  QSettings().clear();
  QTemporaryDir cache;
  ASSERT_TRUE(cache.isValid());

  const QString ns = "/evict_test";
  constexpr std::size_t kTiles = 24;
  // Each fine tile is kEdge*kEdge floats (one band) resident. Budget = 4 tiles' worth
  // so warm-load must evict most of them.
  const std::size_t per_tile = static_cast<std::size_t>(kEdge) * kEdge * sizeof(float);
  const std::size_t budget = 4 * per_tile;

  QSettings().setValue("LiveTileCache/cache_dir", cache.path());
  QSettings().setValue("LiveTileCache/max_vram_bytes", qulonglong(budget));

  // Write kTiles distinct fine tiles into the layer's (sanitized) cache directory.
  const QString tile_dir = QDir(cache.path()).filePath(sanitizeNamespace(ns));
  ASSERT_TRUE(QDir().mkpath(tile_dir));
  const gggs::Level level(kLevel);
  std::size_t written = 0;
  for(std::size_t i = 0; i < kTiles; ++i)
  {
    const gggs::GridIndex grid = level.gridIndex(43.0 + 0.02 * i, -70.5);
    ASSERT_TRUE(grid.valid());
    SonarLiveTile tile(grid, kEdge, kEdge);
    tile.applyPatch(makeUniformDepth(grid, static_cast<std::int16_t>(300 + i)));
    const QString path =
      QDir(tile_dir).filePath(QString("%1_%2_%3.tif")
                                .arg(static_cast<int>(grid.level()))
                                .arg(grid.row())
                                .arg(grid.column()));
    ASSERT_TRUE(tile.writeToGeoTiff(path.toStdString()));
    ++written;
  }
  ASSERT_EQ(written, kTiles);

  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  auto* layer = new SonarLiveCacheLayer(layers, nullptr, ns);

  layer->enableLiveCoverage();   // incremental warm-load + two-phase evict-to-budget

  // Resident fine tiles are bounded (evicted well below the 24 loaded)...
  EXPECT_LT(layer->residentTileCount(), kTiles);
  EXPECT_LE(layer->residentTileCount(), std::size_t(4));
  // ...evicted tiles were folded into resident overview parents, so a zoomed-out
  // view still has coverage (the coarse apex is protected, never fully evicted)...
  EXPECT_GT(layer->overviewTileCount(), std::size_t(0));
  // ...and the overviews are a LOCAL derived product: every fine tile's possession is
  // kept (markHave, never dropped) and NO overview leaks into the reconciler, so the
  // held count is exactly the fine tiles loaded — asserts both the no-drop / no-churn
  // policy (D2) and the reconciler-isolation invariant (D4).
  EXPECT_EQ(layer->reconcilerHeldCount(), kTiles);
}

int main(int argc, char** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
