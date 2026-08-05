// [camp#169] Headless regression test: the latched catalog must survive a
// disabled window and drive a reconcile on (re-)enable. The catalog
// subscription is transient-local depth-1 — its single latched sample is
// delivered exactly once. Before the fix, handleCatalog() discarded it while
// `enabled_` was false and enableLiveCoverage() never reconciled, so whether
// requests ever fired after a camp restart was a startup timing race
// (2026-07-23 field incident: live coverage stale all session, zero
// publishers on coverage_requests).
//
// Scenario: warm-load N cached tiles, deliver an EMPTY catalog (prune-all)
// while disabled, re-enable. With the fix, the buffered catalog replays on
// enable and anti-entropy prunes all N resident tiles; without it, the catalog
// is discarded and the N stale tiles stay resident forever.
//
// handleCatalog is a private slot: invoked by name via QMetaObject
// (DirectConnection — no metatype registration needed), mirroring how the ROS
// callback marshals onto the GUI thread. Offscreen QApplication + Map harness
// as test_sonar_live_eviction; GL-free, ROS-free (null Node).

#include <gtest/gtest.h>

#include <cstdint>

#include <QApplication>
#include <QDir>
#include <QMetaObject>
#include <QSettings>
#include <QTemporaryDir>
#include <QUrl>

#include "marine_autonomy/gggs.h"
#include "marine_interfaces/msg/sonar_visualization_tile.hpp"
#include "marine_interfaces/msg/tile_catalog.hpp"
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

bool deliverCatalog(SonarLiveCacheLayer* layer, const mi::TileCatalog& catalog)
{
  // By-name direct invocation reaches the private slot exactly as the ROS
  // callback's GUI-thread marshal does.
  return QMetaObject::invokeMethod(layer, "handleCatalog", Qt::DirectConnection,
                                   Q_ARG(marine_interfaces::msg::TileCatalog, catalog));
}

}  // namespace

TEST(SonarLiveCatalogResume, CatalogBufferedWhileDisabledReconcilesOnEnable)
{
  QSettings().clear();
  QTemporaryDir cache;
  ASSERT_TRUE(cache.isValid());

  const QString ns = "/catalog_resume_test";
  constexpr std::size_t kTiles = 3;
  // Generous budget: eviction must not interfere with this test.
  const std::size_t per_tile = static_cast<std::size_t>(kEdge) * kEdge * sizeof(float);
  QSettings().setValue("LiveTileCache/cache_dir", cache.path());
  QSettings().setValue("LiveTileCache/max_vram_bytes", qulonglong(100 * per_tile));

  const QString tile_dir = QDir(cache.path()).filePath(sanitizeNamespace(ns));
  ASSERT_TRUE(QDir().mkpath(tile_dir));
  const gggs::Level level(kLevel);
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
  }

  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  auto* layer = new SonarLiveCacheLayer(layers, nullptr, ns);

  layer->enableLiveCoverage();
  ASSERT_EQ(layer->residentTileCount(), kTiles);   // warm-load seeded the cache

  layer->disableLiveCoverage();

  // The latched catalog lands while disabled. Empty catalog = the boat holds
  // nothing = prune-all on the next reconcile. Its generation time must be
  // NONZERO: warm-loaded tiles are seeded at reconciler version 0 (markHave
  // can't recover the original stamp from a cached GeoTIFF), and the
  // reconciler's timestamp gate (ADR-0008 D4b) prunes only tiles with
  // version < generation_time — a zero-stamped catalog prunes nothing
  // (0 < 0 is false).
  mi::TileCatalog empty_catalog;
  empty_catalog.header.stamp.sec = 200;
  ASSERT_TRUE(deliverCatalog(layer, empty_catalog));
  // Buffered, not acted on: still resident while disabled.
  EXPECT_EQ(layer->residentTileCount(), kTiles);

  // Re-enable: the buffered catalog must replay and anti-entropy must prune
  // all resident tiles. Without the fix the catalog was discarded above and
  // this stays at kTiles forever.
  layer->enableLiveCoverage();
  EXPECT_EQ(layer->residentTileCount(), std::size_t(0));
}

int main(int argc, char** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  QCoreApplication::setOrganizationName("camp_test");
  QCoreApplication::setApplicationName("test_sonar_live_catalog_resume");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
