// [#99 Phase 2] Coverage for the MapTiles auto-refresh timer and its bounded-
// memory eviction — the coordination artifact for the #98 tile-lifecycle risk.
//
// The weather-radar overlay reloads its tiles every ~5 minutes. The refresh path
// shares the tile lifecycle implicated in the #98 map zoom/pan OOM crash, so the
// load-bearing test here is that repeated refresh cycles do NOT accumulate Tile
// objects: setLayout() (called from onRefreshTimer) must delete the old tiles and
// rebuild, keeping the count bounded across N cycles.
//
// Per the plan review (must-fix): we deliberately do NOT count wall-clock timer
// fires (flaky under CI load). Instead we assert timer *configuration*
// (isActive/interval/disabled) and verify one DETERMINISTIC refresh by invoking
// the refresh slot directly.
//
// Access mechanism (named per plan-review finding #2): MapTiles::tiles_,
// setLayout, and onRefreshTimer are all private. Tests therefore:
//   - count Tile children via the public QGraphicsItem::childItems() +
//     qgraphicsitem_cast<camp::map_tiles::Tile*>, and
//   - drive a refresh via QMetaObject::invokeMethod(layer, "onRefreshTimer",
//     Qt::DirectConnection) — onRefreshTimer is a private slot, reachable by name
//     through Qt's meta-object system without befriending the test.
// To populate tiles deterministically (without a network or WMTS round-trip) we
// build a tiny hand-made TileLayout and seed it via the public constructor's
// tile_layout argument; onRefreshTimer re-seeds it by re-applying that layout.

#include <gtest/gtest.h>

#include <QApplication>
#include <QTimer>

#include "map/map.h"
#include "map/layer_list.h"
#include "map_tiles/map_tiles.h"
#include "map_tiles/tile.h"
#include "map_tiles/tile_layout.h"

using camp::map::Map;
using camp::map_tiles::MapTiles;
using camp::map_tiles::Tile;
using camp::map_tiles::TileLayout;

namespace
{

// A minimal 1-zoom-level, cols x rows tile layout. zoom_levels.front() is what
// setLayout() expands into top-level Tile children, so the count of Tile children
// after a (re)layout is exactly cols*rows.
TileLayout makeLayout(int cols, int rows)
{
  TileLayout layout;
  TileLayout::ZoomLevel level;
  level.id = "0";
  level.scale = 1.0;
  level.top_left_corner = QPointF(0, 0);
  level.tile_width = 256;
  level.tile_height = 256;
  level.matrix_width = cols;
  level.matrix_height = rows;
  layout.zoom_levels.push_back(level);
  // A static URL part keeps getUrl() well-formed if load() is reached; the
  // network fetch itself is irrelevant here (Tiles are created before load()).
  layout.url_static_parts = {"file:///nonexistent/"};
  layout.url_variable_keys = {};
  return layout;
}

// Count the Tile children currently parented to a MapTiles layer, using only the
// public QGraphicsItem API (tiles_ is private).
int tileChildCount(const MapTiles* layer)
{
  int n = 0;
  for(QGraphicsItem* child : layer->childItems())
    if(qgraphicsitem_cast<Tile*>(child) != nullptr)
      ++n;
  return n;
}

// Drive one refresh deterministically via the private onRefreshTimer slot.
bool invokeRefresh(MapTiles* layer)
{
  return QMetaObject::invokeMethod(layer, "onRefreshTimer", Qt::DirectConnection);
}

} // namespace

// setRefreshInterval(msec>0) starts a repeating timer at exactly that interval.
TEST(MapTilesRefresh, PositiveIntervalStartsRepeatingTimer)
{
  Map map;
  auto* layer = new MapTiles(map.topLevelLayers(), "test_radar_active", makeLayout(1, 1));

  layer->setRefreshInterval(300000);  // 5 minutes, the radar cadence

  // Find the owned QTimer child and assert its configuration (no wall-clock wait).
  QTimer* timer = layer->findChild<QTimer*>();
  ASSERT_NE(timer, nullptr) << "setRefreshInterval should create an owned QTimer";
  EXPECT_TRUE(timer->isActive());
  EXPECT_FALSE(timer->isSingleShot());
  EXPECT_EQ(timer->interval(), 300000);
}

// setRefreshInterval(0) (or <0) disables refresh: any previously running timer is
// stopped, and a fresh layer that was never enabled has no active timer.
TEST(MapTilesRefresh, NonPositiveIntervalDisablesTimer)
{
  Map map;
  auto* layer = new MapTiles(map.topLevelLayers(), "test_radar_disable", makeLayout(1, 1));

  layer->setRefreshInterval(100);
  QTimer* timer = layer->findChild<QTimer*>();
  ASSERT_NE(timer, nullptr);
  ASSERT_TRUE(timer->isActive());

  layer->setRefreshInterval(0);
  EXPECT_FALSE(timer->isActive()) << "msec<=0 must stop the refresh timer";

  layer->setRefreshInterval(-5);
  EXPECT_FALSE(timer->isActive());
}

// A layer that never calls setRefreshInterval (every existing static layer) has
// no refresh timer at all — refresh is strictly opt-in.
TEST(MapTilesRefresh, RefreshIsOptInByDefault)
{
  Map map;
  auto* layer = new MapTiles(map.topLevelLayers(), "test_static", makeLayout(1, 1));
  EXPECT_EQ(layer->findChild<QTimer*>(), nullptr);
}

// One deterministic refresh: invoking onRefreshTimer rebuilds the tile set.
// Starting from a 2x2 layout (4 tiles), a refresh resets to the same layout, so
// the count returns to 4 — proving the layer is repopulated, not emptied.
TEST(MapTilesRefresh, RefreshRepopulatesTiles)
{
  Map map;
  auto* layer = new MapTiles(map.topLevelLayers(), "test_radar_refresh", makeLayout(2, 2));

  ASSERT_EQ(tileChildCount(layer), 4) << "ctor layout should seed 2x2 = 4 tiles";

  ASSERT_TRUE(invokeRefresh(layer)) << "onRefreshTimer slot should be invokable by name";

  EXPECT_EQ(tileChildCount(layer), 4)
      << "refresh must rebuild the zoom-0 tile set, not leave it empty";
}

// The #98-risk regression: memory must stay bounded across many refresh cycles.
// Each onRefreshTimer() must DELETE the prior Tile children before rebuilding, so
// the Tile-child count after N cycles equals the per-layout count, never N*count.
TEST(MapTilesRefresh, MemoryStaysBoundedAcrossManyRefreshes)
{
  Map map;
  auto* layer = new MapTiles(map.topLevelLayers(), "test_radar_bounded", makeLayout(2, 2));

  const int expected = 4;  // 2x2
  ASSERT_EQ(tileChildCount(layer), expected);

  for(int cycle = 0; cycle < 10; ++cycle)
  {
    ASSERT_TRUE(invokeRefresh(layer));
    EXPECT_EQ(tileChildCount(layer), expected)
        << "Tile children accumulated on refresh cycle " << cycle
        << " — old tiles are not being deleted (the #98 leak)";
  }
}

int main(int argc, char** argv)
{
  // MapTiles is a QGraphicsObject; Map builds a QGraphicsScene and MapItem ctors
  // touch qApp, so a QApplication is required. Force offscreen for headless CI,
  // mirroring test/test_map_model.cpp.
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
