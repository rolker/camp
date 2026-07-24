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
// [#111] This file also covers the per-refresh URL cache-buster that defeats
// CDN/proxy caching of the static radar tile URL: withCacheBust() pure-logic, the
// strict per-cycle token advance on refresh, and that static layers never bust.
// The invariant is URL/token DISTINCTNESS per cycle — asserted without any live
// network (the IEM endpoint's tolerance of an unknown ?t= param was confirmed
// out-of-band, HTTP 200 + identical bytes).
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
#include "map_tiles/cached_tile_loader.h"

using camp::map::Map;
using camp::map_tiles::MapTiles;
using camp::map_tiles::Tile;
using camp::map_tiles::TileLayout;
using camp::map_tiles::CachedTileLoader;

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

// Refresh-boundary reset regression: each onRefreshTimer() must DELETE the prior
// Tile children before rebuilding, so the Tile-child count after N cycles equals
// the per-layout count, never N*count.
//
// SCOPE (honest): this guards ONLY the refresh-boundary reset — that a refresh
// does not leave the previous cycle's tiles behind. The within-cycle growth that
// the #98 OOM is actually about — between refreshes, paint() minting new tiles as
// the viewport pans/zooms — is now bounded by the [#98] LRU eviction and is
// covered by test_map_tiles_eviction.cpp (which pans across a deeper zoom level
// and asserts tiles_ stays under the cap). This test remains the refresh-boundary
// regression gate; the two together close the #98 tile-lifecycle risk.
TEST(MapTilesRefresh, RefreshBoundaryResetsTileSet)
{
  Map map;
  auto* layer = new MapTiles(map.topLevelLayers(), "test_radar_bounded", makeLayout(2, 2));

  const int expected = 4;  // 2x2
  ASSERT_EQ(tileChildCount(layer), expected);

  for(int cycle = 0; cycle < 10; ++cycle)
  {
    ASSERT_TRUE(invokeRefresh(layer));
    EXPECT_EQ(tileChildCount(layer), expected)
        << "Tile children carried over across refresh cycle " << cycle
        << " — the refresh boundary is not resetting the tile set (old tiles "
           "not deleted by setLayout). NOTE: this checks the refresh-boundary "
           "reset only, not within-cycle pan/zoom accumulation.";
  }
}

// [#111] withCacheBust is pure/static: a distinct token must yield a distinct URL,
// joined with '?' when the URL has no query yet and '&' when it already does.
TEST(MapTilesRefresh, CacheBustAppendsDistinctQueryToken)
{
  // No existing query → '?t='. XYZ tile URLs (e.g. IEM radar) take this path.
  EXPECT_EQ(CachedTileLoader::withCacheBust("https://h/9/1/2.png", 42),
            "https://h/9/1/2.png?t=42");
  // Existing query → '&t=' so the URL stays well-formed.
  EXPECT_EQ(CachedTileLoader::withCacheBust("https://h/wms?layer=x", 42),
            "https://h/wms?layer=x&t=42");
  // The whole point: different tokens produce different URLs (CDN-distinct).
  EXPECT_NE(CachedTileLoader::withCacheBust("https://h/9/1/2.png", 1),
            CachedTileLoader::withCacheBust("https://h/9/1/2.png", 2));
}

// Enabling refresh (setRefreshInterval > 0) turns cache-busting on with a non-zero
// seed, and every refresh cycle advances the token STRICTLY — so each cycle's
// request URL is one a CDN/proxy cannot satisfy from its edge cache (#111).
TEST(MapTilesRefresh, RefreshAdvancesCacheBustTokenStrictly)
{
  Map map;
  auto* layer = new MapTiles(map.topLevelLayers(), "test_radar_bust", makeLayout(1, 1));

  layer->setRefreshInterval(300000);  // enables busting (radar cadence)
  CachedTileLoader* loader = layer->findChild<CachedTileLoader*>();
  ASSERT_NE(loader, nullptr) << "MapTiles owns a CachedTileLoader child";

  const quint64 t0 = loader->cacheBustToken();
  EXPECT_NE(t0, quint64(0))
      << "enableCacheBusting (via setRefreshInterval) should seed a non-zero token";

  ASSERT_TRUE(invokeRefresh(layer));
  const quint64 t1 = loader->cacheBustToken();
  ASSERT_TRUE(invokeRefresh(layer));
  const quint64 t2 = loader->cacheBustToken();

  EXPECT_GT(t1, t0) << "a refresh must advance the cache-bust token";
  EXPECT_GT(t2, t1)
      << "each refresh must yield a STRICTLY greater token (a CDN-distinct URL), "
         "even for back-to-back cycles within the same millisecond";
}

// Refresh (and therefore cache-busting) is strictly opt-in: a layer that never
// calls setRefreshInterval — every existing static OSM/WMTS layer — keeps token 0,
// so withCacheBust is never applied and its request URLs are unchanged.
TEST(MapTilesRefresh, StaticLayerNeverBustsCache)
{
  Map map;
  auto* layer = new MapTiles(map.topLevelLayers(), "test_static_bust", makeLayout(1, 1));

  CachedTileLoader* loader = layer->findChild<CachedTileLoader*>();
  ASSERT_NE(loader, nullptr);
  EXPECT_EQ(loader->cacheBustToken(), quint64(0))
      << "a layer that never enables refresh must not cache-bust its URLs";
}

// [#111] A refreshing (radar) layer becoming visible must run the full refresh,
// which advances the cache-bust token (the network URL becomes CDN-distinct).
TEST(MapTilesRefresh, BecomingVisibleAdvancesCacheBustToken)
{
  Map map;
  auto* layer = new MapTiles(map.topLevelLayers(), "test_radar_show", makeLayout(1, 1));

  layer->setRefreshInterval(300000);  // enable cache-busting (radar)
  layer->setVisible(false);
  CachedTileLoader* loader = layer->findChild<CachedTileLoader*>();
  ASSERT_NE(loader, nullptr);

  const quint64 hidden = loader->cacheBustToken();
  ASSERT_NE(hidden, quint64(0)) << "refreshing layer should have busting enabled";

  layer->setVisible(true);  // show transition → full refresh (bump + invalidate + rebuild)
  EXPECT_GT(loader->cacheBustToken(), hidden)
      << "becoming visible must advance the token so the first paint fetches fresh, "
         "never a previous session's cached frame";
}

// [#111] Token advance alone is NOT enough — the already-built tiles must be
// REBUILT on show, else they re-show the stale (e.g. yesterday's) frame until the
// next 5-min refresh. This guards the must-fix: itemChange must run the full
// refresh (setLayout rebuild), not just bump+invalidate. We count destruction of
// the pre-show Tile objects (robust to allocator pointer reuse): a rebuild deletes
// and re-creates them; a token-only path would leave them alive.
TEST(MapTilesRefresh, BecomingVisibleRebuildsTilesNotJustToken)
{
  Map map;
  auto* layer = new MapTiles(map.topLevelLayers(), "test_radar_show_rebuild", makeLayout(2, 2));

  layer->setRefreshInterval(300000);  // refreshing (radar) layer
  layer->setVisible(false);
  ASSERT_EQ(tileChildCount(layer), 4) << "ctor seeds 2x2 = 4 tiles";

  int destroyed = 0;
  for(QGraphicsItem* child : layer->childItems())
    if(Tile* t = qgraphicsitem_cast<Tile*>(child))
      QObject::connect(t, &QObject::destroyed, [&destroyed]{ ++destroyed; });

  layer->setVisible(true);  // show → full refresh must DELETE + rebuild the tiles

  EXPECT_EQ(destroyed, 4)
      << "becoming visible must REBUILD the tile set (re-fetch), not merely advance "
         "the token — otherwise the already-built stale tiles persist on screen";
  EXPECT_EQ(tileChildCount(layer), 4) << "rebuilt back to the 2x2 layout";
}

// A static (non-refreshing) layer toggling visibility never enters the
// invalidate-on-show branch: itemChange gates the whole bump/invalidate/rebuild on
// cacheBustToken() != 0, and a static layer's token stays 0 — so its disk cache and
// built tiles are preserved on show. token == 0 is the proxy for "branch not taken".
TEST(MapTilesRefresh, StaticLayerVisibilityDoesNotRefresh)
{
  Map map;
  auto* layer = new MapTiles(map.topLevelLayers(), "test_static_show", makeLayout(2, 2));

  int destroyed = 0;
  for(QGraphicsItem* child : layer->childItems())
    if(Tile* t = qgraphicsitem_cast<Tile*>(child))
      QObject::connect(t, &QObject::destroyed, [&destroyed]{ ++destroyed; });

  layer->setVisible(false);
  layer->setVisible(true);

  CachedTileLoader* loader = layer->findChild<CachedTileLoader*>();
  ASSERT_NE(loader, nullptr);
  EXPECT_EQ(loader->cacheBustToken(), quint64(0))
      << "a static layer must never enable cache-busting (token stays 0)";
  EXPECT_EQ(destroyed, 0)
      << "a static layer must NOT rebuild its tiles on show — the invalidate-on-show "
         "branch is gated on a non-zero token, which a static layer never has";
}

int main(int argc, char** argv)
{
  // MapTiles is a QGraphicsObject; Map builds a QGraphicsScene and MapItem ctors
  // touch qApp, so a QApplication is required. Force offscreen for headless CI,
  // mirroring test/test_map_model.cpp.
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  // [camp#117] Map's ctor now writes the BackgroundTileLayers seed into QSettings;
  // a test org/app name keeps that out of the developer's real camp settings.
  QCoreApplication::setOrganizationName("camp_test");
  QCoreApplication::setApplicationName("test_map_tiles_refresh");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
