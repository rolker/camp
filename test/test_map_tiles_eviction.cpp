// [#98] Coverage for MapTiles' bounded-memory LRU eviction — the fix for the
// CAMP-silently-crashes-on-map-zoom/pan OOM.
//
// Root cause: MapTiles::paint() mints a new Tile* for every newly-visible tile
// address and only hides (setVisible(false)) off-screen ones. The sole deletion
// path was setLayout(), reached only by the radar overlay's ~5-min refresh; the
// OSM/WMTS basemap never refreshes, so tiles_ grew for every tile area visited
// during a pan/zoom session until the process was OOM-killed. The fix caps tiles_
// at max(256, 4 * visible) and evicts the least-recently-visible off-screen tiles
// (deferred onto the event loop, since deleting a scene child inside paint() is a
// use-after-free risk).
//
// Why this test reproduces the field mechanism where test_map_tiles_refresh.cpp
// could not: setLayout() pre-seeds the ENTIRE zoom_levels.front() grid at
// construction, so paint() mints nothing if the layout has a single zoom level.
// We therefore build a layout with a tiny front level and a large DEEPER level;
// paint() renders the deeper level and mints its tiles on demand as we pan, which
// is exactly the basemap accumulation path. We drive paint() directly with a
// hand-built QPainter world transform (deterministic, no QPA rendering) and pan a
// small viewport across every position of the deeper grid. Because eviction is
// deferred, we processEvents() after each paint so the queued evictIfNeeded()
// runs, then assert tiles_ never exceeds the cap.
//
// Non-vacuity: the deeper grid is 20x20 = 400 tiles plus the 1 seeded front tile
// = up to 401 minted. The cap with a small viewport is the 256 floor. With the
// eviction logic removed, tiles_ reaches 401 and the final assertion (<= 256)
// fails — confirmed empirically by building once with evictIfNeeded()'s body
// stubbed out. So this is a genuine regression gate, not a tautology.
//
// Access mechanism (private tiles_): count Tile children via the public
// QGraphicsItem::childItems() + qgraphicsitem_cast, mirroring
// test_map_tiles_refresh.cpp.

#include <gtest/gtest.h>

#include <QApplication>
#include <QImage>
#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QTransform>

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

// The eviction floor (kTileEvictionMinCap in map_tiles.cpp). Kept in sync by hand
// — the constant is file-private. A small panning viewport keeps the visible
// count tiny, so 4 * visible never beats this floor and the cap stays here.
constexpr int kEvictionFloor = 256;

// Geometry constants shared by the layout and the pan transform. See the
// transform derivation in panTo() below; chosen so all index arithmetic in
// MapTiles::paint() lands on exact integers (no floating-point boundary flutter).
constexpr double kFrontScale = 1.0;      // front level: coarse, never rendered here
constexpr double kDeepScale = 0.0625;    // deep level: 1/16, exactly representable
constexpr int kTilePixels = 256;         // tile_width == tile_height
constexpr double kDeepTileWorld = kTilePixels * kDeepScale;  // = 16.0, exact
constexpr int kViewScale = 8;            // world transform uniform scale (m11/|m22|)
constexpr int kDeepGrid = 20;            // deep level is 20x20 = 400 tiles

// A two-zoom-level layout: a 1x1 front level (all that setLayout() seeds) and a
// gridxgrid deeper level that paint() mints on demand. Field-realistic shape: the
// basemap that never refreshes is exactly such a multi-level OSM/WMTS pyramid.
TileLayout makeMultiZoomLayout(int grid)
{
  TileLayout layout;

  TileLayout::ZoomLevel front;
  front.id = "0";
  front.scale = kFrontScale;
  front.top_left_corner = QPointF(0, 0);
  front.tile_width = kTilePixels;
  front.tile_height = kTilePixels;
  front.matrix_width = 1;
  front.matrix_height = 1;
  layout.zoom_levels.push_back(front);

  TileLayout::ZoomLevel deep;
  deep.id = "1";
  deep.scale = kDeepScale;
  deep.top_left_corner = QPointF(0, 0);
  deep.tile_width = kTilePixels;
  deep.tile_height = kTilePixels;
  deep.matrix_width = grid;
  deep.matrix_height = grid;
  layout.zoom_levels.push_back(deep);

  layout.url_static_parts = {"file:///nonexistent/"};
  layout.url_variable_keys = {};
  return layout;
}

int tileChildCount(const MapTiles* layer)
{
  int n = 0;
  for(QGraphicsItem* child : layer->childItems())
    if(qgraphicsitem_cast<Tile*>(child) != nullptr)
      ++n;
  return n;
}

// Drive one paint() of the deep level with the viewport positioned over deep-tile
// (col, row). The world transform is a uniform scale kViewScale with a y-flip
// (m22 negative, matching web-mercator north-up), translated so the painter
// window's world rect is exactly the one deep tile (col, row):
//
//   m11 = S, m22 = -S, m31 = -S*col*T, m32 = -S*row*T   (S = kViewScale, T = 16)
//
// Then in MapTiles::paint(): lod = S/2 = 4, so the front level (scale*lod = 4 >=
// 2) is skipped and the deep level (scale*lod = 0.25 < 2) is rendered; the window
// maps to world [col*T,(col+1)*T] x (north-up) row [row..row+1], so paint() mints
// the up-to-2x2 deep tiles around (col, row). A WxH = (T*S)x(T*S) = 128x128 image
// gives window().width()/height() == T*S, the one-tile world span.
void panTo(MapTiles* layer, int col, int row)
{
  const int wh = static_cast<int>(kDeepTileWorld) * kViewScale;  // 16 * 8 = 128
  QImage image(wh, wh, QImage::Format_ARGB32);
  QPainter painter(&image);
  const double tx = -kViewScale * col * kDeepTileWorld;
  const double ty = -kViewScale * row * kDeepTileWorld;
  painter.setWorldTransform(QTransform(kViewScale, 0, 0, -kViewScale, tx, ty));
  QStyleOptionGraphicsItem option;
  layer->paint(&painter, &option, nullptr);
}

} // namespace

// Sanity: the deeper level really is minted by paint() (not pre-seeded). After a
// single paint at the origin, more than the 1 seeded front tile must exist.
TEST(MapTilesEviction, DeepLevelTilesAreMintedByPaint)
{
  Map map;
  auto* layer =
      new MapTiles(map.topLevelLayers(), "test_evict_mint", makeMultiZoomLayout(kDeepGrid));

  ASSERT_EQ(tileChildCount(layer), 1) << "ctor should seed only the 1x1 front level";

  panTo(layer, 0, 0);
  QApplication::processEvents();

  EXPECT_GT(tileChildCount(layer), 1)
      << "paint() must mint deep-level tiles on demand — otherwise the pan below "
         "would never reproduce the #98 accumulation path";
}

// The load-bearing regression test: pan a small viewport across the entire 20x20
// deeper grid (>> the 256 cap of distinct tiles). With deferred eviction running
// after each paint, tiles_ must stay at or below the cap. Without the fix it would
// climb to ~401 and this assertion would fail.
TEST(MapTilesEviction, PanAcrossLargeGridStaysBounded)
{
  Map map;
  auto* layer =
      new MapTiles(map.topLevelLayers(), "test_evict_bounded", makeMultiZoomLayout(kDeepGrid));

  // Step the viewport so its 2x2 footprint sweeps cols/rows 0..kDeepGrid-1,
  // visiting every one of the 400 deep tiles. processEvents() after each paint
  // lets the queued evictIfNeeded() run (eviction is deferred off paint()).
  for(int row = 0; row < kDeepGrid - 1; ++row)
    for(int col = 0; col < kDeepGrid - 1; ++col)
    {
      panTo(layer, col, row);
      QApplication::processEvents();
      EXPECT_LE(tileChildCount(layer), kEvictionFloor)
          << "tiles_ exceeded the eviction cap mid-pan at (" << col << "," << row
          << ") — within-cycle accumulation is unbounded (the #98 OOM)";
    }

  // Flush any final queued eviction and assert the steady-state bound. A small
  // viewport keeps visible small, so the cap is the 256 floor.
  QApplication::processEvents();
  EXPECT_LE(tileChildCount(layer), kEvictionFloor)
      << "after panning the whole grid, tiles_ must be bounded by the eviction cap";
}

int main(int argc, char** argv)
{
  // MapTiles is a QGraphicsObject; Map builds a QGraphicsScene and MapItem ctors
  // touch qApp, so a QApplication is required. Force offscreen for headless CI,
  // mirroring test/test_map_tiles_refresh.cpp.
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
