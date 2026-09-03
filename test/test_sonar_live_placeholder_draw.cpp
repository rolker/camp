// [field 2026-08-27] Headless regression test: coarse overview data may be drawn ONLY
// as a placeholder for a fine tile that is known to exist and has not loaded, clipped
// to that fine tile's own footprint (ADR-0010 D5).
//
// The draw list used to be "every resident overview, coarse->fine, then the fine tiles
// on top", licensed by the camp#160 premise that "where a fine tile is present it fully
// covers its parent". That premise is false — a child covers a QUARTER of its parent —
// so wherever fine coverage is sparse (most of a survey in progress) the coarse
// ancestors painted through, over the chart. Measured on pandy during the BizzyBoat
// deployment: every cached tile is 960x960 cells at every level, so a level-0 cell is
// ~667 x 926 m of the MEAN of everything folded beneath it. The operator reported a
// solid ~650 m block over a good portion of the survey area, and the apex is exactly
// what kApexProtectLevel keeps resident forever.
//
// What is pinned here:
//   * NothingCoarseWhileTheFineTilesAreResident  the reported defect: with the fine
//                                data loaded, no overview contributes at all;
//   * PlaceholderClippedToItsOwnFootprint        a catalogued, non-resident fine tile
//                                IS covered by coarse data — sampled through a strict
//                                sub-rect of the overview, over the fine tile's extent
//                                and no further. Drawing the full coarse extent is
//                                precisely the bug, so the clip bound is asserted;
//   * ZoomedOutCoverageSurvivesEviction          the ADR-0010 D1 guarantee: with every
//                                fine tile evicted, a whole-survey view still shows
//                                each surveyed footprint (and here with no catalog at
//                                all, so evicted_fine_indices_ carries it alone);
//   * UnsurveyedWaterDrawsNothing                water inside the apex's extent that no
//                                fine tile ever covered gets nothing, at either zoom —
//                                the apex's own extent is 8 degrees of mostly untouched
//                                ocean.
//
// Offscreen QApplication + Map harness as test_sonar_live_overview_prune / _eviction:
// GL-free (renderImage() is never called, so no texture is ever uploaded — planDraw()
// resolves the selection without touching GL) and ROS-free (null Node -> no
// subscription; handleCatalog is invoked by name exactly as the ROS callback's
// GUI-thread marshal does).

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include <QApplication>
#include <QCoreApplication>
#include <QDir>
#include <QGeoCoordinate>
#include <QFile>
#include <QMetaObject>
#include <QRectF>
#include <QSettings>
#include <QTemporaryDir>
#include <QUrl>

#include "marine_autonomy/gggs.h"
#include "marine_interfaces/msg/sonar_visualization_tile.hpp"
#include "marine_interfaces/msg/tile_catalog.hpp"
#include "marine_interfaces/msg/visualization_band.hpp"

#include "map/map.h"
#include "map/layer_list.h"
#include "map_view/web_mercator.h"
#include "ros/live_coverage/sonar_live_cache_layer.h"
#include "ros/live_coverage/sonar_live_tile.h"

namespace mi = marine_interfaces::msg;
using camp::map::Map;
using camp::ros::live_coverage::SonarLiveCacheLayer;
using camp::ros::live_coverage::SonarLiveTile;
using camp::ros::live_coverage::tileIndexFromGridIndex;
using DrawItem = SonarLiveCacheLayer::DrawItem;

namespace
{

constexpr int kLevel = 10;
constexpr int kEdge = 8;
constexpr std::int16_t kDepthBase = 300;   // 3.00 m

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

QString stemOf(const gggs::GridIndex& index)
{
  return QString("%1_%2_%3")
    .arg(static_cast<int>(index.level()))
    .arg(index.row())
    .arg(index.column());
}

// Every strict ancestor of @p index, finest first, up to level 0.
std::vector<gggs::GridIndex> ancestorChain(const gggs::GridIndex& index)
{
  std::vector<gggs::GridIndex> chain;
  for(gggs::GridIndex a = gggs::parent(index); a.valid(); a = gggs::parent(a))
    chain.push_back(a);
  return chain;
}

struct CachePaths
{
  QString fine;
  QString overviews;

  QString finePath(const gggs::GridIndex& index) const
  {
    return QDir(fine).filePath(stemOf(index) + ".tif");
  }
};

// Seed the on-disk cache the way a live session would have left it: each fine tile at
// the cache-dir root plus the full overview chain it folds into, mirroring
// foldIntoParent() (fold the fine tile into its parent, then the WHOLE parent into the
// grandparent, up to level 0).
CachePaths seedCache(const QString& cache_root, const QString& ns,
                     const std::vector<gggs::GridIndex>& fine_tiles)
{
  CachePaths paths;
  paths.fine = QDir(cache_root).filePath(sanitizeNamespace(ns));
  paths.overviews = QDir(paths.fine).filePath("overviews");
  EXPECT_TRUE(QDir().mkpath(paths.fine));
  EXPECT_TRUE(QDir().mkpath(paths.overviews));

  std::map<gggs::GridIndex, SonarLiveTile> overviews;
  for(const gggs::GridIndex& grid : fine_tiles)
  {
    SonarLiveTile tile(grid, kEdge, kEdge);
    tile.applyPatch(makeUniformDepth(grid, kDepthBase));
    EXPECT_TRUE(tile.writeToGeoTiff(paths.finePath(grid).toStdString()));

    const SonarLiveTile* child = &tile;
    for(const gggs::GridIndex& ancestor : ancestorChain(grid))
    {
      auto it = overviews.find(ancestor);
      if(it == overviews.end())
        it = overviews.emplace(ancestor, SonarLiveTile(ancestor, kEdge, kEdge)).first;
      it->second.foldChild(*child);
      child = &it->second;
    }
  }
  for(const auto& [index, tile] : overviews)
    EXPECT_TRUE(tile.writeToGeoTiff(
      QDir(paths.overviews).filePath(stemOf(index) + ".tif").toStdString()));
  return paths;
}

mi::TileCatalog catalogOf(const std::vector<gggs::GridIndex>& indices, std::int32_t stamp_sec)
{
  mi::TileCatalog catalog;
  catalog.header.stamp.sec = stamp_sec;
  for(const gggs::GridIndex& index : indices)
  {
    mi::TileCatalogEntry entry;
    entry.index = tileIndexFromGridIndex(index);
    entry.version.sec = 150;
    catalog.entries.push_back(entry);
  }
  return catalog;
}

bool deliverCatalog(SonarLiveCacheLayer* layer, const mi::TileCatalog& catalog)
{
  return QMetaObject::invokeMethod(layer, "handleCatalog", Qt::DirectConnection,
                                   Q_ARG(marine_interfaces::msg::TileCatalog, catalog));
}

// The Web-Mercator scene rect of a GGGS index — the same extent math the layer's clip
// test uses, so a test clip means exactly what the draw path means by it.
QRectF sceneRectOf(const gggs::GridIndex& index)
{
  const QPointF lo = web_mercator::geoToMap(
    QGeoCoordinate(index.southLatitude(), index.westLongitude()));
  const QPointF hi = web_mercator::geoToMap(
    QGeoCoordinate(index.northLatitude(), index.eastLongitude()));
  return QRectF(lo, hi).normalized();
}

// Two well-separated fine tiles at the survey's level (~0.15 deg apart in latitude, so
// they share only the coarse ancestors) and the cache seeded from them.
struct Fixture
{
  gggs::GridIndex a;
  gggs::GridIndex b;
  CachePaths paths;
};

Fixture makeFixture(const QString& cache_root, const QString& ns)
{
  Fixture f;
  const gggs::Level level(kLevel);
  f.a = level.gridIndex(43.05, -70.55);
  f.b = level.gridIndex(43.20, -70.55);
  EXPECT_TRUE(f.a.valid());
  EXPECT_TRUE(f.b.valid());
  EXPECT_FALSE(f.a == f.b);
  f.paths = seedCache(cache_root, ns, {f.a, f.b});
  return f;
}

// Point the layer at the temp cache with @p budget_bytes of resident-footprint budget
// (0 disables eviction entirely, so the tests always pass a real number).
void useCache(const QString& cache_root, std::size_t budget_bytes)
{
  QSettings().clear();
  QSettings().setValue("LiveTileCache/cache_dir", cache_root);
  QSettings().setValue("LiveTileCache/max_vram_bytes", qulonglong(budget_bytes));
}

constexpr std::size_t kTileBytes = std::size_t(kEdge) * kEdge * sizeof(float);

std::vector<DrawItem> placeholdersOf(const std::vector<DrawItem>& plan)
{
  std::vector<DrawItem> out;
  for(const DrawItem& item : plan)
    if(item.placeholder)
      out.push_back(item);
  return out;
}

const DrawItem* footprintIn(const std::vector<DrawItem>& plan, const gggs::GridIndex& index)
{
  for(const DrawItem& item : plan)
    if(item.footprint == index)
      return &item;
  return nullptr;
}

}  // namespace

// The reported defect. With the fine tiles resident there is nothing for a coarse
// placeholder to stand in for, so no overview may contribute — not the level-0 apex
// (a ~650 m block of the mean of everything beneath it), not any other level. Before
// this change EVERY resident overview level was appended to the draw list at EVERY
// zoom, and the apex is resident by design (kApexProtectLevel).
TEST(SonarLivePlaceholderDraw, NothingCoarseWhileTheFineTilesAreResident)
{
  QTemporaryDir cache;
  ASSERT_TRUE(cache.isValid());
  useCache(cache.path(), 1000 * kTileBytes);   // generous: nothing is evicted

  const QString ns = "/placeholder_resident_test";
  const Fixture f = makeFixture(cache.path(), ns);

  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  auto* layer = new SonarLiveCacheLayer(layers, nullptr, ns);
  layer->enableLiveCoverage();

  ASSERT_EQ(layer->residentTileCount(), std::size_t(2));
  // Not vacuous: the pyramid IS resident, apex included, and would have been drawn.
  ASSERT_GT(layer->overviewTileCount(), std::size_t(0));
  gggs::GridIndex apex = f.a;
  while(gggs::parent(apex).valid())
    apex = gggs::parent(apex);
  ASSERT_NE(layer->overviewTileForTest(apex), nullptr)
    << "the apex is resident by design (kApexProtectLevel) — it is what used to draw";
  ASSERT_TRUE(deliverCatalog(layer, catalogOf({f.a, f.b}, 200)));
  ASSERT_EQ(layer->residentTileCount(), std::size_t(2)) << "the catalog kept both";

  // Survey zoom: the viewport is one fine tile.
  const std::vector<DrawItem> plan = layer->planDraw(sceneRectOf(f.a));
  EXPECT_TRUE(placeholdersOf(plan).empty())
    << "coarse data drawn while the fine tile it would stand in for is loaded";
  ASSERT_EQ(plan.size(), std::size_t(1));
  EXPECT_TRUE(plan.front().footprint == f.a);
  EXPECT_TRUE(plan.front().source == f.a);
  // A resident tile samples its whole texture — the RasterFieldItem default, which is
  // what keeps every other caller of the shared renderer unchanged.
  EXPECT_FLOAT_EQ(plan.front().u0, 0.0f);
  EXPECT_FLOAT_EQ(plan.front().v0, 0.0f);
  EXPECT_FLOAT_EQ(plan.front().u1, 1.0f);
  EXPECT_FLOAT_EQ(plan.front().v1, 1.0f);
}

// A catalogued fine tile that is NOT resident is covered by coarse data — sampled out
// of its finest resident ancestor through a strict sub-rect, and painted over the fine
// tile's own footprint. Painting the ancestor's whole extent instead is the bug, so
// both halves are asserted: the extent painted, and the texture window.
TEST(SonarLivePlaceholderDraw, PlaceholderClippedToItsOwnFootprint)
{
  QTemporaryDir cache;
  ASSERT_TRUE(cache.isValid());
  useCache(cache.path(), 1000 * kTileBytes);

  const QString ns = "/placeholder_clip_test";
  const Fixture f = makeFixture(cache.path(), ns);
  // Tile b's fine copy is gone (the boat has it, we don't yet); its folded coverage
  // survives in the pyramid, which is exactly the placeholder case.
  ASSERT_TRUE(QFile::remove(f.paths.finePath(f.b)));

  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  auto* layer = new SonarLiveCacheLayer(layers, nullptr, ns);
  layer->enableLiveCoverage();
  ASSERT_EQ(layer->residentTileCount(), std::size_t(1));

  // The boat holds both, so b is known to exist.
  ASSERT_TRUE(deliverCatalog(layer, catalogOf({f.a, f.b}, 200)));

  const std::vector<DrawItem> plan = layer->planDraw(sceneRectOf(f.b));
  const std::vector<DrawItem> placeholders = placeholdersOf(plan);
  ASSERT_EQ(placeholders.size(), std::size_t(1))
    << "a known-but-unloaded tile must be covered by coarse data";
  const DrawItem& item = placeholders.front();

  EXPECT_TRUE(item.footprint == f.b) << "painted over the missing tile's extent";
  const gggs::GridIndex expected_source = gggs::parent(f.b);
  EXPECT_TRUE(item.source == expected_source)
    << "the FINEST resident ancestor supplies the pixels";
  ASSERT_NE(layer->overviewTileForTest(expected_source), nullptr);

  // The clip bound. One level up is a 2x2 fold, so the window is exactly a quarter of
  // the ancestor — a fine index NEVER covers the whole of a coarser tile.
  EXPECT_LT(item.u1 - item.u0, 1.0f);
  EXPECT_LT(item.v1 - item.v0, 1.0f);
  EXPECT_FLOAT_EQ(item.u1 - item.u0, 0.5f);
  EXPECT_FLOAT_EQ(item.v1 - item.v0, 0.5f);

  // ...and it is the RIGHT quarter. Derived here from the geographic extents, which is
  // an independent route to the same answer as the index arithmetic the layer uses
  // (u west->east, v north->south, texture row 0 = north).
  const double lon_span = expected_source.eastLongitude() - expected_source.westLongitude();
  const double lat_span = expected_source.northLatitude() - expected_source.southLatitude();
  ASSERT_GT(lon_span, 0.0);
  ASSERT_GT(lat_span, 0.0);
  EXPECT_NEAR(item.u0, (f.b.westLongitude() - expected_source.westLongitude()) / lon_span,
              1e-5);
  EXPECT_NEAR(item.u1, (f.b.eastLongitude() - expected_source.westLongitude()) / lon_span,
              1e-5);
  EXPECT_NEAR(item.v0, (expected_source.northLatitude() - f.b.northLatitude()) / lat_span,
              1e-5);
  EXPECT_NEAR(item.v1, (expected_source.northLatitude() - f.b.southLatitude()) / lat_span,
              1e-5);

  // The resident tile is untouched by any of this: it is outside this viewport, and it
  // never gets a placeholder of its own.
  EXPECT_EQ(footprintIn(plan, f.a), nullptr);
}

// ADR-0010 D1, the regression guard: evict every fine tile (the shed-load path) and a
// whole-survey view must still show the surveyed area. No catalog is delivered here on
// purpose — a warm start with the link down has only evicted_fine_indices_, and the
// coverage must not depend on a catalog arriving.
TEST(SonarLivePlaceholderDraw, ZoomedOutCoverageSurvivesEviction)
{
  QTemporaryDir cache;
  ASSERT_TRUE(cache.isValid());
  useCache(cache.path(), kTileBytes / 2);   // below one tile: everything sheds

  const QString ns = "/placeholder_zoomout_test";
  const Fixture f = makeFixture(cache.path(), ns);

  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  auto* layer = new SonarLiveCacheLayer(layers, nullptr, ns);
  layer->enableLiveCoverage();

  ASSERT_EQ(layer->residentTileCount(), std::size_t(0)) << "warm-load must have shed";
  ASSERT_EQ(layer->evictedFineCount(), std::size_t(2));

  const QRectF survey = sceneRectOf(f.a).united(sceneRectOf(f.b));
  const std::vector<DrawItem> plan = layer->planDraw(survey);
  ASSERT_EQ(plan.size(), std::size_t(2)) << "a zoomed-out view lost its coverage";
  for(const gggs::GridIndex& index : {f.a, f.b})
  {
    const DrawItem* item = footprintIn(plan, index);
    ASSERT_NE(item, nullptr) << "no coverage at " << index;
    EXPECT_TRUE(item->placeholder);
    // Still clipped: the apex survives eviction (kApexProtectLevel) and is many levels
    // up, so the window is a small fraction of it — never the whole coarse extent.
    EXPECT_LT(item->u1 - item->u0, 1.0f);
    EXPECT_LT(item->v1 - item->v0, 1.0f);
    EXPECT_GT(item->u1, item->u0);
    EXPECT_GT(item->v1, item->v0);
    EXPECT_LT(static_cast<int>(item->source.level()), kLevel);
  }
}

// Water inside the apex's own extent that no fine tile ever covered draws nothing, at
// any zoom. A level-0 grid spans 8 degrees — the overwhelming majority of it is ocean
// nobody surveyed, and painting a coarse cell's mean across it is what put a block over
// the operator's chart.
TEST(SonarLivePlaceholderDraw, UnsurveyedWaterDrawsNothing)
{
  QTemporaryDir cache;
  ASSERT_TRUE(cache.isValid());
  useCache(cache.path(), kTileBytes / 2);   // everything evicted: the apex is all there is

  const QString ns = "/placeholder_unsurveyed_test";
  const Fixture f = makeFixture(cache.path(), ns);

  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  auto* layer = new SonarLiveCacheLayer(layers, nullptr, ns);
  layer->enableLiveCoverage();
  ASSERT_TRUE(deliverCatalog(layer, catalogOf({f.a, f.b}, 200)));

  // Unsurveyed water ~1.3 deg north of the survey, inside the SAME level-0 grid — so
  // the apex tile really does span it, and the old draw path really did paint it.
  const gggs::GridIndex empty_fine = gggs::Level(kLevel).gridIndex(44.5, -70.55);
  ASSERT_TRUE(empty_fine.valid());
  gggs::GridIndex apex = f.a;
  while(gggs::parent(apex).valid())
    apex = gggs::parent(apex);
  ASSERT_EQ(static_cast<int>(apex.level()), 0);
  ASSERT_NE(layer->overviewTileForTest(apex), nullptr) << "the apex must be resident";
  gggs::GridIndex empty_apex = empty_fine;
  while(gggs::parent(empty_apex).valid())
    empty_apex = gggs::parent(empty_apex);
  ASSERT_TRUE(empty_apex == apex) << "fixture: the empty area must be under the apex";

  EXPECT_TRUE(layer->planDraw(sceneRectOf(empty_fine)).empty())
    << "coarse data painted over water no fine tile ever covered";

  // Zoomed out to the whole apex, the surveyed footprints appear and nothing else does:
  // no drawn footprint reaches the unsurveyed area.
  const std::vector<DrawItem> wide = layer->planDraw(sceneRectOf(apex));
  EXPECT_FALSE(wide.empty());
  const QRectF empty_rect = sceneRectOf(empty_fine);
  for(const DrawItem& item : wide)
    EXPECT_FALSE(sceneRectOf(item.footprint).intersects(empty_rect))
      << "drew " << item.footprint << " over unsurveyed water";
}

int main(int argc, char** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  QCoreApplication::setOrganizationName("camp_test");
  QCoreApplication::setApplicationName("test_sonar_live_placeholder_draw");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
