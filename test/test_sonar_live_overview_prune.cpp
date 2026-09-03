// [field 2026-08-27] Headless regression test: anti-entropy prune-on-absence must
// propagate into the DERIVED overview pyramid (ADR-0010 D7).
//
// The catalog reconcile pruned only `tiles_` (memory + GPU + the fine `.tif`). The
// overview pyramid built by foldIntoParent() was exempt, so a retracted region kept its
// coarse coverage forever: the `overviews/` files survived every prune and warm-load
// brought them straight back on the next restart. Measured on pandy during the
// BizzyBoat deployment after a boat-side store reset — the fine cache pruned correctly
// (107M -> 44M, every survivor newer than the restart) while 41M of `overviews/`
// survived and the operator still saw the pre-reset coverage.
//
// Compounding it, the fold is purely accumulative: foldChild() only ever folds data IN
// and has no inverse, so an overview that lost SOME of its descendants cannot have the
// withdrawn contribution subtracted in place — it has to be rebuilt from what is left.
//
// What is pinned here:
//   * StaleOverviewsRemoved      an overview with no catalogued descendant leaves both
//                                memory and disk;
//   * SurvivingAncestorsKept     an overview that still has a catalogued descendant is
//                                NOT removed — the over-deletion guard. Every pruned
//                                tile's ancestor chain reaches level 0, so a naive
//                                invalidate-the-chain would destroy the whole pyramid
//                                on any ordinary retraction;
//   * SharedAncestorRebuilt      an overview that lost only SOME descendants is rebuilt
//                                from the survivors, so the withdrawn tile's values are
//                                really gone from it (partial withdrawal);
//   * RoutineCatalogLeavesPyramidAlone  a catalog that retracts nothing touches nothing;
//   * UnstampedCatalogDoesNotSweep      generation_time 0 disables prune-on-absence in
//                                the reconciler (ADR-0008 D4b), so it must disable the
//                                pyramid sweep too — otherwise an un-stamped catalog
//                                would delete the pyramid while every fine tile stayed.
//
// Offscreen QApplication + Map harness as test_sonar_live_eviction / _resume: GL-free
// (renderImage() is never called, so no texture is ever uploaded) and ROS-free (null
// Node -> no subscription; handleCatalog is invoked by name exactly as the ROS
// callback's GUI-thread marshal does).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <utility>
#include <vector>

#include <QApplication>
#include <QCoreApplication>
#include <QDir>
#include <QEvent>
#include <QFileInfo>
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
using camp::ros::live_coverage::SonarLiveBand;
using camp::ros::live_coverage::SonarLiveCacheLayer;
using camp::ros::live_coverage::SonarLiveTile;
using camp::ros::live_coverage::tileIndexFromGridIndex;

namespace
{

constexpr int kLevel = 10;
constexpr int kEdge = 8;
constexpr std::int16_t kKeepBase = 300;   // 3.00 m — the tile the catalog keeps
constexpr std::int16_t kDropBase = 500;   // 5.00 m — the tile the catalog withdraws

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

// The layer's cache layout for one source namespace.
struct CachePaths
{
  QString fine;
  QString overviews;

  QString finePath(const gggs::GridIndex& index) const
  {
    return QDir(fine).filePath(stemOf(index) + ".tif");
  }
  QString overviewPath(const gggs::GridIndex& index) const
  {
    return QDir(overviews).filePath(stemOf(index) + ".tif");
  }
};

// Seed the on-disk cache the way a live session would have left it: each fine tile at
// the cache-dir root, plus the full overview chain each one folds into, mirroring
// foldIntoParent() (fold the fine tile into its parent, then the WHOLE parent into the
// grandparent, up to level 0). Returns the layout so a test can assert on the files.
CachePaths seedCache(const QString& cache_root, const QString& ns,
                     const std::vector<std::pair<gggs::GridIndex, std::int16_t>>& fine_tiles)
{
  CachePaths paths;
  paths.fine = QDir(cache_root).filePath(sanitizeNamespace(ns));
  paths.overviews = QDir(paths.fine).filePath("overviews");
  EXPECT_TRUE(QDir().mkpath(paths.fine));
  EXPECT_TRUE(QDir().mkpath(paths.overviews));

  std::map<gggs::GridIndex, SonarLiveTile> overviews;
  for(const auto& [grid, base] : fine_tiles)
  {
    SonarLiveTile tile(grid, kEdge, kEdge);
    tile.applyPatch(makeUniformDepth(grid, base));
    EXPECT_TRUE(tile.writeToGeoTiff(paths.finePath(grid).toStdString()));

    // Walk the chain, folding the accumulated tile into each successive parent.
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
    EXPECT_TRUE(tile.writeToGeoTiff(paths.overviewPath(index).toStdString()));
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

// Cells of the tile's "depth" band that carry real data (NaN / NoData never count),
// and of those, how many hold something OTHER than @p value.
//
// The assertions are written as "every finite cell equals the kept tile's value"
// rather than "no cell equals the withdrawn tile's value" on purpose: the fold takes
// the MEAN of the child cells landing in a parent cell, so at coarse levels the two
// fine tiles collapse into the SAME parent cell and the withdrawn 5.00 shows up as a
// 4.00 mean, not as a 5.00. Any contribution from the withdrawn tile — direct or
// averaged — moves a cell off 3.00, so this catches both.
std::size_t finiteCellCount(const SonarLiveTile& tile)
{
  const SonarLiveBand* band = tile.band("depth");
  if(!band)
    return 0;
  std::size_t hits = 0;
  for(float v : band->data)
    if(std::isfinite(v) && !(band->has_nodata && v == band->nodata))
      ++hits;
  return hits;
}

std::size_t cellsUnequalTo(const SonarLiveTile& tile, float value)
{
  const SonarLiveBand* band = tile.band("depth");
  if(!band)
    return 0;
  std::size_t hits = 0;
  for(float v : band->data)
    if(std::isfinite(v) && !(band->has_nodata && v == band->nodata) &&
       std::fabs(v - value) >= 1e-3f)
      ++hits;
  return hits;
}

constexpr float kKeepValue = 0.01f * kKeepBase;

// The shared test fixture data: two well-separated fine tiles, the ancestors unique to
// each, and the ancestors they share.
struct Fixture
{
  gggs::GridIndex keep;    // stays in the catalog
  gggs::GridIndex drop;    // withdrawn by the catalog
  std::vector<gggs::GridIndex> keep_only;    // ancestors of `keep` alone
  std::vector<gggs::GridIndex> drop_only;    // ancestors of `drop` alone
  std::vector<gggs::GridIndex> shared;       // ancestors of both, coarsest-common up
  CachePaths paths;
};

Fixture makeFixture(const QString& cache_root, const QString& ns)
{
  Fixture f;
  const gggs::Level level(kLevel);
  // Same longitude, ~0.15 deg apart in latitude: identical grids down to level 5
  // (0.25 deg spans) and distinct from level 6 (0.125 deg) on down. That gives the
  // fixture a multi-level SHARED ancestor chain (levels 0-5, the rebuild targets) as
  // well as exclusive chains for each tile (levels 6-9), all asserted below rather
  // than assumed.
  f.keep = level.gridIndex(43.05, -70.55);
  f.drop = level.gridIndex(43.20, -70.55);
  EXPECT_TRUE(f.keep.valid());
  EXPECT_TRUE(f.drop.valid());
  EXPECT_FALSE(f.keep == f.drop);

  const std::vector<gggs::GridIndex> keep_chain = ancestorChain(f.keep);
  const std::vector<gggs::GridIndex> drop_chain = ancestorChain(f.drop);
  for(const gggs::GridIndex& a : keep_chain)
  {
    const bool in_drop =
      std::find(drop_chain.begin(), drop_chain.end(), a) != drop_chain.end();
    if(in_drop)
      f.shared.push_back(a);
    else
      f.keep_only.push_back(a);
  }
  for(const gggs::GridIndex& a : drop_chain)
    if(std::find(keep_chain.begin(), keep_chain.end(), a) == keep_chain.end())
      f.drop_only.push_back(a);

  // Fail loudly rather than passing vacuously if the chosen coordinates stop yielding
  // all three ancestor classes.
  EXPECT_FALSE(f.keep_only.empty());
  EXPECT_FALSE(f.drop_only.empty());
  EXPECT_FALSE(f.shared.empty());

  f.paths = seedCache(cache_root, ns, {{f.keep, kKeepBase}, {f.drop, kDropBase}});
  return f;
}

// Configure a generous budget so eviction never interferes, and point the layer at the
// temp cache.
void useCache(const QString& cache_root)
{
  QSettings().clear();
  QSettings().setValue("LiveTileCache/cache_dir", cache_root);
  const std::size_t per_tile = static_cast<std::size_t>(kEdge) * kEdge * sizeof(float);
  QSettings().setValue("LiveTileCache/max_vram_bytes", qulonglong(1000 * per_tile));
}

}  // namespace

// An overview tile whose every descendant vanished from the catalog is entirely stale:
// it must leave memory AND disk. Nothing else in the pyramid may be touched.
TEST(SonarLiveOverviewPrune, StaleOverviewsRemovedAndSurvivingAncestorsKept)
{
  QTemporaryDir cache;
  ASSERT_TRUE(cache.isValid());
  useCache(cache.path());

  const QString ns = "/overview_prune_test";
  const Fixture f = makeFixture(cache.path(), ns);

  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  auto* layer = new SonarLiveCacheLayer(layers, nullptr, ns);

  layer->enableLiveCoverage();
  ASSERT_EQ(layer->residentTileCount(), std::size_t(2));
  ASSERT_EQ(layer->overviewTileCount(),
            f.keep_only.size() + f.drop_only.size() + f.shared.size());

  // The boat now holds only `keep`. generation_time must be non-zero or the
  // reconciler's prune gate (ADR-0008 D4b) declines to prune at all.
  ASSERT_TRUE(deliverCatalog(layer, catalogOf({f.keep}, 200)));

  // The fine tile went, as before this change.
  EXPECT_EQ(layer->residentTileCount(), std::size_t(1));
  EXPECT_FALSE(QFileInfo::exists(f.paths.finePath(f.drop)));
  EXPECT_TRUE(QFileInfo::exists(f.paths.finePath(f.keep)));

  // ...and now so did every overview that existed only for it — this is what used to
  // survive, in memory and on disk, and come back on every restart via warm-load.
  for(const gggs::GridIndex& a : f.drop_only)
  {
    EXPECT_EQ(layer->overviewTileForTest(a), nullptr) << "resident: " << a;
    EXPECT_FALSE(QFileInfo::exists(f.paths.overviewPath(a))) << "on disk: " << a;
  }

  // The over-deletion guard. Every pruned tile's ancestor chain runs all the way to
  // level 0, so a chain-invalidating fix would have wiped the pyramid here. Ancestors
  // with a surviving catalogued descendant must all still be present.
  for(const gggs::GridIndex& a : f.keep_only)
  {
    EXPECT_NE(layer->overviewTileForTest(a), nullptr) << "resident: " << a;
    EXPECT_TRUE(QFileInfo::exists(f.paths.overviewPath(a))) << "on disk: " << a;
  }
  for(const gggs::GridIndex& a : f.shared)
  {
    EXPECT_NE(layer->overviewTileForTest(a), nullptr) << "resident: " << a;
    EXPECT_TRUE(QFileInfo::exists(f.paths.overviewPath(a))) << "on disk: " << a;
  }
}

// Partial withdrawal: an ancestor shared by a kept and a withdrawn tile survives, but
// it still holds the withdrawn tile's folded cells. foldChild() has no inverse, so it
// must be REBUILT from its surviving children rather than kept as-is.
TEST(SonarLiveOverviewPrune, SharedAncestorRebuiltWithoutTheWithdrawnDescendant)
{
  QTemporaryDir cache;
  ASSERT_TRUE(cache.isValid());
  useCache(cache.path());

  const QString ns = "/overview_rebuild_test";
  const Fixture f = makeFixture(cache.path(), ns);

  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  auto* layer = new SonarLiveCacheLayer(layers, nullptr, ns);
  layer->enableLiveCoverage();

  // Precondition: every shared ancestor carries the withdrawn tile's contribution —
  // some finite cell is not the kept tile's value.
  for(const gggs::GridIndex& a : f.shared)
  {
    const SonarLiveTile* tile = layer->overviewTileForTest(a);
    ASSERT_NE(tile, nullptr) << a;
    EXPECT_GT(finiteCellCount(*tile), std::size_t(0)) << a;
    EXPECT_GT(cellsUnequalTo(*tile, kKeepValue), std::size_t(0)) << a;
  }

  ASSERT_TRUE(deliverCatalog(layer, catalogOf({f.keep}, 200)));

  for(const gggs::GridIndex& a : f.shared)
  {
    const SonarLiveTile* tile = layer->overviewTileForTest(a);
    ASSERT_NE(tile, nullptr) << a;
    // ...and the surviving descendant's coverage is intact: the repair must not empty
    // the tile out, which would trade stale coverage for no coverage.
    EXPECT_GT(finiteCellCount(*tile), std::size_t(0))
      << "rebuild lost the surviving descendant at " << a;
    // The withdrawn tile's contribution is gone (directly or as a mean).
    EXPECT_EQ(cellsUnequalTo(*tile, kKeepValue), std::size_t(0))
      << "withdrawn contribution still folded into " << a;
  }

  // The repair is persisted, or warm-load would bring the stale content straight back
  // on the next restart — the mechanism that kept the pandy pyramid alive.
  layer->deleteLater();
  QCoreApplication::sendPostedEvents(nullptr, QEvent::DeferredDelete);
  const gggs::GridIndex apex = f.shared.back();
  const std::optional<SonarLiveTile> from_disk = SonarLiveTile::loadFromGeoTiff(
    f.paths.overviewPath(apex).toStdString(), gggs::Level(apex.level()));
  ASSERT_TRUE(from_disk.has_value());
  EXPECT_GT(finiteCellCount(*from_disk), std::size_t(0));
  EXPECT_EQ(cellsUnequalTo(*from_disk, kKeepValue), std::size_t(0));
}

// Routine churn: a catalog that retracts nothing must leave the pyramid completely
// alone — no removals, no rebuilds, no lost coverage.
TEST(SonarLiveOverviewPrune, RoutineCatalogLeavesPyramidAlone)
{
  QTemporaryDir cache;
  ASSERT_TRUE(cache.isValid());
  useCache(cache.path());

  const QString ns = "/overview_churn_test";
  const Fixture f = makeFixture(cache.path(), ns);

  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  auto* layer = new SonarLiveCacheLayer(layers, nullptr, ns);
  layer->enableLiveCoverage();
  const std::size_t before = layer->overviewTileCount();

  ASSERT_TRUE(deliverCatalog(layer, catalogOf({f.keep, f.drop}, 200)));

  EXPECT_EQ(layer->residentTileCount(), std::size_t(2));
  EXPECT_EQ(layer->overviewTileCount(), before);
  for(const gggs::GridIndex& a : f.shared)
  {
    const SonarLiveTile* tile = layer->overviewTileForTest(a);
    ASSERT_NE(tile, nullptr) << a;
    // Both contributions still folded in — nothing was rebuilt or thrown away.
    EXPECT_GT(finiteCellCount(*tile), std::size_t(0)) << a;
    EXPECT_GT(cellsUnequalTo(*tile, kKeepValue), std::size_t(0)) << a;
  }
  for(const gggs::GridIndex& a : f.drop_only)
    EXPECT_TRUE(QFileInfo::exists(f.paths.overviewPath(a))) << a;
}

// Prune-gate parity: a generation_time of 0 (un-stamped / sim-time-0 catalog) disables
// prune-on-absence in the reconciler, because no held version can be strictly older
// than 0. The pyramid sweep must honour the same gate — otherwise an un-stamped catalog
// would delete the whole pyramid while every fine tile stayed exactly where it was.
TEST(SonarLiveOverviewPrune, UnstampedCatalogDoesNotSweepThePyramid)
{
  QTemporaryDir cache;
  ASSERT_TRUE(cache.isValid());
  useCache(cache.path());

  const QString ns = "/overview_unstamped_test";
  const Fixture f = makeFixture(cache.path(), ns);

  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  auto* layer = new SonarLiveCacheLayer(layers, nullptr, ns);
  layer->enableLiveCoverage();
  const std::size_t before = layer->overviewTileCount();

  ASSERT_TRUE(deliverCatalog(layer, catalogOf({f.keep}, 0)));

  EXPECT_EQ(layer->residentTileCount(), std::size_t(2));   // reconciler pruned nothing
  EXPECT_EQ(layer->overviewTileCount(), before);
  for(const gggs::GridIndex& a : f.drop_only)
  {
    EXPECT_NE(layer->overviewTileForTest(a), nullptr) << a;
    EXPECT_TRUE(QFileInfo::exists(f.paths.overviewPath(a))) << a;
  }
}

int main(int argc, char** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  QCoreApplication::setOrganizationName("camp_test");
  QCoreApplication::setApplicationName("test_sonar_live_overview_prune");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
