// [camp#104] Flat-layer spawn coverage for raster::GggsStoreSource::instantiate
// (ADR-0005). Selecting a tile-set in the catalog browser must add exactly one
// flat top-level GggsTileLayer to the Map's top-level layers, keep the Map model
// valid (no QAbstractItemModelTester protocol violation), and dedup so the same
// directory selected twice does not spawn a second layer (which its
// directory()-keyed onRemovedFromMap would otherwise orphan).
//
// An empty tile-set directory keeps this GL/GDAL-free: the spawned GggsTileLayer
// holds no tiles, so no pixel load or GL context is created.

#include <gtest/gtest.h>

#include <atomic>

#include <QApplication>
#include <QAbstractItemModelTester>
#include <QDir>
#include <QFile>
#include <QSettings>
#include <QTemporaryDir>

#include "catalog/catalog_item.h"
#include "catalog/catalog_model.h"
#include "map/map.h"
#include "map/layer_list.h"
#include "map/map_item.h"
#include "raster/gggs_store_source.h"
#include "raster/gggs_tile_layer.h"

using camp::catalog::CatalogItem;
using camp::catalog::CatalogModel;
using camp::map::Map;
using camp::map::MapItem;
using camp::raster::GggsStoreSource;
using camp::raster::GggsTileLayer;

namespace
{

// Counts Qt warning/critical/fatal messages while in scope (mirrors
// test_map_model's trap): a non-zero count after a mutation means the
// QAbstractItemModelTester flagged a model-protocol violation.
class WarningTrap
{
public:
  WarningTrap() { count() = 0; previous_ = qInstallMessageHandler(&WarningTrap::handler); }
  ~WarningTrap() { qInstallMessageHandler(previous_); }
  int messages() const { return count().load(); }

private:
  static std::atomic<int>& count() { static std::atomic<int> c{0}; return c; }
  static void handler(QtMsgType type, const QMessageLogContext& ctx, const QString& msg)
  {
    if(type == QtWarningMsg || type == QtCriticalMsg || type == QtFatalMsg)
      count().fetch_add(1);
    if(previous_)
      previous_(type, ctx, msg);
  }
  static QtMessageHandler previous_;
};
QtMessageHandler WarningTrap::previous_ = nullptr;

// Create an empty placeholder tile file `<dir>/<name>` — discovery only matches
// the `*.tif` filename, so the content is irrelevant (no GL/GDAL).
void touchTif(const QString& dir, const QString& name)
{
  QDir().mkpath(dir);
  QFile f(QDir(dir).filePath(name));
  ASSERT_TRUE(f.open(QIODevice::WriteOnly));
  f.close();
}

int gggsLayersOn(camp::map::LayerList* layers, const QString& dir)
{
  int n = 0;
  for(MapItem* child : layers->childMapItems())
    if(auto* g = dynamic_cast<GggsTileLayer*>(child))
      if(g->directory() == dir)
        ++n;
  return n;
}

}  // namespace

// instantiate() adds exactly one flat GggsTileLayer under topLevelLayers and the
// Map model stays valid.
TEST(GggsFlatLayerSpawn, InstantiateAddsOneFlatLayer)
{
  QSettings().clear();
  Map map;
  WarningTrap trap;
  QAbstractItemModelTester tester(&map, QAbstractItemModelTester::FailureReportingMode::Warning);

  auto* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);
  const int before = layers->childMapItems().size();

  QTemporaryDir tileset;   // empty -> no tiles, no GL/GDAL
  ASSERT_TRUE(tileset.isValid());

  GggsStoreSource source;
  camp::map::Layer* layer = source.instantiate(layers, tileset.path());

  ASSERT_NE(layer, nullptr);
  EXPECT_EQ(layers->childMapItems().size(), before + 1);
  EXPECT_EQ(gggsLayersOn(layers, tileset.path()), 1);
  auto* tile_layer = dynamic_cast<GggsTileLayer*>(layer);
  ASSERT_NE(tile_layer, nullptr);
  EXPECT_EQ(tile_layer->directory(), tileset.path());
  EXPECT_EQ(trap.messages(), 0) << "model-protocol warning during flat-layer spawn";
}

// Selecting the same tile-set twice returns the existing layer — no duplicate.
TEST(GggsFlatLayerSpawn, DedupOnSelect)
{
  QSettings().clear();
  Map map;
  auto* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);

  QTemporaryDir tileset;
  ASSERT_TRUE(tileset.isValid());

  GggsStoreSource source;
  camp::map::Layer* first = source.instantiate(layers, tileset.path());
  const int after_first = layers->childMapItems().size();
  camp::map::Layer* second = source.instantiate(layers, tileset.path());

  EXPECT_EQ(second, first) << "re-selecting a displayed tile-set must reuse its layer";
  EXPECT_EQ(layers->childMapItems().size(), after_first) << "no duplicate spawn";
  EXPECT_EQ(gggsLayersOn(layers, tileset.path()), 1);
}

// A CatalogModel populated by GggsStoreSource::discover() satisfies the
// QAbstractItemModel protocol directly — index/parent/rowCount/columnCount/data
// over the discovered group/leaf tree, not just indirectly through the Map model
// (the spawn tests above). QAbstractItemModelTester validates the insertion path
// (addTopLevel wraps begin/endInsertRows) and every index round-trip; the
// WarningTrap turns any protocol violation into a test failure. Discovery only
// stats directories for `*.tif`, so empty placeholder tiles keep this GL/GDAL-free.
TEST(GggsCatalogModel, SatisfiesItemModelProtocol)
{
  QTemporaryDir store;
  ASSERT_TRUE(store.isValid());
  touchTif(store.path() + "/bathymetry/draft/tileset_a", "0_0_0.tif");
  touchTif(store.path() + "/sidescan/tileset_b", "0_0_0.tif");
  touchTif(store.path() + "/sidescan/tileset_b", "0_0_1.tif");

  GggsStoreSource source;
  std::unique_ptr<CatalogItem> tree = source.discover(store.path());
  ASSERT_NE(tree, nullptr);

  CatalogModel model;
  WarningTrap trap;
  // Attach the tester BEFORE population so it validates the insertion signals too.
  QAbstractItemModelTester tester(&model, QAbstractItemModelTester::FailureReportingMode::Warning);
  model.addTopLevel(std::move(tree));

  // One browsed store root at the top level, single-column tree.
  ASSERT_EQ(model.rowCount(QModelIndex()), 1);
  EXPECT_EQ(model.columnCount(QModelIndex()), 1);

  // Walk one index round-trip: root -> a child group, and parent() back.
  const QModelIndex root_index = model.index(0, 0, QModelIndex());
  ASSERT_TRUE(root_index.isValid());
  EXPECT_FALSE(model.parent(root_index).isValid()) << "top-level node has no parent";
  ASSERT_GT(model.rowCount(root_index), 0) << "store root has tile-bearing children";
  const QModelIndex child_index = model.index(0, 0, root_index);
  ASSERT_TRUE(child_index.isValid());
  EXPECT_EQ(model.parent(child_index), root_index) << "child parent() round-trips to its parent";

  EXPECT_EQ(trap.messages(), 0) << "QAbstractItemModelTester flagged a CatalogModel protocol violation";
}

int main(int argc, char** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  QCoreApplication::setOrganizationName("camp_test");
  QCoreApplication::setApplicationName("test_gggs_flat_layer_spawn");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
