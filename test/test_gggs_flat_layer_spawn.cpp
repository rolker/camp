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
#include <QSettings>
#include <QTemporaryDir>

#include "map/map.h"
#include "map/layer_list.h"
#include "map/map_item.h"
#include "raster/gggs_store_source.h"
#include "raster/gggs_tile_layer.h"

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

int main(int argc, char** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  QCoreApplication::setOrganizationName("camp_test");
  QCoreApplication::setApplicationName("test_gggs_flat_layer_spawn");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
