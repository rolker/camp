// [#59 ADR-0002/0003] Model-correctness coverage for camp::map::Map — the layer
// tree model the deployed CAMP now runs on after the camp2 map-system port.
//
// It validates the QAbstractItemModel protocol (balanced begin/end{Insert,
// Remove}Rows, valid indices, consistent row counts) across the exact
// model-mutation paths the BackgroundRaster retirement introduced:
//   - stacked-layer inserts (AutonomousVehicleProject::openBackground), and
//   - detach-and-delete removal (the Layers-tab Remove action's
//     setMapItemParent(layer, nullptr) + delete).
//
// Until now this protocol was only checked at runtime by the camp2 sandbox app's
// QAbstractItemModelTester (camp2/main/main_window.cpp), which never runs in CI.
// This lifts that check into the gtest suite so a row-accounting regression on
// these paths fails the build instead of crashing the deployed app in the field.

#include <gtest/gtest.h>

#include <atomic>

#include <QApplication>
#include <QAbstractItemModelTester>

#include "map/map.h"
#include "map/layer.h"
#include "map/layer_list.h"
#include "map/map_item.h"

using camp::map::Layer;
using camp::map::Map;
using camp::map::MapItem;

namespace
{

// Counts Qt warning/critical/fatal messages while in scope. In Warning mode the
// QAbstractItemModelTester reports every model-protocol violation via qWarning,
// so a non-zero count after a mutation means the model misbehaved — caught as a
// clean test failure rather than a process abort (Fatal mode).
class WarningTrap
{
public:
  WarningTrap() { count() = 0; previous_ = qInstallMessageHandler(&WarningTrap::handler); }
  ~WarningTrap() { qInstallMessageHandler(previous_); }
  int messages() const { return count().load(); }

private:
  static std::atomic<int>& count()
  {
    static std::atomic<int> c{0};
    return c;
  }
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

// Layers currently under the map's top-level layer list.
QList<MapItem*> topLevelLayerItems(Map& map)
{
  return map.topLevelLayers()->childMapItems();
}

} // namespace

// Stacking charts: each load appends a layer under topLevelLayers (mirrors
// AutonomousVehicleProject::openBackground). The model must see one inserted row
// per layer with no protocol violation.
TEST(MapModel, StackedInsertsKeepModelValid)
{
  Map map;
  QAbstractItemModelTester tester(&map, QAbstractItemModelTester::FailureReportingMode::Warning);
  WarningTrap trap;

  const int before = topLevelLayerItems(map).size();
  for(int i = 0; i < 3; ++i)
    new Layer(map.topLevelLayers(), QString("chart%1").arg(i));

  EXPECT_EQ(topLevelLayerItems(map).size(), before + 3);
  EXPECT_EQ(trap.messages(), 0) << "model-protocol warning during stacked inserts";
}

// The Layers-tab Remove action detaches a layer via setMapItemParent(layer,
// nullptr) (firing rowsAboutToBeRemoved) and deletes it. Removing the middle of
// a stack must keep the model consistent and leave the other layers intact.
TEST(MapModel, DetachAndDeleteRemovesOneRow)
{
  Map map;
  QAbstractItemModelTester tester(&map, QAbstractItemModelTester::FailureReportingMode::Warning);
  WarningTrap trap;

  const int before = topLevelLayerItems(map).size();
  Layer* a = new Layer(map.topLevelLayers(), "a");
  Layer* b = new Layer(map.topLevelLayers(), "b");
  Layer* c = new Layer(map.topLevelLayers(), "c");
  ASSERT_EQ(topLevelLayerItems(map).size(), before + 3);

  // Remove the middle layer exactly the way the Layers-tab action does.
  map.setMapItemParent(b, nullptr);
  delete b;

  const auto remaining = topLevelLayerItems(map);
  EXPECT_EQ(remaining.size(), before + 2);
  EXPECT_TRUE(remaining.contains(a));
  EXPECT_FALSE(remaining.contains(b));
  EXPECT_TRUE(remaining.contains(c));
  EXPECT_EQ(trap.messages(), 0) << "model-protocol warning during detach+delete";
}

// Reordering a layer within the stack (drag/drop, and the basis for chart
// stacking order) is a model move = remove + insert. It must stay valid and
// preserve the layer set.
TEST(MapModel, ReorderKeepsModelValid)
{
  Map map;
  QAbstractItemModelTester tester(&map, QAbstractItemModelTester::FailureReportingMode::Warning);
  WarningTrap trap;

  const int before = topLevelLayerItems(map).size();
  Layer* a = new Layer(map.topLevelLayers(), "a");
  new Layer(map.topLevelLayers(), "b");
  new Layer(map.topLevelLayers(), "c");
  ASSERT_EQ(topLevelLayerItems(map).size(), before + 3);

  // Move 'a' to the end of the list (row == -1).
  map.setMapItemParent(a, map.topLevelLayers(), -1);

  const auto items = topLevelLayerItems(map);
  EXPECT_EQ(items.size(), before + 3);
  EXPECT_TRUE(items.contains(a));
  EXPECT_EQ(trap.messages(), 0) << "model-protocol warning during reorder";
}

int main(int argc, char** argv)
{
  // Map builds a QGraphicsScene and MapItem ctors touch qApp, so a QApplication
  // is required; force the offscreen platform so the test runs headless in CI.
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
