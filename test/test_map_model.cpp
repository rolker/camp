// [#59 ADR-0002/0003] Model-correctness coverage for camp::map::Map — the layer
// tree model the deployed CAMP now runs on after the camp_map map-system port.
//
// It validates the QAbstractItemModel protocol (balanced begin/end{Insert,
// Remove}Rows, valid indices, consistent row counts) across the exact
// model-mutation paths the BackgroundRaster retirement introduced:
//   - stacked-layer inserts (AutonomousVehicleProject::openBackground), and
//   - detach-and-delete removal (the Layers-tab Remove action's
//     setMapItemParent(layer, nullptr) + delete).
//
// Previously this protocol was only checked at runtime by the (now-retired)
// camp2 sandbox app's QAbstractItemModelTester, which never ran in CI. This
// lifts that check into the gtest suite so a row-accounting regression on these
// paths fails the build instead of crashing the deployed app in the field.

#include <gtest/gtest.h>

#include <atomic>

#include <QApplication>
#include <QAbstractItemModelTester>
#include <QMimeData>

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
  WarningTrap trap;
  QAbstractItemModelTester tester(&map, QAbstractItemModelTester::FailureReportingMode::Warning);

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
  WarningTrap trap;
  QAbstractItemModelTester tester(&map, QAbstractItemModelTester::FailureReportingMode::Warning);

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
  WarningTrap trap;
  QAbstractItemModelTester tester(&map, QAbstractItemModelTester::FailureReportingMode::Warning);

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

// ---- helpers for the drag-drop reorder path (issue #84) ----

namespace
{
// Index of the LayerList container under the model root.
QModelIndex layerListIndex(Map& map)
{
  const int n = map.rowCount(QModelIndex());
  for(int i = 0; i < n; ++i)
  {
    auto idx = map.index(i, 0, QModelIndex());
    if(reinterpret_cast<MapItem*>(idx.internalPointer()) == map.topLevelLayers())
      return idx;
  }
  return QModelIndex();
}

// Display order (top -> bottom) of layer names under the LayerList.
QStringList displayOrder(Map& map)
{
  QStringList out;
  auto parent = layerListIndex(map);
  const int n = map.rowCount(parent);
  for(int i = 0; i < n; ++i)
    out << map.index(i, 0, parent).data(Qt::EditRole).toString();
  return out;
}

// Index of the layer named `name` under the LayerList (search by display).
QModelIndex layerIndex(Map& map, const QString& name)
{
  auto parent = layerListIndex(map);
  const int n = map.rowCount(parent);
  for(int i = 0; i < n; ++i)
  {
    auto idx = map.index(i, 0, parent);
    if(idx.data(Qt::EditRole).toString() == name)
      return idx;
  }
  return QModelIndex();
}

// Drive a reorder exactly as the view does: build the model's own mime for the
// dragged layer, then ask the model to drop it at display `row` under LayerList.
bool dropLayerAt(Map& map, const QString& name, int row)
{
  auto src = layerIndex(map, name);
  auto* mime = map.mimeData({src});
  bool ok = map.dropMimeData(mime, Qt::MoveAction, row, 0, layerListIndex(map));
  delete mime;
  return ok;
}
} // namespace

// [#84] Drag-drop reorder regression coverage for the Layers tab.
//
// The "drops only land at the first spot" bug was a view/flags interaction, not
// a model-math bug: a leaf Layer was ItemIsDropEnabled, so a drop in the middle
// of its row stayed an OnItem drop (which Map::dropMimeData rejects) instead of
// being converted by QAbstractItemView to an Above/Below drop targeting the
// LayerList. These tests lock both halves of the fix:
//   1. flag contract — a leaf layer is draggable but NOT a drop target, while
//      the LayerList container is; this is what makes the view convert mid-row
//      drops into between-row (reorderable) drops.
//   2. drop-path correctness — driving Map::dropMimeData through the LayerList
//      parent (the exact call the view makes for a between-row drop) places the
//      layer at the requested display row.

// A leaf layer must be drag-enabled but not drop-enabled (see Layer::updateFlags).
TEST(MapModel, LeafLayerIsDraggableNotDroppable)
{
  Map map;
  new Layer(map.topLevelLayers(), "leaf");
  auto idx = layerIndex(map, "leaf");
  ASSERT_TRUE(idx.isValid());
  const auto f = map.flags(idx);
  EXPECT_TRUE(f & Qt::ItemIsDragEnabled) << "layer should be draggable";
  EXPECT_FALSE(f & Qt::ItemIsDropEnabled)
      << "a drop-enabled leaf layer keeps mid-row drops as forbidden OnItem drops "
         "(issue #84) — reordering then only works in the between-row margins";
}

// The LayerList container is the reorder drop target.
TEST(MapModel, LayerListAcceptsDrops)
{
  Map map;
  auto idx = layerListIndex(map);
  // Guard against a false pass: flags() on an invalid index does not reflect the
  // LayerList's own flags, so the EXPECT below would be meaningless.
  ASSERT_TRUE(idx.isValid());
  EXPECT_TRUE(map.flags(idx) & Qt::ItemIsDropEnabled);
}

// Dropping a layer at a display row reorders it there — and keeps the model
// protocol valid (no QAbstractItemModelTester warnings) on the drop path.
TEST(MapModel, DropReordersToTargetRow)
{
  // Each case: drag `who` to display `row`; expect `who` at display index `at`
  // among the three user layers. Initial display order is [c, b, a, defaults...]
  // (last-created renders on top, so shows first). Standard Qt move-before-row
  // semantics: an item dragged downward past its own slot lands one row earlier.
  struct Case { const char* who; int row; int at; };
  const Case cases[] = {
    {"a", 0, 0},  // bottom layer to the very top
    {"a", 1, 1},  // bottom layer into the middle
    {"c", 2, 1},  // top layer dragged down one slot (before old row 2 -> lands at 1)
    {"c", 3, 2},  // top layer to the bottom of the user layers
  };

  for(const auto& c : cases)
  {
    Map m;
    WarningTrap trap;
    QAbstractItemModelTester tester(&m, QAbstractItemModelTester::FailureReportingMode::Warning);
    new Layer(m.topLevelLayers(), "a");
    new Layer(m.topLevelLayers(), "b");
    new Layer(m.topLevelLayers(), "c");

    // The row numbers above assume the user layers occupy the top three display
    // rows in this order. Map seeds default tile layers (openstreetmap, etc.)
    // below them; assert the prefix so a change to those defaults fails loudly
    // here instead of silently shifting what each `row` means.
    ASSERT_EQ(displayOrder(m).mid(0, 3), (QStringList{"c", "b", "a"}));

    ASSERT_TRUE(dropLayerAt(m, c.who, c.row)) << "drop " << c.who << " @ row " << c.row;
    const auto order = displayOrder(m);
    EXPECT_EQ(order.indexOf(c.who), c.at)
        << "drop " << c.who << " @ row " << c.row
        << " -> " << order.join(",").toStdString();
    EXPECT_EQ(trap.messages(), 0)
        << "model-protocol warning on drop " << c.who << " @ row " << c.row;
  }
}

int main(int argc, char** argv)
{
  // Map builds a QGraphicsScene and MapItem ctors touch qApp, so a QApplication
  // is required; force the offscreen platform so the test runs headless in CI.
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  // [camp#117] Map construction now writes QSettings (the tile-layer seed);
  // a test org/app name keeps that out of the developer's real camp settings.
  QCoreApplication::setOrganizationName("camp_test");
  QCoreApplication::setApplicationName("test_map_model");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
