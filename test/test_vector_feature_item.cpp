// [camp#22] Hit-testing and coordinate validity for one read-only vector feature.
//
// Two defects this pins, both of which are invisible until an operator meets them
// with a real file:
//
//  1. CLICK-TO-INSPECT ON LINES. Qt picks an item by testing the point against
//     `shape()`'s FILL AREA. A line's path is open and encloses no area, so
//     returning the raw path means no click ever lands on a line feature and the
//     headline "click a feature to see its attributes" silently does not work for
//     line data (tracklines, contours, cable routes — most of what gets imported).
//     The mission-tree LineString strokes its path for exactly this reason.
//  2. UNPLACEABLE COORDINATES. A shapefile shipped without its `.prj` sidecar has
//     no spatial reference, so projected eastings/northings are read as degrees: a
//     UTM northing of 4 800 000 becomes "latitude 4800000". A failed coordinate
//     transform produces NaN. Either one, projected into the scene, corrupts the
//     layer's extent (fit-to-extent flies to nowhere) and the scene index for
//     every other feature in the file.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>

#include <QApplication>
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsView>
#include <QToolTip>

#include "map_view/web_mercator.h"
#include "vector/vector_feature_item.h"
#include "vector/vector_parse.h"

using camp::vector::ParsedGeometry;
using camp::vector::VectorFeatureItem;
using camp::vector::hasPlaceableCoordinate;
using camp::vector::isPlaceable;

namespace
{

ParsedGeometry lineThrough(const std::vector<QGeoCoordinate>& vertices)
{
  ParsedGeometry g;
  g.type = ParsedGeometry::LineString;
  g.exterior = vertices;
  return g;
}

// Item-local coordinates: the item is positioned at its first placeable vertex.
QPointF itemPoint(const QGeoCoordinate& coordinate, const QGeoCoordinate& anchor)
{
  return web_mercator::geoToMap(coordinate) - web_mercator::geoToMap(anchor);
}

}  // namespace

// A click on a line — anywhere along it, not only at a vertex — hits the feature.
// With the raw open path this fails outright: an open path has no fill area.
TEST(VectorFeatureItem, LineShapeIsClickable)
{
  const QGeoCoordinate a(43.00, -70.80);
  const QGeoCoordinate b(43.00, -70.60);
  VectorFeatureItem item(nullptr, lineThrough({a, b}));

  const QPainterPath shape = item.shape();
  ASSERT_FALSE(shape.isEmpty());

  // Midway along the segment, and right on it.
  const QGeoCoordinate mid(43.00, -70.70);
  EXPECT_TRUE(shape.contains(itemPoint(mid, a))) << "a click on the line must hit it";
  // Each endpoint too.
  EXPECT_TRUE(shape.contains(itemPoint(a, a)));
  EXPECT_TRUE(shape.contains(itemPoint(b, a)));

  // Well off the line, it does not.
  const QGeoCoordinate off(43.05, -70.70);
  EXPECT_FALSE(shape.contains(itemPoint(off, a)));

  // Qt requires shape() to lie inside boundingRect(); the stroke is wider than
  // the path, so the bounding rect has to account for it.
  EXPECT_TRUE(item.boundingRect().contains(shape.boundingRect()));
}

// A polygon keeps its filled path as its shape — clicking inside it hits it.
TEST(VectorFeatureItem, PolygonShapeIsItsFilledPath)
{
  ParsedGeometry g;
  g.type = ParsedGeometry::Polygon;
  g.exterior = {QGeoCoordinate(43.00, -70.80), QGeoCoordinate(43.00, -70.60),
                QGeoCoordinate(43.20, -70.60), QGeoCoordinate(43.20, -70.80)};
  VectorFeatureItem item(nullptr, g);

  const QPointF inside = itemPoint(QGeoCoordinate(43.10, -70.70), g.exterior.front());
  EXPECT_TRUE(item.shape().contains(inside));
  const QPointF outside = itemPoint(QGeoCoordinate(43.40, -70.70), g.exterior.front());
  EXPECT_FALSE(item.shape().contains(outside));
}

// The validity contract: finite, latitude within +/-90, longitude within +/-180.
TEST(VectorFeatureItem, PlaceabilityRejectsTheCoordinatesThatPoisonTheScene)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  EXPECT_TRUE(isPlaceable(QGeoCoordinate(43.07, -70.71)));
  EXPECT_TRUE(isPlaceable(QGeoCoordinate(90.0, 180.0)));
  EXPECT_TRUE(isPlaceable(QGeoCoordinate(-90.0, -180.0)));

  EXPECT_FALSE(isPlaceable(QGeoCoordinate()));            // never set
  EXPECT_FALSE(isPlaceable(QGeoCoordinate(nan, -70.71))); // failed transform
  EXPECT_FALSE(isPlaceable(QGeoCoordinate(43.07, nan)));
  EXPECT_FALSE(isPlaceable(QGeoCoordinate(inf, 0.0)));
  EXPECT_FALSE(isPlaceable(QGeoCoordinate(4800000.0, 350000.0)));  // UTM read as degrees
  EXPECT_FALSE(isPlaceable(QGeoCoordinate(90.1, 0.0)));
  EXPECT_FALSE(isPlaceable(QGeoCoordinate(0.0, 180.1)));
}

// A feature with no placeable vertex is reported as such, so the layer can skip
// it instead of building an item positioned nowhere.
TEST(VectorFeatureItem, FeatureWithNoPlaceableVertexIsRejected)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(hasPlaceableCoordinate(lineThrough({QGeoCoordinate(nan, nan),
                                                   QGeoCoordinate(4800000.0, 350000.0)})));
  EXPECT_FALSE(hasPlaceableCoordinate(lineThrough({})));
  EXPECT_TRUE(hasPlaceableCoordinate(lineThrough({QGeoCoordinate(nan, nan),
                                                  QGeoCoordinate(43.0, -70.0)})));
}

// A partly-unplaceable line drops only the bad vertices: the item is anchored at
// the first good one and its extent stays local, instead of spanning the world.
TEST(VectorFeatureItem, UnplaceableVerticesDoNotStretchTheItem)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const QGeoCoordinate a(43.00, -70.80);
  const QGeoCoordinate b(43.00, -70.60);
  VectorFeatureItem item(nullptr, lineThrough({QGeoCoordinate(4800000.0, 350000.0),
                                               a, QGeoCoordinate(nan, nan), b}));

  const QRectF bounds = item.boundingRect();
  ASSERT_FALSE(bounds.isEmpty());
  EXPECT_TRUE(std::isfinite(bounds.width()));
  EXPECT_TRUE(std::isfinite(bounds.height()));
  // The two good vertices are ~16 km apart; anything world-scale means a bad one
  // was projected anyway.
  EXPECT_LT(bounds.width(), 1.0e5);
  EXPECT_LT(bounds.height(), 1.0e5);
  EXPECT_TRUE(std::isfinite(item.pos().x()));
  EXPECT_TRUE(std::isfinite(item.pos().y()));
  EXPECT_EQ(item.pos(), web_mercator::geoToMap(a));
}

// [camp#22] Click-to-inspect fires on RELEASE WITHOUT MOVEMENT, and only when the
// view the event came from is in pan mode.
//
// ProjectView places waypoints on left-press in its add-* modes, so a popup on
// press would appear mid-placement; and a press that turns into a drag is the
// operator panning across the map, not asking about the feature they happened to
// start on. The mode is read from the EVENT's view rather than from any attached
// view, which matters as soon as a scene has more than one.
TEST(VectorFeatureItem, PopupIsGatedOnPanModeAndOnAClickNotADrag)
{
  QGraphicsScene scene;
  QGraphicsView view(&scene);
  ParsedGeometry g;
  g.type = ParsedGeometry::Point;
  g.exterior.push_back(QGeoCoordinate(43.0, -70.0));
  // An attribute, so the popup has something to say and "shown" is
  // distinguishable from "suppressed" by the tooltip's TEXT.
  g.attributes["assessment"] = QStringLiteral("candidate C");
  auto* item = new VectorFeatureItem(nullptr, g);
  scene.addItem(item);
  ASSERT_FALSE(item->attributeText().isEmpty());

  auto press = [&](const QPoint& screen)
  {
    QGraphicsSceneMouseEvent event(QEvent::GraphicsSceneMousePress);
    event.setButton(Qt::LeftButton);
    event.setWidget(view.viewport());
    event.setScenePos(item->pos());
    event.setScreenPos(screen);
    event.setButtonDownScreenPos(Qt::LeftButton, screen);
    event.setAccepted(false);
    scene.sendEvent(item, &event);
    return event.isAccepted();
  };
  auto release = [&](const QPoint& down, const QPoint& up)
  {
    QGraphicsSceneMouseEvent event(QEvent::GraphicsSceneMouseRelease);
    event.setButton(Qt::LeftButton);
    event.setWidget(view.viewport());
    event.setScenePos(item->pos());
    event.setScreenPos(up);
    event.setButtonDownScreenPos(Qt::LeftButton, down);
    event.setAccepted(false);
    scene.sendEvent(item, &event);
    return event.isAccepted();
  };

  // An add-* mode (NoDrag): the press is IGNORED, so placement is untouched.
  view.setDragMode(QGraphicsView::NoDrag);
  EXPECT_FALSE(press(QPoint(100, 100)));

  // Pan mode: the press is taken but answers nothing yet — whether this is a
  // click or the start of a pan is not known until the button comes back up.
  //
  // What is asserted on the release is the TOOLTIP, not isAccepted(): the item
  // accepts the release on both the click and the drag path (a drag it started on
  // is still its event), so acceptance cannot tell "popup shown" from "popup
  // suppressed" — the one thing this test exists to distinguish.
  view.setDragMode(QGraphicsView::ScrollHandDrag);
  EXPECT_TRUE(press(QPoint(100, 100)));
  EXPECT_TRUE(release(QPoint(100, 100), QPoint(100, 100)));   // a click
  EXPECT_EQ(QToolTip::text(), item->attributeText())
      << "a click without movement must show the feature's attributes";
  EXPECT_FALSE(QToolTip::text().isEmpty());

  // Now the drag. "No popup" is asserted as "the tooltip text did not change",
  // against a sentinel planted first: QToolTip::hideText() does not clear
  // QToolTip::text() on the offscreen platform, so an emptiness assertion here
  // would be testing the platform rather than the item.
  const QString sentinel = QStringLiteral("sentinel: no popup was shown");
  QToolTip::showText(QPoint(0, 0), sentinel);
  ASSERT_EQ(QToolTip::text(), sentinel) << "harness: the sentinel did not take";

  EXPECT_TRUE(press(QPoint(100, 100)));
  EXPECT_TRUE(release(QPoint(100, 100), QPoint(400, 250)));   // a drag: no popup
  EXPECT_EQ(QToolTip::text(), sentinel)
      << "a release far from the press is a pan, and must not answer with a tooltip";

  // A right-button release is not ours.
  QGraphicsSceneMouseEvent right(QEvent::GraphicsSceneMouseRelease);
  right.setButton(Qt::RightButton);
  right.setWidget(view.viewport());
  right.setAccepted(false);
  scene.sendEvent(item, &right);
  EXPECT_FALSE(right.isAccepted());
}

int main(int argc, char** argv)
{
  // QGraphicsItem construction touches qApp; offscreen keeps this headless.
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
