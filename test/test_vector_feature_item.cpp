// [camp#22] Hit-testing and coordinate validity for one read-only vector feature.
//
// Two defects this pins, both of which are invisible until an operator meets them
// with a real file:
//
//  1. HOVER-TO-INSPECT ON LINES. Qt picks an item by testing the point against
//     `shape()`'s FILL AREA — for the scene's hover dispatch as much as for a
//     click. A line's path is open and encloses no area, so returning the
//     raw path means the cursor never lands on a line feature and the headline
//     "point at a feature to see its attributes" silently does not work for line
//     data (tracklines, contours, cable routes — most of what gets imported).
//     The mission-tree LineString strokes its path for exactly this reason.
//  1b. THE POINT HIT TARGET AND THE NO-DATA MARKER, both found in the operator
//     GUI test of 2026-09-15: a 5-pixel marker cannot be hit under the open-hand
//     pan cursor, whose hotspot the operator cannot see, and a hollow no-data
//     marker stroked with a width-0 hairline is not visible on a chart at all —
//     the operator reported colouring by a string field as the features
//     DISAPPEARING.
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
#include <QGraphicsSceneHoverEvent>
#include <QGraphicsSimpleTextItem>
#include <QImage>
#include <QPainter>

#include "map_view/web_mercator.h"
#include "vector/vector_feature_item.h"
#include "vector/vector_parse.h"
#include "vector/vector_style.h"

using camp::vector::ParsedGeometry;
using camp::vector::VectorFeatureItem;
using camp::vector::hasPlaceableCoordinate;
using camp::vector::isPlaceable;
using camp::vector::placeableToMap;

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

ParsedGeometry pointAt(const QGeoCoordinate& coordinate)
{
  ParsedGeometry g;
  g.type = ParsedGeometry::Point;
  g.exterior.push_back(coordinate);
  return g;
}

// Number of pixels @p item touches when painted onto a transparent 64x64 image
// centred on its origin. The item paints in device pixels
// (ItemIgnoresTransformations), so no view transform is needed to make this the
// size an operator sees.
int paintedPixelCount(VectorFeatureItem& item)
{
  QImage image(64, 64, QImage::Format_ARGB32);
  image.fill(Qt::transparent);
  {
    QPainter painter(&image);
    painter.translate(32, 32);
    item.paint(&painter, nullptr, nullptr);
  }
  int painted = 0;
  for(int y = 0; y < image.height(); ++y)
    for(int x = 0; x < image.width(); ++x)
      if(qAlpha(image.pixel(x, y)) > 0)
        ++painted;
  return painted;
}

// The hover label a VectorFeatureItem creates on its first hover-enter, or
// nullptr before that. Found through childItems() rather than through a
// test-only accessor: the label is an ordinary child item, which is exactly what
// the vessel and AIS labels are.
//
// HoverProbe exists because hoverEnterEvent()/hoverLeaveEvent() are protected
// (as Qt declares them) and a QGraphicsItem is not a QObject, so there is no
// QApplication::sendEvent() path to them from outside. Delivering the events
// through a scene is what VectorLayerInteraction does in
// test_vector_layer_teardown.cpp; here the handlers themselves are the subject.
struct HoverProbe: public camp::vector::VectorFeatureItem
{
  using VectorFeatureItem::VectorFeatureItem;
  using VectorFeatureItem::hoverEnterEvent;
  using VectorFeatureItem::hoverLeaveEvent;
};

QGraphicsSimpleTextItem* labelOf(const QGraphicsItem& item)
{
  for(QGraphicsItem* child : item.childItems())
    if(auto* text = dynamic_cast<QGraphicsSimpleTextItem*>(child))
      return text;
  return nullptr;
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

// [camp#22 round-3 should-fix] A polygon whose EXTERIOR is entirely unplaceable is
// rejected even when one of its holes has a valid vertex.
//
// The anchor search used to fall back to interior rings, so such a polygon was
// admitted, the constructor closed an empty exterior and only the hole was built
// — which Qt::OddEvenFill paints as SOLID FILL. A hole drawn as a feature, from a
// file whose coordinates CAMP has already said it cannot place, is worse than the
// honest skip: the layer now counts it in its `skipped` tally like any other
// unplaceable feature.
TEST(VectorFeatureItem, PolygonWithUnplaceableExteriorIsRejectedDespiteAValidHole)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  ParsedGeometry g;
  g.type = ParsedGeometry::Polygon;
  // Eastings/northings read as degrees — the .prj-less shapefile case.
  g.exterior = {QGeoCoordinate(4800000.0, 350000.0), QGeoCoordinate(4800100.0, 350100.0),
                QGeoCoordinate(nan, nan)};
  g.interiorRings = {{QGeoCoordinate(43.05, -70.75), QGeoCoordinate(43.06, -70.74),
                      QGeoCoordinate(43.07, -70.76)}};
  EXPECT_FALSE(hasPlaceableCoordinate(g))
      << "an interior-ring vertex must not anchor a polygon whose exterior is unplaceable";

  // The counterpart: a placeable exterior still qualifies, holes or not.
  g.exterior = {QGeoCoordinate(43.00, -70.80), QGeoCoordinate(43.00, -70.60),
                QGeoCoordinate(43.20, -70.60)};
  EXPECT_TRUE(hasPlaceableCoordinate(g));
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

// [camp#22 should-fix] A polar vertex is CLAMPED to the Web-Mercator limit, not
// projected and not dropped.
//
// isPlaceable() admits latitude +/-90 (it is a perfectly valid WGS84 coordinate),
// but Web Mercator does not converge there: geoToMap() stays finite only because
// tan(pi/2) is 1.633e16 rather than inf in double, and returns y ~ 2.425e8 m —
// about twelve times the world half-extent. One such vertex blows out the layer's
// extent, fit-to-extent and the scene index, exactly as a .prj-less shapefile's
// eastings do. Clamping draws the feature at the edge of the Mercator world,
// which is where the projection puts everything at that latitude anyway.
TEST(VectorFeatureItem, PolarLatitudeIsClampedToTheWebMercatorLimit)
{
  const double half_extent = M_PI * web_mercator::earth_radius_at_equator;  // 2.0037e7 m

  // The raw projection is the problem this clamps.
  ASSERT_GT(std::abs(web_mercator::geoToMap(QGeoCoordinate(90.0, 0.0)).y()), 10.0 * half_extent);

  EXPECT_NEAR(placeableToMap(QGeoCoordinate(90.0, 0.0)).y(), half_extent, 1.0);
  EXPECT_NEAR(placeableToMap(QGeoCoordinate(-90.0, 0.0)).y(), -half_extent, 1.0);
  // Longitude is untouched by the clamp.
  EXPECT_DOUBLE_EQ(placeableToMap(QGeoCoordinate(90.0, -70.71)).x(),
                   web_mercator::geoToMap(QGeoCoordinate(0.0, -70.71)).x());
  // Everything inside the limit is the plain projection.
  const QGeoCoordinate here(43.07, -70.71);
  EXPECT_EQ(placeableToMap(here), web_mercator::geoToMap(here));
}

// And the item built from polar geometry stays inside the Mercator world rather
// than dragging the scene index across twelve worlds of empty space.
TEST(VectorFeatureItem, PolarFeatureStaysInsideTheMercatorWorld)
{
  const double half_extent = M_PI * web_mercator::earth_radius_at_equator;
  VectorFeatureItem item(nullptr, lineThrough({QGeoCoordinate(89.0, -70.8),
                                               QGeoCoordinate(90.0, -70.6)}));
  EXPECT_LE(std::abs(item.pos().y()), half_extent + 1.0);
  const QRectF bounds = item.boundingRect();
  EXPECT_TRUE(std::isfinite(bounds.height()));
  // A line's boundingRect is grown by the click-tolerance half-width (see
  // boundingRect()), so the allowance is a few metres rather than exact — the
  // assertion is about world scale, not about metres.
  const double slack = 10.0;
  EXPECT_LE(std::abs(item.pos().y() + bounds.bottom()), half_extent + slack);
  EXPECT_LE(std::abs(item.pos().y() + bounds.top()), half_extent + slack);
}

// [camp#22 / ADR-0016 D5] INSPECTION IS ON HOVER, and the popup is an IN-SCENE
// LABEL that appears instantly — the vessel/AIS mechanism, not a Qt tooltip.
//
// Hover is CAMP's house convention for "tell me what this is", and the operator
// asked for it after the 2026-09-15 GUI test. A first implementation used the
// item's ordinary Qt tooltip; testing THAT, the operator's verdict was "similar
// to what was existing, but not the same" — CAMP's own items answer the cursor
// with no delay, because `Platform::hoverEnterEvent()` and
// `AISContact::hoverEnterEvent()` set the text of a child QGraphicsSimpleTextItem
// (`GeoGraphicsItem::setShowLabelFlag()`). This pins the replicated mechanism:
// a hover-enter fills a child text item with the attributes, a hover-leave empties
// it — which is also what keeps only ONE label on screen at a time.
TEST(VectorFeatureItem, HoverShowsAnInSceneLabelWithTheAttributes)
{
  ParsedGeometry g;
  g.type = ParsedGeometry::Point;
  g.exterior.push_back(QGeoCoordinate(43.0, -70.0));
  g.attributes["assessment"] = QStringLiteral("candidate C");
  HoverProbe item(nullptr, g);

  ASSERT_FALSE(item.attributeText().isEmpty());
  EXPECT_TRUE(item.acceptHoverEvents())
      << "without hover events the label is never filled in";
  EXPECT_TRUE(item.toolTip().isEmpty())
      << "a tooltip would be a SECOND, delayed popup beside the label";

  // No label item exists before the first hover: a layer may hold up to 50 000
  // features and would otherwise carry 50 000 text items created at load.
  EXPECT_EQ(labelOf(item), nullptr);

  QGraphicsSceneHoverEvent enter(QEvent::GraphicsSceneHoverEnter);
  enter.setPos(QPointF(0.0, 0.0));
  item.hoverEnterEvent(&enter);

  QGraphicsSimpleTextItem* label = labelOf(item);
  ASSERT_NE(label, nullptr) << "hovering the feature created no label";
  EXPECT_EQ(label->text(), item.attributeText());
  EXPECT_TRUE(label->text().contains(QStringLiteral("assessment: candidate C")));
  // The same settings GeoGraphicsItem gives the vessel and AIS labels
  // (geographicsitem.cpp:16-25) — screen-sized, black on a white outline.
  EXPECT_TRUE(label->flags().testFlag(QGraphicsItem::ItemIgnoresTransformations));
  EXPECT_EQ(label->brush().color(), QColor("black"));
  EXPECT_EQ(label->pen().color(), QColor("white"));
  EXPECT_TRUE(label->font().bold());

  // The point branch: the label sits BESIDE the marker, clear of it and of the
  // hit slack, never on top of the symbol it describes.
  EXPECT_GT(label->pos().x(), camp::vector::kDefaultPointRadius)
      << "the label overlaps the marker it labels";
  EXPECT_DOUBLE_EQ(label->pos().y(), 0.0);

  // [camp#22] The hovered ITEM is raised above its siblings, not the label. Feature
  // items are created in file order with no zValue of their own, and Qt stacks a
  // child with its PARENT's subtree — so a label parented to a point paints under
  // a polygon loaded after it no matter how high the label's OWN zValue is set.
  // The label has no zValue set at all (see labelItem()): Qt compares zValue only
  // among siblings, and the label is this item's only child, so a child-level
  // zValue would do nothing.
  EXPECT_GT(item.zValue(), 0.0) << "the hovered feature was not raised";
  EXPECT_DOUBLE_EQ(label->zValue(), 0.0)
      << "the label's own zValue is a no-op among no siblings; it should not be set";

  QGraphicsSceneHoverEvent leave(QEvent::GraphicsSceneHoverLeave);
  item.hoverLeaveEvent(&leave);
  EXPECT_TRUE(labelOf(item)->text().isEmpty())
      << "the label must clear on leave, or every hovered feature keeps one";
  EXPECT_DOUBLE_EQ(item.zValue(), 0.0)
      << "a hovered-once feature stays raised above every other feature";
}

// [camp#22 / ADR-0016 D5] A LINE or POLYGON labels WHERE THE CURSOR ENTERED IT.
//
// The other branch of hoverEnterEvent(): a line may cross the whole view and a
// polygon may fill it, so neither has an anchor worth labelling beside — the text
// goes where the operator is pointing. Untested until now: the point branch is
// the one HoverShowsAnInSceneLabelWithTheAttributes exercises, and a line that
// labelled itself at its first vertex could be metres or kilometres off screen
// from the cursor with nothing failing. The label is placed once, on hover-enter,
// and stays there for the hover — there is no hoverMoveEvent().
TEST(VectorFeatureItem, HoverOnALineOrPolygonLabelsWhereTheCursorEntered)
{
  const QGeoCoordinate a(43.00, -70.80);
  ParsedGeometry line = lineThrough({a, QGeoCoordinate(43.00, -70.60)});
  line.attributes["survey"] = QStringLiteral("line 7");
  HoverProbe item(nullptr, line);
  ASSERT_FALSE(item.isPoint());

  // Item coordinates: scene metres relative to the first vertex, which is where
  // the item is positioned — so a cursor part-way along the line is well away
  // from the item's own origin.
  const QPointF cursor(4210.0, -3.0);
  QGraphicsSceneHoverEvent enter(QEvent::GraphicsSceneHoverEnter);
  enter.setPos(cursor);
  item.hoverEnterEvent(&enter);

  QGraphicsSimpleTextItem* label = labelOf(item);
  ASSERT_NE(label, nullptr) << "hovering the line created no label";
  EXPECT_TRUE(label->text().contains(QStringLiteral("survey: line 7")));
  EXPECT_EQ(label->pos(), cursor)
      << "a line's label must be placed where the cursor ENTERED, not at the item's origin";

  // A second hover elsewhere on the same line moves it.
  const QPointF elsewhere(120.0, 2.0);
  QGraphicsSceneHoverEvent leave(QEvent::GraphicsSceneHoverLeave);
  item.hoverLeaveEvent(&leave);
  QGraphicsSceneHoverEvent again(QEvent::GraphicsSceneHoverEnter);
  again.setPos(elsewhere);
  item.hoverEnterEvent(&again);
  EXPECT_EQ(label->pos(), elsewhere);

  // A polygon takes the same branch.
  ParsedGeometry poly;
  poly.type = ParsedGeometry::Polygon;
  poly.exterior = {a, QGeoCoordinate(43.00, -70.60), QGeoCoordinate(43.05, -70.60)};
  poly.attributes["zone"] = QStringLiteral("A");
  HoverProbe area(nullptr, poly);
  ASSERT_TRUE(area.isPolygon());
  QGraphicsSceneHoverEvent in_area(QEvent::GraphicsSceneHoverEnter);
  in_area.setPos(cursor);
  area.hoverEnterEvent(&in_area);
  ASSERT_NE(labelOf(area), nullptr);
  EXPECT_EQ(labelOf(area)->pos(), cursor);
}

// [camp#22] A size-by-field restyle under a PARKED cursor moves the label with
// the marker.
//
// The point label's gap is computed from radius_, and applyStyle() can call
// setRadius() at any time — including while the cursor is sitting on the feature,
// which is exactly when the operator is looking at the label. Recomputed at every
// prepareGeometryChange() site rather than only at hover-enter, or a grown marker
// paints over its own text until the operator hovers away and back.
TEST(VectorFeatureItem, RestylingUnderAParkedCursorMovesTheLabel)
{
  ParsedGeometry g = pointAt(QGeoCoordinate(43.07, -70.71));
  g.attributes["depth"] = 12.5;
  HoverProbe item(nullptr, g);

  QGraphicsSceneHoverEvent enter(QEvent::GraphicsSceneHoverEnter);
  enter.setPos(QPointF(0.0, 0.0));
  item.hoverEnterEvent(&enter);
  QGraphicsSimpleTextItem* label = labelOf(item);
  ASSERT_NE(label, nullptr);
  const double at_default = label->pos().x();

  // The cursor has not moved; only the styling has.
  item.setRadius(camp::vector::kDefaultPointRadius + 15.0);
  EXPECT_DOUBLE_EQ(label->pos().x(), at_default + 15.0)
      << "the label kept the gap of the OLD radius and now sits on the marker";

  item.setRadius(camp::vector::kDefaultPointRadius);
  EXPECT_DOUBLE_EQ(label->pos().x(), at_default);
}

// [camp#22 / camp#225] A feature accepts NO mouse button, so every press over it
// reaches the view.
//
// This is what makes camp#225 (a pan that starts on a feature does not pan) fixed
// by construction rather than by a gate: there is nothing left to gate. It is
// asserted on the ITEM here — QGraphicsItem accepts the left button by default,
// so the absence of this call is a silent regression — and through a real view in
// VectorLayerInteraction.APressOverAFeatureFallsThroughToTheView.
TEST(VectorFeatureItem, AcceptsNoMouseButtonSoThePressReachesTheView)
{
  VectorFeatureItem item(nullptr, pointAt(QGeoCoordinate(43.07, -70.71)));
  EXPECT_EQ(item.acceptedMouseButtons(), Qt::NoButton);
}

// [camp#22] A point marker's HIT target is wider than the marker itself.
//
// The drawn marker is kDefaultPointRadius = 5 device pixels, and in the operator
// GUI test of 2026-09-15 nobody managed to land the cursor inside it: the pan
// cursor is an open hand whose hotspot is not visible, so a 5 px target is aimed
// at blind and inspection read as "there is no popup". shape() therefore
// carries kPointHoverSlackPixels (4 px) of slack around the marker. Nothing drawn
// grows — this is the target, not the symbol. The slack was added for a click and
// serves hover unchanged: shape() is what the scene hit-tests to dispatch hover
// events too. (ADR-0016 D16 attacks the same problem from the other end — the pan
// cursor is an arrow now — and the slack stays, because aim is never exact.)
TEST(VectorFeatureItem, PointHoverTargetIsWiderThanTheDrawnMarker)
{
  VectorFeatureItem item(nullptr, pointAt(QGeoCoordinate(43.07, -70.71)));
  ASSERT_TRUE(item.isPoint());

  const QPainterPath shape = item.shape();
  // Dead centre, and inside the drawn marker: both always worked.
  EXPECT_TRUE(shape.contains(QPointF(0.0, 0.0)));
  EXPECT_TRUE(shape.contains(QPointF(3.0, 0.0)));
  // 7 px out: OUTSIDE the 5 px marker, inside the 9 px target. This is the case
  // the slack exists for, and the one that fails without it.
  EXPECT_TRUE(shape.contains(QPointF(7.0, 0.0)))
      << "the cursor just outside the marker must still hit the feature";
  EXPECT_TRUE(shape.contains(QPointF(0.0, -7.0)));
  // The slack is bounded: a cursor well away from the marker is not this feature's.
  EXPECT_FALSE(shape.contains(QPointF(20.0, 0.0)));

  // Qt requires shape() to lie inside boundingRect(); a shape outside it is
  // undefined behaviour, so the slack has to be carried into both.
  EXPECT_TRUE(item.boundingRect().contains(shape.boundingRect()));
}

// [camp#22] The hit shape is CACHED, and the cache tracks the radius.
//
// shape() is called by the scene's hit test on every mouse-move for every item
// whose bounding rect is under the cursor — a long polyline's bounding rect
// covers most of the map — so it must not re-stroke the path each time; the
// stroke is built once, in the constructor and at every prepareGeometryChange()
// site. That the RESULT is a stable object is what a test can see (Qt gives no
// stroker call count), and the failure mode caching introduces is a STALE shape:
// a size-by-field restyle must move the hover target with the marker.
TEST(VectorFeatureItem, HitShapeIsCachedAndFollowsTheRadius)
{
  VectorFeatureItem item(nullptr, pointAt(QGeoCoordinate(43.07, -70.71)));
  EXPECT_EQ(item.shape(), item.shape()) << "two calls answered differently";

  // Grown by a size-by-field pass: the target must grow with the marker.
  item.setRadius(20.0);
  EXPECT_TRUE(item.shape().contains(QPointF(22.0, 0.0)))
      << "the hit shape is stale: it still has the old radius";
  EXPECT_TRUE(item.boundingRect().contains(item.shape().boundingRect()));

  // And back down: a stale cache would leave the old, too-wide target behind.
  item.setRadius(camp::vector::kDefaultPointRadius);
  EXPECT_FALSE(item.shape().contains(QPointF(22.0, 0.0)));

  // A LINE's stroked ribbon is the expensive one; it is stable too.
  const QGeoCoordinate a(43.00, -70.80);
  VectorFeatureItem line(nullptr, lineThrough({a, QGeoCoordinate(43.00, -70.60)}));
  const QPainterPath first = line.shape();
  ASSERT_FALSE(first.isEmpty());
  EXPECT_EQ(first, line.shape());
}

// [camp#22] The NO-DATA marker is visible.
//
// A no-data point is drawn hollow and dashed (ADR-0016 D6's second channel), and
// it used to be stroked with the same width-0 hairline the filled marker gets for
// contrast. The filled marker has a disc of colour behind that hairline; the
// hollow one has nothing, so one dashed device pixel of mid grey over a chart
// background is effectively not there — the operator's GUI test of 2026-09-15
// reported colouring by a string field (which marks every feature no-data) as the
// features DISAPPEARING.
//
// "Visible" is asserted against a hairline reference the test draws itself —
// the same ellipse, the same dash, at width 0 — rather than against an absolute
// pixel count, so the assertion stays true of any marker size and any antialiasing
// behaviour: what is pinned is that the no-data ring is drawn HEAVIER than a
// hairline, which is exactly what changed.
TEST(VectorFeatureItem, NoDataPointMarkerIsDrawnHeavierThanAHairline)
{
  VectorFeatureItem item(nullptr, pointAt(QGeoCoordinate(43.07, -70.71)));
  item.setColor(camp::vector::noDataColor());
  item.setNoData(true);
  ASSERT_TRUE(item.isNoData());

  const int no_data_pixels = paintedPixelCount(item);
  ASSERT_GT(no_data_pixels, 0) << "the no-data marker painted nothing at all";

  // The reference: the same dashed ring at the old width-0 hairline.
  QImage reference(64, 64, QImage::Format_ARGB32);
  reference.fill(Qt::transparent);
  {
    QPainter painter(&reference);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.translate(32, 32);
    QPen pen(camp::vector::noDataColor());
    pen.setWidth(0);
    pen.setStyle(Qt::DashLine);
    painter.setPen(pen);
    painter.setBrush(Qt::NoBrush);
    painter.drawEllipse(QPointF(0.0, 0.0), camp::vector::kDefaultPointRadius,
                        camp::vector::kDefaultPointRadius);
  }
  int hairline_pixels = 0;
  for(int y = 0; y < reference.height(); ++y)
    for(int x = 0; x < reference.width(); ++x)
      if(qAlpha(reference.pixel(x, y)) > 0)
        ++hairline_pixels;
  ASSERT_GT(hairline_pixels, 0) << "harness: the hairline reference painted nothing";

  EXPECT_GT(no_data_pixels, hairline_pixels)
      << "the no-data marker is still drawn as a hairline: " << no_data_pixels
      << " px against a hairline's " << hairline_pixels;

  // And it stays HOLLOW — the second channel is a RING, not a filled disc, and a
  // heavier pen must not have become a fill. Asserted where "hollow" actually
  // lives: the middle of the marker is untouched. (A pixel budget cannot say
  // this — a dashed width-2 ring with antialiasing covers about as many pixels
  // as a solid disc of the same radius.)
  QImage image(64, 64, QImage::Format_ARGB32);
  image.fill(Qt::transparent);
  {
    QPainter painter(&image);
    painter.translate(32, 32);
    item.paint(&painter, nullptr, nullptr);
  }
  EXPECT_EQ(qAlpha(image.pixel(32, 32)), 0)
      << "the no-data marker filled in — hollow is half of what says 'no value'";
  EXPECT_EQ(qAlpha(image.pixel(34, 32)), 0) << "the marker's interior is painted";
}

int main(int argc, char** argv)
{
  // QGraphicsItem construction touches qApp; offscreen keeps this headless.
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
