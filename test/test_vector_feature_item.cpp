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

int main(int argc, char** argv)
{
  // QGraphicsItem construction touches qApp; offscreen keeps this headless.
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
