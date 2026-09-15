#ifndef CAMP_VECTOR_FEATURE_ITEM_H
#define CAMP_VECTOR_FEATURE_ITEM_H

#include <QColor>
#include <QGeoCoordinate>
#include <QGraphicsItem>
#include <QPointF>
#include <QMap>
#include <QPainterPath>
#include <QRectF>
#include <QString>
#include <QVariant>

namespace camp::vector
{

struct ParsedGeometry;

/// [camp#22] One read-only feature of a VectorLayer.
///
/// A plain QGraphicsItem, NOT a map::MapItem and not a QGraphicsObject: a feature
/// is not a row in the Layers tree, has no settings group of its own and needs no
/// signal/slot machinery, and a layer can hold thousands of them. It is also
/// deliberately distinct from the mission-tree Point / LineString / Polygon
/// classes (src/camp/vector/), which are MissionItems carrying editing, dragging
/// and waypoint-linking behaviour a read-only display layer must not inherit.
///
/// Coordinates are Web-Mercator scene metres (ADR-0002), transformed once by the
/// layer when the file finishes loading — never per paint.
///
/// Points ignore the view transform (`ItemIgnoresTransformations`), so their
/// radius is a screen size that stays constant across zoom, like the mission
/// items' symbols. Lines and polygons live in scene space and scale with the
/// view, drawn with a cosmetic pen so the stroke stays one pixel wide.
/// [camp#22] True when @p coordinate can be placed on the Web-Mercator scene:
/// both ordinates finite, latitude within +/-90, longitude within +/-180
/// (`QGeoCoordinate::isValid()`).
///
/// This is not a theoretical guard. A shapefile shipped without its `.prj`
/// sidecar has no spatial reference, so the parser reads its projected eastings
/// and northings as degrees — a UTM northing of 4 800 000 becomes "latitude
/// 4800000", and `geoToMap()` turns that into a position ~1e17 scene metres away.
/// A single such feature poisons the layer's `childrenBoundingRect()` (so
/// fit-to-extent flies to nowhere) and the scene's spatial index. A NaN ordinate,
/// which a failed coordinate transform produces, is worse: every comparison
/// against it is false, and the bounding rect becomes permanently invalid.
bool isPlaceable(const QGeoCoordinate& coordinate);

/// [camp#22] @p coordinate in Web-Mercator scene metres, with its latitude
/// CLAMPED to the projection's own limit (`web_mercator::maximum_latitude`,
/// 85.0511 deg) first. Every geometry conversion in this file goes through here.
///
/// `isPlaceable()` admits latitude +/-90, where Web Mercator does not converge:
/// `geoToMap()` is finite there only because `tan(pi/2)` is 1.633e16 rather than
/// inf in double, and it yields y = +/-2.425e8 m — about twelve times the
/// Web-Mercator world half-extent of 2.004e7 m. A single polar vertex therefore
/// blows out the layer's `childrenBoundingRect()`, fit-to-extent and the scene
/// index just as a `.prj`-less shapefile's eastings do, without being invalid in
/// any way the parser can see.
///
/// The clamp is deliberate and matches what the projection does everywhere else
/// it is used (tile schemes truncate at the same latitude): a polar feature is
/// drawn at the top or bottom edge of the Mercator world rather than being
/// dropped, because dropping it would silently lose real data — a polar survey
/// line is a thing this program should be able to show.
QPointF placeableToMap(const QGeoCoordinate& coordinate);

/// True when @p geometry has at least one placeable coordinate — i.e. when a
/// VectorFeatureItem built from it would land somewhere real. The layer checks
/// this before constructing an item and reports the number of features skipped.
bool hasPlaceableCoordinate(const ParsedGeometry& geometry);

class VectorFeatureItem: public QGraphicsItem
{
public:
  /// @param parent  the owning VectorLayer (untransformed, at the scene origin).
  /// @param geometry  the parsed feature; its coordinates are converted to scene
  ///                  metres here, and its attributes are copied for the popup.
  VectorFeatureItem(QGraphicsItem* parent, const ParsedGeometry& geometry);

  QRectF boundingRect() const override;
  QPainterPath shape() const override;
  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

  /// Resolved paint colour (from the layer's colour-by-field pass, or its default).
  void setColor(const QColor& color);
  const QColor& color() const { return color_; }

  /// Marker radius in device pixels. Points only — lines and polygons ignore
  /// size-by-field entirely (there is no honest reading of "a polygon's size"
  /// from an attribute), which is stated here rather than dropped silently.
  void setRadius(double radius);

  const QMap<QString, QVariant>& attributes() const { return attributes_; }
  bool isPoint() const { return point_; }
  bool isPolygon() const { return polygon_; }

  /// The text shown by the click-to-inspect popup: one "name: value" line per
  /// attribute. Exposed for tests.
  QString attributeText() const;

protected:
  /// Click-to-inspect, gated on the view being in pan mode.
  ///
  /// ProjectView places waypoints, tracklines and survey patterns on left-press
  /// in its add-* modes and forwards the press to the scene either way, so a
  /// popup that always fired would appear in the middle of placing a waypoint.
  /// Pan mode is identified by the drag mode of the view the event came from
  /// (ScrollHandDrag, which ProjectView::setPanMode sets and every add-* mode
  /// clears to NoDrag) rather than by asking ProjectView: this item lives in
  /// camp_map, which cannot call into the executable that links it (#217).
  void mousePressEvent(QGraphicsSceneMouseEvent* event) override;
  /// The popup fires HERE, on release without movement — not on press. A press
  /// that turns into a drag is a pan gesture, and answering it with an attribute
  /// tooltip is an answer to a question the operator did not ask.
  void mouseReleaseEvent(QGraphicsSceneMouseEvent* event) override;

private:
  bool point_ = false;
  bool polygon_ = false;
  // Line / polygon outline in item-local coordinates (the item is positioned at
  // the geometry's first vertex, so the path's numbers stay small instead of
  // being millions of Web-Mercator metres).
  QPainterPath path_;
  QColor color_;
  double radius_ = 0.0;
  QMap<QString, QVariant> attributes_;

  /// True when the view the event came from is in pan mode. @p widget is the
  /// event's `widget()` — the viewport of the view that delivered it, whose
  /// parent is the QGraphicsView. Reading the EVENT's view matters once a scene
  /// has more than one (a second map window, a print preview): the mode of some
  /// other view says nothing about the one the operator clicked in. Falls back to
  /// any attached view when the event carries none, and is false for a scene with
  /// no view at all, which keeps the popup out of tests.
  bool viewInPanMode(const QWidget* widget) const;
};

}  // namespace camp::vector

#endif  // CAMP_VECTOR_FEATURE_ITEM_H
