#ifndef CAMP_VECTOR_FEATURE_ITEM_H
#define CAMP_VECTOR_FEATURE_ITEM_H

#include <QColor>
#include <QGraphicsItem>
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
  /// Pan mode is identified by the view's drag mode (ScrollHandDrag, which
  /// ProjectView::setPanMode sets and every add-* mode clears to NoDrag) rather
  /// than by asking ProjectView: this item lives in camp_map, which cannot call
  /// into the executable that links it (#217).
  void mousePressEvent(QGraphicsSceneMouseEvent* event) override;

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

  /// True when the view under the cursor is in pan mode. False when there is no
  /// view (a headless scene), which keeps the popup out of tests.
  bool viewInPanMode() const;
};

}  // namespace camp::vector

#endif  // CAMP_VECTOR_FEATURE_ITEM_H
