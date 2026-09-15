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

class QGraphicsSceneHoverEvent;
class QGraphicsSimpleTextItem;

namespace camp::vector
{

struct ParsedGeometry;

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
///
/// [camp#22 / ADR-0016 D5] INSPECTION IS ON HOVER, and the item answers NO mouse
/// button at all (`setAcceptedMouseButtons(Qt::NoButton)`). Hovering is CAMP's
/// house convention for "tell me what this is" — `Platform` and `AISContact` show
/// their label on hover, `GeoGraphicsMissionItem` brightens on hover, and nothing
/// in CAMP inspects on click. It is also the only answer that leaves the view's
/// own gestures intact: a press over a feature falls straight through to
/// `QGraphicsView`'s ScrollHandDrag, so a pan that starts on a feature pans
/// (camp#225, fixed by construction) and a press in one of ProjectView's add-*
/// modes places its mission item with nothing in the way.
///
/// The popup is an IN-SCENE LABEL — a child `QGraphicsSimpleTextItem` filled in
/// by `hoverEnterEvent()` and emptied by `hoverLeaveEvent()` — which is the very
/// mechanism `GeoGraphicsItem` gives `Platform` and `AISContact`, copied because
/// `camp_map` cannot depend on the `camp` app layer where that base class lives.
/// It appears INSTANTLY, with no tooltip delay: in the operator GUI test of
/// 2026-09-15 a delayed Qt tooltip read as "similar to what was existing, but not
/// the same", because every other CAMP item answers the cursor at once.
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

  /// [camp#22] Draw this feature as NO DATA — its styling field is missing, or
  /// its value is not a finite number (`camp::vector::isNoData()`).
  ///
  /// This is a SECOND channel, not a repeat of the colour. `noDataColor()`'s grey
  /// is indistinguishable from the middle of the grayscale palette, which the
  /// operator can select and which is the fallback for an unknown palette name, so
  /// a colour-only answer lets a missing value read as a mid-range measurement.
  /// A no-data feature is drawn with a DASHED outline and a HATCHED fill (a hollow
  /// marker for a point) — channels no palette touches, and which survive
  /// grayscale, colour-blind vision and a black-and-white printout alike.
  void setNoData(bool no_data);
  bool isNoData() const { return no_data_; }

  const QMap<QString, QVariant>& attributes() const { return attributes_; }
  bool isPoint() const { return point_; }
  bool isPolygon() const { return polygon_; }

  /// The text shown by the hover-to-inspect label: one "name: value" line per
  /// attribute. Attributes never change after construction, so this is stable
  /// for the item's lifetime; the label is filled from it on every hover-enter.
  QString attributeText() const;

protected:
  /// [camp#22 / ADR-0016 D5] Show / hide the attribute label. The same pair
  /// `Platform::hoverEnterEvent()` and `AISContact::hoverEnterEvent()` use, with
  /// `setShowLabelFlag()` spelled out because this item has no `GeoGraphicsItem`
  /// base to inherit it from.
  void hoverEnterEvent(QGraphicsSceneHoverEvent* event) override;
  void hoverLeaveEvent(QGraphicsSceneHoverEvent* event) override;

private:
  /// The hover label, created on FIRST HOVER and kept afterwards — never one per
  /// feature at load. A layer may hold up to `kMaxFeatureItems` (50 000) features
  /// and the operator hovers a handful of them, so creating a text item per
  /// feature up front would cost 50 000 scene items, their index entries and
  /// their font metrics for nothing. Owned by this item as a child.
  QGraphicsSimpleTextItem* label_ = nullptr;

  /// The hover label, created on first call. Carries the same flag, font, brush
  /// and outline pen as `GeoGraphicsItem`'s label.
  QGraphicsSimpleTextItem* labelItem();

  bool point_ = false;
  bool polygon_ = false;
  bool no_data_ = false;
  // Line / polygon outline in item-local coordinates (the item is positioned at
  // the geometry's first vertex, so the path's numbers stay small instead of
  // being millions of Web-Mercator metres).
  QPainterPath path_;
  QColor color_;
  double radius_ = 0.0;
  QMap<QString, QVariant> attributes_;
};

}  // namespace camp::vector

#endif  // CAMP_VECTOR_FEATURE_ITEM_H
