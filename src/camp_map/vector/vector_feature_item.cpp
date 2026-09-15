#include "vector_feature_item.h"

#include <QBrush>
#include <QFont>
#include <QGraphicsSceneHoverEvent>
#include <QGraphicsSimpleTextItem>
#include <QPainter>
#include <QPainterPathStroker>
#include <QPen>
#include <QStringList>

#include <algorithm>
#include <cmath>

#include "../map_view/web_mercator.h"
#include "vector_parse.h"
#include "vector_style.h"

namespace camp::vector
{

namespace
{

// Append a ring's vertices to `path`, relative to `origin` (scene metres).
//
// [camp#22] Unplaceable vertices are DROPPED rather than projected: one NaN or
// out-of-range vertex would otherwise stretch the path — and with it the layer's
// extent and the scene index — across the whole world. Dropping a vertex from a
// ring is a visible shortcut in the drawn geometry, which is the honest outcome
// for a file whose coordinates CAMP cannot place; the layer reports the count.
void addRing(QPainterPath& path, const std::vector<QGeoCoordinate>& ring, const QPointF& origin)
{
  bool first = true;
  for(const auto& coordinate : ring)
  {
    if(!isPlaceable(coordinate))
      continue;
    const QPointF point = placeableToMap(coordinate) - origin;
    if(first)
    {
      path.moveTo(point);
      first = false;
    }
    else
      path.lineTo(point);
  }
}

// [camp#22] Hover tolerance for a line feature, in scene metres — see shape().
constexpr double kHoverWidth = 5.0;

// [camp#22] Hover tolerance ADDED AROUND a point marker, in device pixels — see
// shape(). The drawn marker is kDefaultPointRadius (5 px) at the default size,
// and in the operator GUI test of 2026-09-15 nobody could land the cursor inside
// it: under the open-hand pan cursor the hotspot is not visible, so a 5 px target
// is aimed at blind. The slack is a HIT target only; nothing drawn grows. (It was
// added for click-to-inspect and serves hover-to-inspect for the same reason —
// what the popup answers to is where the cursor can be put, not what the
// renderer draws.)
constexpr double kPointHoverSlackPixels = 4.0;

// [camp#22] Gap in device pixels between a point marker and its hover label, so
// the text does not sit on top of the symbol it describes.
constexpr double kLabelGapPixels = 4.0;

// The first vertex CAMP can place, which is what the item is positioned at.
const QGeoCoordinate* firstCoordinate(const ParsedGeometry& geometry)
{
  for(const auto& coordinate : geometry.exterior)
    if(isPlaceable(coordinate))
      return &coordinate;
  for(const auto& ring : geometry.interiorRings)
    for(const auto& coordinate : ring)
      if(isPlaceable(coordinate))
        return &coordinate;
  return nullptr;
}

}  // namespace

bool isPlaceable(const QGeoCoordinate& coordinate)
{
  // QGeoCoordinate::isValid() is exactly the contract documented in the header:
  // both ordinates set and finite, latitude in [-90, 90], longitude in
  // [-180, 180]. A default-constructed (unset) coordinate is invalid too.
  return coordinate.isValid();
}

QPointF placeableToMap(const QGeoCoordinate& coordinate)
{
  // maximum_latitude is in RADIANS; QGeoCoordinate carries degrees.
  constexpr double kMaximumLatitudeDegrees = web_mercator::maximum_latitude * 180.0 / M_PI;
  const double latitude =
      std::max(-kMaximumLatitudeDegrees, std::min(kMaximumLatitudeDegrees, coordinate.latitude()));
  return web_mercator::geoToMap(QGeoCoordinate(latitude, coordinate.longitude()));
}

bool hasPlaceableCoordinate(const ParsedGeometry& geometry)
{
  return firstCoordinate(geometry) != nullptr;
}

VectorFeatureItem::VectorFeatureItem(QGraphicsItem* parent, const ParsedGeometry& geometry):
  QGraphicsItem(parent),
  point_(geometry.type == ParsedGeometry::Point),
  polygon_(geometry.type == ParsedGeometry::Polygon),
  color_(Qt::darkCyan),
  radius_(kDefaultPointRadius),
  attributes_(geometry.attributes)
{
  // [camp#22 / ADR-0016 D5] Inspection is on HOVER, so this item answers NO mouse
  // button at all: every press over a feature falls through to QGraphicsView,
  // which is what makes a pan gesture that starts on a feature pan (camp#225,
  // fixed by construction) and what keeps ProjectView's add-* placement clicks
  // out of this item's hands. QGraphicsItem accepts the left button by default,
  // so this has to be said.
  setAcceptedMouseButtons(Qt::NoButton);
  // [camp#22 / ADR-0016 D5] The popup is an IN-SCENE LABEL, shown on hover-enter
  // and cleared on hover-leave — the mechanism GeoGraphicsItem gives Platform and
  // AISContact, not a Qt tooltip. A tooltip waits out Qt's delay before it
  // appears; every other CAMP item answers the cursor instantly, and in the
  // operator GUI test of 2026-09-15 that difference was the whole of the
  // complaint. Hover events have to be accepted for that, which a tooltip did not
  // need: the per-item hover tracking is the price of matching the application.
  //
  // The label item itself is NOT created here — see labelItem(). At up to
  // kMaxFeatureItems (50 000) features a text child per feature would be 50 000
  // scene items created at load for the handful the operator ever hovers.
  setAcceptHoverEvents(true);

  const QGeoCoordinate* anchor = firstCoordinate(geometry);
  if(anchor)
  {
    // [ADR-0002] Transform to Web-Mercator scene metres ONCE, here, at load —
    // never per paint. The parent layer is untransformed at the scene origin, so
    // parent-local coordinates are scene coordinates.
    // placeableToMap, not geoToMap: the anchor is a vertex like any other, and a
    // polar one must be clamped here too or the item's POSITION is what blows the
    // extent out, with the path's own numbers staying small and innocent.
    const QPointF origin = placeableToMap(*anchor);
    setPos(origin);

    if(point_)
      // Screen-sized marker: constant pixels across zoom, like the mission items'
      // symbols. The item's own coordinates become device pixels around setPos().
      setFlag(QGraphicsItem::ItemIgnoresTransformations);
    else
    {
      addRing(path_, geometry.exterior, origin);
      if(polygon_)
      {
        path_.closeSubpath();
        for(const auto& ring : geometry.interiorRings)
        {
          addRing(path_, ring, origin);
          path_.closeSubpath();
        }
        // Odd-even so an interior ring paints as a HOLE rather than as more fill.
        path_.setFillRule(Qt::OddEvenFill);
      }
    }
  }
  // The hit shape is built ONCE here, not on demand — see rebuildShape().
  rebuildShape();
}

QRectF VectorFeatureItem::boundingRect() const
{
  if(point_)
  {
    // Device pixels (ItemIgnoresTransformations). The HIT shape is the marker
    // grown by kPointHoverSlackPixels (see shape()), and a shape outside
    // boundingRect() is undefined behaviour in Qt — so the slack is included
    // here too, plus one pixel of allowance for the width-2 no-data outline.
    const double r = radius_ + kPointHoverSlackPixels + 1.0;
    return QRectF(-r, -r, 2.0 * r, 2.0 * r);
  }
  if(path_.isEmpty())
    return QRectF();
  // A cosmetic pen is one device pixel wide however far the view is zoomed out,
  // so the pen's scene-space allowance cannot be derived here and the path's own
  // bounds are what the paint needs; Qt tolerates the one-pixel overdraw at the
  // edges. The HIT shape, however, is wider than the path for a line (see
  // shape()), and a shape outside boundingRect() is undefined behaviour in Qt —
  // so the line case is grown by the stroker's half-width.
  const QRectF bounds = path_.boundingRect();
  if(polygon_)
    return bounds;
  return bounds.adjusted(-kHoverWidth / 2.0, -kHoverWidth / 2.0,
                         kHoverWidth / 2.0, kHoverWidth / 2.0);
}

QPainterPath VectorFeatureItem::shape() const
{
  // [camp#22] The CACHED shape — never built here. shape() is called by the
  // scene's hit test, and with inspection on hover that is per MOUSE-MOVE for
  // every item whose bounding rect is under the cursor; a long polyline's
  // bounding rect covers most of the map, so a whole-path re-stroke on each call
  // would be paid continuously while the operator simply moves the mouse. The
  // click version paid it once per click, which is why it went unnoticed.
  return shape_;
}

void VectorFeatureItem::rebuildShape()
{
  shape_ = QPainterPath();
  if(point_)
  {
    // [camp#22] The marker PLUS kPointHoverSlackPixels, for the same reason a
    // line's shape is stroked wider than its path: the target the operator aims
    // at is the one the cursor can actually be placed on, not the one the
    // renderer draws. boundingRect() grows with it. shape() is what
    // the scene hit-tests to dispatch hover events, so this is the hover target.
    const double click_radius = radius_ + kPointHoverSlackPixels;
    shape_.addEllipse(QPointF(0.0, 0.0), click_radius, click_radius);
    return;
  }
  if(path_.isEmpty())
    return;
  if(polygon_)
  {
    shape_ = path_;   // a closed, filled path: Qt's fill-area hit test works on it
    return;
  }
  // [camp#22] A LINE has no fill area, so returning the raw open path means Qt's
  // hit test never picks it and hover-to-inspect is unusable on every line
  // feature. Stroke it into a thin ribbon, as the mission-tree LineString does
  // (src/camp/vector/linestring.cpp:66-84). The width is in ITEM coordinates
  // (scene metres for a line item), so it is deliberately generous: kHoverWidth
  // metres of tolerance is a few pixels at survey zoom levels and still a small
  // target when zoomed far out, which is the same trade the mission item makes.
  QPainterPathStroker stroker;
  stroker.setWidth(kHoverWidth);
  shape_ = stroker.createStroke(path_);
}

void VectorFeatureItem::paint(QPainter* painter, const QStyleOptionGraphicsItem*, QWidget*)
{
  painter->save();
  painter->setRenderHint(QPainter::Antialiasing, true);
  // [camp#22] No data is carried in the OUTLINE and the FILL PATTERN, not only in
  // the colour: noDataColor()'s grey and the middle of the grayscale palette are
  // the same grey, so a colour-only answer would make "no value here" look like a
  // mid-range measurement under a palette the operator can select. A dash pattern
  // and a hatch are channels no palette touches.
  if(point_)
  {
    // A point's marker is its fill, so no-data is drawn HOLLOW: an unfilled ring
    // in the same grey, which no styled marker can look like.
    painter->setBrush(no_data_ ? QBrush(Qt::NoBrush) : QBrush(color_));
    QPen pen(no_data_ ? color_ : QColor(Qt::black));
    if(no_data_)
    {
      // [camp#22] A VISIBLE ring, drawn at the same weight as a line's or a
      // polygon's outline. The hollow marker used to be stroked with the same
      // width-0 hairline the filled marker gets for contrast — but the filled
      // marker has a disc of colour behind that hairline and the hollow one has
      // nothing, so one dashed device pixel of mid grey over a chart background
      // is effectively not there. In the operator GUI test of 2026-09-15,
      // colouring by a free-text field (every feature no-data) read as the
      // features DISAPPEARING. Hollow and dashed stay: they are the second
      // channel no palette can imitate (ADR-0016 D6); only the weight changes.
      pen.setCosmetic(true);
      pen.setWidth(2);
      pen.setStyle(Qt::DashLine);
    }
    else
      pen.setWidth(0);         // cosmetic: a hairline outline for contrast
    painter->setPen(pen);
    painter->drawEllipse(QPointF(0.0, 0.0), radius_, radius_);
  }
  else
  {
    QPen pen(color_);
    pen.setCosmetic(true);     // one device pixel at every zoom
    pen.setWidth(2);
    if(no_data_)
      pen.setStyle(Qt::DashLine);
    painter->setPen(pen);
    if(polygon_)
    {
      QColor fill = color_;
      fill.setAlpha(80);       // a filled polygon must not hide what is under it
      // Hatched rather than solid: the same distinction the outline makes, in the
      // channel a filled polygon is mostly read by.
      painter->setBrush(no_data_ ? QBrush(fill, Qt::BDiagPattern) : QBrush(fill));
    }
    else
      painter->setBrush(Qt::NoBrush);
    painter->drawPath(path_);
  }
  painter->restore();
}

void VectorFeatureItem::setColor(const QColor& color)
{
  if(color == color_)
    return;
  color_ = color;
  update();
}

void VectorFeatureItem::setNoData(bool no_data)
{
  if(no_data == no_data_)
    return;
  no_data_ = no_data;
  update();
}

void VectorFeatureItem::setRadius(double radius)
{
  if(!point_ || radius == radius_)
    return;
  prepareGeometryChange();     // the radius IS the bounding rect for a point
  radius_ = radius;
  // The radius is also the hit shape, which is cached: every
  // prepareGeometryChange() site has to refresh it or a restyled marker keeps the
  // old hover target until something else rebuilds it.
  rebuildShape();
  update();
}

void VectorFeatureItem::hoverEnterEvent(QGraphicsSceneHoverEvent* event)
{
  // [camp#22 / ADR-0016 D5] Exactly what Platform::hoverEnterEvent() and
  // AISContact::hoverEnterEvent() do (platform.cpp:170, ais_contact.cpp:272):
  // set the label's text. setShowLabelFlag() is spelled out here because
  // camp_map cannot depend on the camp app layer that GeoGraphicsItem lives in.
  QGraphicsSimpleTextItem* label = labelItem();
  label->setText(attributeText());
  if(point_)
  {
    // A point ignores the view transform, so its own coordinates are already
    // device pixels around setPos(): put the label just beside the marker, clear
    // of the hit slack. The existing labels are placed relative to their anchor
    // the same way (GeoGraphicsItem::setLabelPosition()).
    label->setPos(radius_ + kPointHoverSlackPixels + kLabelGapPixels, 0.0);
  }
  else
  {
    // A line or polygon has no single anchor worth labelling — it may cross the
    // whole view — so the label goes where the cursor is. event->pos() is in
    // item coordinates, which is what setPos() on a child wants.
    label->setPos(event->pos());
  }
  QGraphicsItem::hoverEnterEvent(event);
}

void VectorFeatureItem::hoverLeaveEvent(QGraphicsSceneHoverEvent* event)
{
  // Emptying the text is how GeoGraphicsItem hides its label too
  // (setShowLabelFlag(false) -> setText("")). Because every item clears its own
  // on leave, only one label is ever on screen.
  if(label_)
    label_->setText(QString());
  QGraphicsItem::hoverLeaveEvent(event);
}

QGraphicsSimpleTextItem* VectorFeatureItem::labelItem()
{
  if(label_)
    return label_;
  label_ = new QGraphicsSimpleTextItem(this);
  // [camp#22] Settings COPIED from GeoGraphicsItem's constructor
  // (src/camp/geographicsitem.cpp:16-25), which is what the vessel and AIS
  // labels use: screen-sized regardless of zoom, black text outlined in white so
  // it stays readable over a chart. Copied rather than shared because
  // GeoGraphicsItem is in the camp executable and this item is in camp_map,
  // which must not depend on it.
  label_->setFlag(QGraphicsItem::ItemIgnoresTransformations);
  QFont font = label_->font();
  font.setPointSize(20);
  font.setBold(true);
  label_->setFont(font);
  label_->setBrush(QBrush(QColor("black")));
  QPen outline(QColor("white"));
  outline.setWidth(0);
  label_->setPen(outline);
  return label_;
}

QString VectorFeatureItem::attributeText() const
{
  if(attributes_.isEmpty())
    return QStringLiteral("(no attributes)");
  QStringList lines;
  for(auto it = attributes_.begin(); it != attributes_.end(); ++it)
    lines << it.key() + ": " + it.value().toString();
  return lines.join('\n');
}

}  // namespace camp::vector
