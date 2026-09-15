#include "vector_feature_item.h"

#include <QBrush>
#include <QPainter>
#include <QPainterPathStroker>
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
// what the tooltip answers to is where the cursor can be put, not what the
// renderer draws.)
constexpr double kPointHoverSlackPixels = 4.0;

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
  // The popup is the ordinary Qt tooltip: QGraphicsScene::helpEvent() finds the
  // top item under the cursor whose toolTip() is non-empty and shows it, after
  // the usual hover delay, hiding it when the cursor leaves. No event handler of
  // ours is involved, which is why setAcceptHoverEvents() is deliberately NOT set
  // — hover events are not what drives a tooltip, and turning per-item hover
  // tracking on across a layer of up to kMaxFeatureItems items would cost
  // something for nothing. Attributes are copied once here and never change, so
  // the text is set once; a mutable attribute would have to refresh it.
  setToolTip(attributeText());

  const QGeoCoordinate* anchor = firstCoordinate(geometry);
  if(!anchor)
    return;
  // [ADR-0002] Transform to Web-Mercator scene metres ONCE, here, at load —
  // never per paint. The parent layer is untransformed at the scene origin, so
  // parent-local coordinates are scene coordinates.
  // placeableToMap, not geoToMap: the anchor is a vertex like any other, and a
  // polar one must be clamped here too or the item's POSITION is what blows the
  // extent out, with the path's own numbers staying small and innocent.
  const QPointF origin = placeableToMap(*anchor);
  setPos(origin);

  if(point_)
  {
    // Screen-sized marker: constant pixels across zoom, like the mission items'
    // symbols. The item's own coordinates become device pixels around setPos().
    setFlag(QGraphicsItem::ItemIgnoresTransformations);
    return;
  }

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
  QPainterPath shape;
  if(point_)
  {
    // [camp#22] The marker PLUS kPointHoverSlackPixels, for the same reason a
    // line's shape is stroked wider than its path: the target the operator aims
    // at is the one the cursor can actually be placed on, not the one the
    // renderer draws. boundingRect() grows with it. shape() is what
    // QGraphicsScene::helpEvent() hit-tests, so this is the hover target.
    const double click_radius = radius_ + kPointHoverSlackPixels;
    shape.addEllipse(QPointF(0.0, 0.0), click_radius, click_radius);
    return shape;
  }
  if(path_.isEmpty())
    return shape;
  if(polygon_)
    return path_;   // a closed, filled path: Qt's fill-area hit test works on it
  // [camp#22] A LINE has no fill area, so returning the raw open path means Qt's
  // hit test never picks it and hover-to-inspect is unusable on every line
  // feature. Stroke it into a thin ribbon, as the mission-tree LineString does
  // (src/camp/vector/linestring.cpp:66-84). The width is in ITEM coordinates
  // (scene metres for a line item), so it is deliberately generous: kHoverWidth
  // metres of tolerance is a few pixels at survey zoom levels and still a small
  // target when zoomed far out, which is the same trade the mission item makes.
  QPainterPathStroker stroker;
  stroker.setWidth(kHoverWidth);
  return stroker.createStroke(path_);
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
  update();
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
