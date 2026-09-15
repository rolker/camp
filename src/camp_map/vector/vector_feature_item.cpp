#include "vector_feature_item.h"

#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsView>
#include <QBrush>
#include <QPainter>
#include <QPainterPathStroker>
#include <QStringList>

#include <algorithm>
#include <cmath>
#include <QToolTip>

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

// [camp#22] Click tolerance for a line feature, in scene metres — see shape().
constexpr double kClickWidth = 5.0;

// [camp#22] Click tolerance ADDED AROUND a point marker, in device pixels — see
// shape(). The drawn marker is kDefaultPointRadius (5 px) at the default size,
// and in the operator GUI test of 2026-09-15 nobody could land a click inside it:
// under the open-hand pan cursor the hotspot is not visible, so a 5 px target is
// aimed at blind. The slack is a CLICK target only; nothing drawn grows.
constexpr double kPointClickSlackPixels = 4.0;

// [camp#22] How far the cursor may travel between press and release and still
// count as a click rather than a pan, in device pixels.
constexpr double kClickSlopPixels = 4.0;

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
  // Left-press is the only button this item answers to; everything else falls
  // through to the view (context menu, middle-click) untouched.
  setAcceptedMouseButtons(Qt::LeftButton);

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
    // Device pixels (ItemIgnoresTransformations). The CLICK shape is the marker
    // grown by kPointClickSlackPixels (see shape()), and a shape outside
    // boundingRect() is undefined behaviour in Qt — so the slack is included
    // here too, plus one pixel of allowance for the width-2 no-data outline.
    const double r = radius_ + kPointClickSlackPixels + 1.0;
    return QRectF(-r, -r, 2.0 * r, 2.0 * r);
  }
  if(path_.isEmpty())
    return QRectF();
  // A cosmetic pen is one device pixel wide however far the view is zoomed out,
  // so the pen's scene-space allowance cannot be derived here and the path's own
  // bounds are what the paint needs; Qt tolerates the one-pixel overdraw at the
  // edges. The CLICK shape, however, is wider than the path for a line (see
  // shape()), and a shape outside boundingRect() is undefined behaviour in Qt —
  // so the line case is grown by the stroker's half-width.
  const QRectF bounds = path_.boundingRect();
  if(polygon_)
    return bounds;
  return bounds.adjusted(-kClickWidth / 2.0, -kClickWidth / 2.0,
                         kClickWidth / 2.0, kClickWidth / 2.0);
}

QPainterPath VectorFeatureItem::shape() const
{
  QPainterPath shape;
  if(point_)
  {
    // [camp#22] The marker PLUS kPointClickSlackPixels, for the same reason a
    // line's shape is stroked wider than its path: the target the operator aims
    // at is the one the cursor can actually be placed on, not the one the
    // renderer draws. boundingRect() grows with it.
    const double click_radius = radius_ + kPointClickSlackPixels;
    shape.addEllipse(QPointF(0.0, 0.0), click_radius, click_radius);
    return shape;
  }
  if(path_.isEmpty())
    return shape;
  if(polygon_)
    return path_;   // a closed, filled path: Qt's fill-area hit test works on it
  // [camp#22] A LINE has no fill area, so returning the raw open path means Qt's
  // hit test never picks it and click-to-inspect is unusable on every line
  // feature. Stroke it into a thin ribbon, as the mission-tree LineString does
  // (src/camp/vector/linestring.cpp:66-84). The width is in ITEM coordinates
  // (scene metres for a line item), so it is deliberately generous: kClickWidth
  // metres of tolerance is a few pixels at survey zoom levels and still a small
  // target when zoomed far out, which is the same trade the mission item makes.
  QPainterPathStroker stroker;
  stroker.setWidth(kClickWidth);
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

bool VectorFeatureItem::viewInPanMode(const QWidget* widget) const
{
  // The event's own view: its widget() is the viewport, whose parent is the view.
  if(widget)
    if(const auto* view = qobject_cast<const QGraphicsView*>(widget->parentWidget()))
      return view->dragMode() == QGraphicsView::ScrollHandDrag;
  // A synthesized event carries no widget; fall back to the attached views.
  if(!scene())
    return false;
  for(const QGraphicsView* view : scene()->views())
    if(view->dragMode() == QGraphicsView::ScrollHandDrag)
      return true;
  return false;
}

void VectorFeatureItem::mousePressEvent(QGraphicsSceneMouseEvent* event)
{
  // In an add-* mode the operator is placing a mission item; ignore the press so
  // it is not marked accepted at the item level and no popup fires mid-placement.
  // ProjectView reads the event position itself and forwards to
  // QGraphicsView::mousePressEvent regardless, so its placement logic is
  // unaffected either way.
  //
  // This gating only holds because ProjectView DEFERS its switch back to pan mode
  // until after that forward — see the panModeAfterDispatch comment in
  // ProjectView::mousePressEvent. Switching inline made the view read as pan
  // during the very press that placed the item, which is what this branch exists
  // to keep out.
  //
  // [camp#225] KNOWN COST of accepting the press in pan mode: the press no longer
  // reaches QGraphicsView's ScrollHandDrag, so a pan that starts ON a feature does
  // not pan the map. The item cannot both accept the press (which is what lets the
  // release tell a click from a drag) and leave the view's gesture intact; the fix
  // belongs in ProjectView, which knows about both. Shipped deliberately as-is.
  if(event->button() != Qt::LeftButton || !viewInPanMode(event->widget()))
  {
    event->ignore();
    return;
  }
  // Say nothing yet: whether this is a click or the start of a pan is not known
  // until the button comes back up. Where the press landed does not need
  // remembering — the release event carries its own button-down screen position.
  event->accept();
}

void VectorFeatureItem::mouseReleaseEvent(QGraphicsSceneMouseEvent* event)
{
  if(event->button() != Qt::LeftButton)
  {
    event->ignore();
    return;
  }
  // A release far from the press is a drag — the operator was panning across the
  // map and happened to start on a feature. Answering that with an attribute
  // tooltip is an answer to a question they did not ask.
  //
  // Measured in SCREEN PIXELS, which is the unit kClickSlopPixels is in and the
  // unit a hand's worth of wobble is constant in: a scene-coordinate slop would
  // mean a different tolerance at every zoom level. The screen positions are
  // carried by the event itself and need no widget, so there is no second,
  // untestable comparison path (this used to fall back to a SCENE-metre delta
  // whenever the event had no widget, comparing metres against a pixel
  // threshold).
  const QPointF moved_px(event->screenPos() - event->buttonDownScreenPos(Qt::LeftButton));
  if(QPointF::dotProduct(moved_px, moved_px) > kClickSlopPixels * kClickSlopPixels)
  {
    event->accept();
    return;
  }
  QToolTip::showText(event->screenPos(), attributeText());
  event->accept();
}

}  // namespace camp::vector
