#include "vector_feature_item.h"

#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsView>
#include <QPainter>
#include <QPainterPathStroker>
#include <QStringList>
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
    const QPointF point = web_mercator::geoToMap(coordinate) - origin;
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
  const QPointF origin = web_mercator::geoToMap(*anchor);
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
    // Device pixels (ItemIgnoresTransformations); one pixel of pen allowance.
    const double r = radius_ + 1.0;
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
    shape.addEllipse(QPointF(0.0, 0.0), radius_, radius_);
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
  if(point_)
  {
    painter->setBrush(color_);
    QPen pen(Qt::black);
    pen.setWidth(0);           // cosmetic: a hairline outline for contrast
    painter->setPen(pen);
    painter->drawEllipse(QPointF(0.0, 0.0), radius_, radius_);
  }
  else
  {
    QPen pen(color_);
    pen.setCosmetic(true);     // one device pixel at every zoom
    pen.setWidth(2);
    painter->setPen(pen);
    if(polygon_)
    {
      QColor fill = color_;
      fill.setAlpha(80);       // a filled polygon must not hide what is under it
      painter->setBrush(fill);
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
  if(event->button() != Qt::LeftButton || !viewInPanMode(event->widget()))
  {
    event->ignore();
    return;
  }
  // Remember where the press landed and say nothing yet: whether this is a click
  // or the start of a pan is not known until the button comes back up.
  press_scene_pos_ = event->scenePos();
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
  const QPointF moved = event->scenePos() - press_scene_pos_;
  const QPointF moved_px = event->widget()
    ? QPointF(event->screenPos() - event->buttonDownScreenPos(Qt::LeftButton))
    : moved;
  if(QPointF::dotProduct(moved_px, moved_px) > kClickSlopPixels * kClickSlopPixels)
  {
    event->accept();
    return;
  }
  QToolTip::showText(event->screenPos(), attributeText());
  event->accept();
}

}  // namespace camp::vector
