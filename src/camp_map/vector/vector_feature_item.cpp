#include "vector_feature_item.h"

#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsView>
#include <QPainter>
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
void addRing(QPainterPath& path, const std::vector<QGeoCoordinate>& ring, const QPointF& origin)
{
  bool first = true;
  for(const auto& coordinate : ring)
  {
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

const QGeoCoordinate* firstCoordinate(const ParsedGeometry& geometry)
{
  if(!geometry.exterior.empty())
    return &geometry.exterior.front();
  for(const auto& ring : geometry.interiorRings)
    if(!ring.empty())
      return &ring.front();
  return nullptr;
}

}  // namespace

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
  // A cosmetic pen is one device pixel wide however far the view is zoomed out,
  // so the scene-space allowance cannot be derived here; the path's own bounds
  // plus nothing is correct for hit-testing, and Qt tolerates the one-pixel
  // overdraw at the edges.
  return path_.boundingRect();
}

QPainterPath VectorFeatureItem::shape() const
{
  QPainterPath shape;
  if(point_)
    shape.addEllipse(QPointF(0.0, 0.0), radius_, radius_);
  else if(path_.isEmpty())
    return shape;
  else
    shape = path_;
  return shape;
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

bool VectorFeatureItem::viewInPanMode() const
{
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
  if(event->button() != Qt::LeftButton || !viewInPanMode())
  {
    event->ignore();
    return;
  }
  QToolTip::showText(event->screenPos(), attributeText());
  event->accept();
}

}  // namespace camp::vector
