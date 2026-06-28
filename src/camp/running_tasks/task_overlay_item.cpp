#include "running_tasks/task_overlay_item.h"

#include <QPainter>
#include <QPen>
#include <QBrush>
#include <QColor>
#include <QMarginsF>

TaskOverlayItem::TaskOverlayItem(const QString& id,
                                 const QList<QGeoCoordinate>& geo_poses,
                                 bool is_current, bool is_done,
                                 QGraphicsItem* parent)
  : QObject(nullptr), GeoGraphicsItem(parent),
    id_(id), geo_poses_(geo_poses),
    is_current_(is_current), is_done_(is_done)
{
  setAcceptHoverEvents(false);
}

QPainterPath TaskOverlayItem::buildPath() const
{
  QPainterPath path;
  if (geo_poses_.isEmpty())
    return path;

  if (geo_poses_.size() == 1)
  {
    // Single pose: filled circle at scene position.
    const QPointF center = geoToPixel(geo_poses_.first());
    path.addEllipse(center, 6.0, 6.0);
  }
  else
  {
    // Multi-pose: polyline through all scene positions.
    path.moveTo(geoToPixel(geo_poses_.first()));
    for (int i = 1; i < geo_poses_.size(); ++i)
      path.lineTo(geoToPixel(geo_poses_[i]));
  }
  return path;
}

QRectF TaskOverlayItem::boundingRect() const
{
  return buildPath().boundingRect().marginsAdded(QMarginsF(6, 6, 6, 6));
}

QPainterPath TaskOverlayItem::shape() const
{
  return buildPath();
}

void TaskOverlayItem::paint(QPainter* painter,
                            const QStyleOptionGraphicsItem* /*option*/,
                            QWidget* /*widget*/)
{
  if (geo_poses_.isEmpty())
    return;

  painter->save();

  QColor color;
  if (is_done_)
    color = QColor(0x80, 0x80, 0x80, 180);      // gray — completed
  else if (is_selected_)
    color = QColor(0xFF, 0xA5, 0x00, 230);       // orange — selected
  else if (is_current_)
    color = QColor(0x00, 0xCC, 0x00, 220);       // green — active
  else
    color = QColor(0x40, 0x80, 0xFF, 150);       // blue-muted — pending

  QPen pen(color);
  pen.setCosmetic(true);
  pen.setWidth(is_selected_ ? 3 : 2);
  painter->setPen(pen);

  if (geo_poses_.size() == 1)
  {
    painter->setBrush(QBrush(color));
    painter->drawPath(buildPath());
  }
  else
  {
    painter->setBrush(Qt::NoBrush);
    painter->drawPath(buildPath());
    // Draw small endpoint markers.
    painter->setBrush(QBrush(color));
    for (const QGeoCoordinate& gc : geo_poses_)
    {
      if (gc.isValid())
        painter->drawEllipse(geoToPixel(gc), 3.0, 3.0);
    }
  }

  painter->restore();
}

void TaskOverlayItem::setSelected(bool selected)
{
  is_selected_ = selected;
  update();
}

void TaskOverlayItem::mousePressEvent(QGraphicsSceneMouseEvent* /*event*/)
{
  emit clicked(id_);
}
