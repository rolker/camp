#include "running_tasks/task_overlay_item.h"

#include <cmath>

#include <QPainter>
#include <QPen>
#include <QBrush>
#include <QColor>
#include <QMarginsF>
#include <QPainterPathStroker>

namespace
{
// Desired on-screen marker/hit footprints in display pixels (held constant
// across zoom via sceneRadius — ADR-0003).
constexpr qreal kSinglePoseRadiusPx = 6.0;
constexpr qreal kEndpointRadiusPx = 3.0;
constexpr qreal kHitStrokePx = 5.0;
}  // namespace

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

qreal TaskOverlayItem::sceneRadius(const QGeoCoordinate& at, qreal pixels) const
{
  if (!at.isValid())
    return pixels;
  // Real metres spanning `pixels` display pixels here, projected back through
  // geoToPixel so the result is the equivalent radius in scene units.
  const qreal metres = pixels * metresPerPixel(at);
  const QPointF center = geoToPixel(at);
  const QPointF edge = geoToPixel(at.atDistanceAndAzimuth(metres, 0.0));
  return std::hypot(center.x() - edge.x(), center.y() - edge.y());
}

QPainterPath TaskOverlayItem::buildPath() const
{
  QPainterPath path;
  if (geo_poses_.isEmpty())
    return path;

  if (geo_poses_.size() == 1)
  {
    // Single pose: filled circle at scene position (constant pixel footprint).
    const QGeoCoordinate& c = geo_poses_.first();
    const qreal r = sceneRadius(c, kSinglePoseRadiusPx);
    path.addEllipse(geoToPixel(c), r, r);
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
  if (geo_poses_.isEmpty())
    return {};
  const qreal m = sceneRadius(geo_poses_.first(), kHitStrokePx + kSinglePoseRadiusPx);
  return buildPath().boundingRect().marginsAdded(QMarginsF(m, m, m, m));
}

QPainterPath TaskOverlayItem::shape() const
{
  QPainterPath path = buildPath();
  if (geo_poses_.size() <= 1)
    return path;  // single-pose ellipse already encloses a clickable area

  // A multi-pose buildPath() is an open polyline: zero area, so Qt's
  // shape().contains() hit-test never matches and the item is unclickable
  // (breaking map->tree selection for survey_line/transit tasks). Stroke it
  // into a clickable band of constant pixel width (ADR-0003).
  QPainterPathStroker stroker;
  stroker.setWidth(2.0 * sceneRadius(geo_poses_.first(), kHitStrokePx));
  return stroker.createStroke(path);
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
  else if (is_highlighted_)
    color = QColor(0xFF, 0xA5, 0x00, 230);       // orange — selected
  else if (is_current_)
    color = QColor(0x00, 0xCC, 0x00, 220);       // green — active
  else
    color = QColor(0x40, 0x80, 0xFF, 150);       // blue-muted — pending

  QPen pen(color);
  pen.setCosmetic(true);
  pen.setWidth(is_highlighted_ ? 3 : 2);
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
    // Draw small endpoint markers (constant pixel footprint — ADR-0003).
    painter->setBrush(QBrush(color));
    for (const QGeoCoordinate& gc : geo_poses_)
    {
      if (gc.isValid())
      {
        const qreal r = sceneRadius(gc, kEndpointRadiusPx);
        painter->drawEllipse(geoToPixel(gc), r, r);
      }
    }
  }

  painter->restore();
}

void TaskOverlayItem::setHighlighted(bool highlighted)
{
  is_highlighted_ = highlighted;
  update();
}

void TaskOverlayItem::mousePressEvent(QGraphicsSceneMouseEvent* /*event*/)
{
  emit clicked(id_);
}
