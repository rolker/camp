#include "marker.h"
#include <QTimer>
#include <QPainter>
#include <QGraphicsScene>
#include "../../map_view/web_mercator.h"

namespace camp_ros
{

Marker::Marker(MapItem* parent, NodeManager* node_manager, uint32_t id):
  Layer(parent, node_manager, QString::number(id)), id_(id)
{
  QTimer* timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &Marker::checkExpired);
  timer->start(1000);
}

void Marker::updateMarker(const MarkerData& data)
{
  for(auto child: childItems())
  {
    if(child->scene())
      child->scene()->removeItem(child);
    delete child;
  }
  if(data.marker.action == visualization_msgs::Marker::ADD)
  {
    setPos(data.position);
    auto map_distortion = web_mercator::metersPerUnit(data.position);
    double scale = 1.0/map_distortion;
    setTransform(QTransform::fromScale(scale, scale));

    setRotation(data.rotation*180.0/M_PI);
    QPen p;
    p.setColor(QColor::fromRgbF(data.marker.color.r, data.marker.color.g, data.marker.color.b, data.marker.color.a));
    p.setCosmetic(true);
    QBrush b;
    b.setColor(QColor::fromRgbF(data.marker.color.r, data.marker.color.g, data.marker.color.b, data.marker.color.a));
    b.setStyle(Qt::BrushStyle::SolidPattern);

    switch(data.marker.type)
    {
      case visualization_msgs::Marker::CUBE:
      {
        auto cube = new QGraphicsRectItem(0, 0, data.marker.scale.x, data.marker.scale.y, this);
        cube->setPen(p);
        cube->setBrush(b);
        break;
      }
      case visualization_msgs::Marker::SPHERE:
      {
        auto sphere = new QGraphicsEllipseItem(0,0,data.marker.scale.x, data.marker.scale.y, this);
        sphere->setPen(p);
        sphere->setBrush(b);
        break;
      }
      case visualization_msgs::Marker::LINE_STRIP:
      case visualization_msgs::Marker::LINE_LIST:
      {
        for(auto p1 = data.marker.points.begin(); p1 != data.marker.points.end(); ++p1)
        {
          auto p2 = p1;
          p2++;
          if(p2 != data.marker.points.end())
          {
            auto cosmetic_line = new QGraphicsLineItem(p1->x, p1->y, p2->x, p2->y, this);
            p.setWidthF(2.0);
            p.setCosmetic(true);
            cosmetic_line->setPen(p);
            auto line = new QGraphicsLineItem(p1->x, p1->y, p2->x, p2->y, this);
            p.setWidthF(data.marker.scale.x);
            p.setCosmetic(false);
            line->setPen(p);
            // \todo use colors field to add gradients for per vertex colors
          }
          if(data.marker.type == visualization_msgs::Marker::LINE_LIST)
          {
            p1++;
            if(p1 == data.marker.points.end())
              break;
          }
        }
        break;
      }
      case visualization_msgs::Marker::TEXT_VIEW_FACING:
      {
        auto text = new QGraphicsSimpleTextItem(data.marker.text.c_str(), this);
        text->setBrush(b);
        break;
      }
      default:
        ROS_WARN_STREAM("marker type not handles: " << data_.marker.type);
    }
  }
  data_ = data;
}

uint32_t Marker::id() const
{
  return id_;
}

void Marker::checkExpired()
{
  auto now = ros::Time::now();
  bool expired = !data_.marker.header.stamp.isZero() && !data_.marker.lifetime.isZero()&& data_.marker.header.stamp + data_.marker.lifetime < now;
  expired |= data_.marker.action != 0; // consider deleted as expired
  if(expired)
    for(auto child: childItems())
    {
      if(child->scene())
        child->scene()->removeItem(child);
      delete child;
    }
}

} // namespace camp_ros
