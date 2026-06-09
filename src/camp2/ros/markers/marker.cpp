#include "marker.h"
#include <QTimer>
#include <QPainter>
#include <QGraphicsScene>
#include "../../map_view/web_mercator.h"
#include "../node.h"

#include <QDebug>

namespace camp
{
namespace ros
{
namespace markers
{

Marker::Marker(MapItem* parent, Node* node, uint32_t id):
  Layer(parent, node, QString::number(id)), id_(id)
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
  if(data.marker.action == visualization_msgs::msg::Marker::ADD)
  {
    setWebMercatorPositionAndScale(data.position);
    setRotation(data.rotation*180.0/M_PI);
    QPen p;
    p.setColor(QColor::fromRgbF(data.marker.color.r, data.marker.color.g, data.marker.color.b, data.marker.color.a));
    p.setCosmetic(true);
    QBrush b;
    b.setColor(QColor::fromRgbF(data.marker.color.r, data.marker.color.g, data.marker.color.b, data.marker.color.a));
    b.setStyle(Qt::BrushStyle::SolidPattern);

    switch(data.marker.type)
    {
      case visualization_msgs::msg::Marker::CUBE:
      {
        auto cube = new QGraphicsRectItem(0, 0, data.marker.scale.x, data.marker.scale.y, this);
        cube->setPen(p);
        cube->setBrush(b);
        break;
      }
      case visualization_msgs::msg::Marker::SPHERE:
      {
        auto sphere = new QGraphicsEllipseItem(-data.marker.scale.x/2.0, -data.marker.scale.y/2.0, data.marker.scale.x, data.marker.scale.y, this);
        sphere->setPen(p);
        sphere->setBrush(b);
        break;
      }
      case visualization_msgs::msg::Marker::LINE_STRIP:
      case visualization_msgs::msg::Marker::LINE_LIST:
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
          if(data.marker.type == visualization_msgs::msg::Marker::LINE_LIST)
          {
            p1++;
            if(p1 == data.marker.points.end())
              break;
          }
        }
        break;
      }
      case visualization_msgs::msg::Marker::TEXT_VIEW_FACING:
      {
        auto text = new QGraphicsSimpleTextItem(data.marker.text.c_str(), this);
        // The map view flips the Y axis so north renders up (ProjectView applies
        // scale(1.0, -1.0); Web Mercator is Y-up, QGraphicsView is Y-down). A plain
        // child item inherits that flip and renders upside-down. ItemIgnoresTransformations
        // keeps the text screen-aligned, upright, and constant-size — the correct behavior
        // for TEXT_VIEW_FACING (a billboard in rviz) and the same convention camp's other
        // labels use (see geographicsitem.cpp, vector/point.cpp).
        text->setFlag(QGraphicsItem::ItemIgnoresTransformations);
        text->setBrush(b);
        break;
      }
      default:
        RCLCPP_WARN_STREAM(node_->node()->get_logger(), "marker type not handled: " << data_.marker.type);
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
  auto now = node_->node()->get_clock()->now();
  bool expired = rclcpp::Time(data_.marker.header.stamp).nanoseconds() != 0 && rclcpp::Duration(data_.marker.lifetime).nanoseconds() != 0 && rclcpp::Time(data_.marker.header.stamp) + rclcpp::Duration(data_.marker.lifetime) < now;
  expired |= data_.marker.action != 0; // consider deleted as expired
  if(expired)
    for(auto child: childItems())
    {
      if(child->scene())
        child->scene()->removeItem(child);
      delete child;
    }
}

}  // namespace markers
}  // namespace ros
}  // namespace camp
