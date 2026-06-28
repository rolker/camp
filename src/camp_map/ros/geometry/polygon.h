#ifndef CAMP_ROS_GEOMETRY_POLYGON_H
#define CAMP_ROS_GEOMETRY_POLYGON_H

#include "../layer.h"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include <QtConcurrent>

namespace camp
{
namespace ros
{
namespace geometry
{

struct PolygonData
{
  QPolygonF polygon;
  QPointF position;
};

class Polygon: public Layer
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  Polygon(MapItem* parent, Node* node, QString topic);

signals:
  void newPolygonData(PolygonData data);

private slots:
  void polygonCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr data);
  void processPolygon(const geometry_msgs::msg::PolygonStamped::SharedPtr data);
  void updatePolygon(const PolygonData &data);

private:
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr subscription_;

  QFuture<void> process_future_;

};

} // namespace geometry
} // namespace ros
} // namespace camp

#endif
