#ifndef CAMP_ROS_LAYER_H
#define CAMP_ROS_LAYER_H

#include "../map/layer.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace camp_ros
{

class NodeManager;

// Base class for ROS map layers.
class Layer: public map::Layer
{
public:
  Layer(MapItem* parent, NodeManager* node_manager, const QString& object_name);

  enum { Type = map::RosLayerType };

  int type() const override
  {
    // Enable the use of qgraphicsitem_cast with this item.
    return Type;
  }

protected:
  QPointF transformToWebMercator(const geometry_msgs::msg::Pose &pose, const std_msgs::msg::Header &header);


protected:
  NodeManager* node_manager_ = nullptr;

};

} // namespace map

#endif
