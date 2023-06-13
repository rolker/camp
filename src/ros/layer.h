#ifndef CAMP_ROS_LAYER_H
#define CAMP_ROS_LAYER_H

#include "../map/layer.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

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
  QPointF transformToWebMercator(const geometry_msgs::Pose &pose, const std_msgs::Header &header);


protected slots:
  void unsubscribe();

protected:
  NodeManager* node_manager_ = nullptr;

  ros::Subscriber subscriber_;

};

} // namespace map

#endif
