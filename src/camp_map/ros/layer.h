#ifndef CAMP_ROS_LAYER_H
#define CAMP_ROS_LAYER_H

#include "../map/layer.h"
#include "transform_cache.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <mutex>
#include <string>

namespace camp
{

namespace ros
{

class Node;

// Base class for ROS map layers.
class Layer: public map::Layer
{
public:
  Layer(MapItem* parent, Node* node, const QString& object_name);

  enum { Type = map::RosLayerType };

  int type() const override
  {
    // Enable the use of qgraphicsitem_cast with this item.
    return Type;
  }

protected:
  QPointF transformToWebMercator(const geometry_msgs::msg::Pose &pose, const std_msgs::msg::Header &header);

  QPointF frameOriginInWebMercator(const std_msgs::msg::Header &header);


protected:
  Node* node_ = nullptr;

private:
  // Resolve earth<-frame_id, tolerating a transient TF gap. On a successful
  // lookup the transform is cached per frame_id; on a lookup miss the last
  // good transform is reused while it is younger than the staleness budget,
  // so a momentary gap holds the last placement instead of dropping the frame.
  // Throws tf2::TransformException only on cold start (frame never resolved)
  // or once the gap exceeds the budget, leaving callers to drop as before.
  geometry_msgs::msg::TransformStamped lookupEarthTransform(const std::string& frame_id);

  // Guards transform_cache_: transformToWebMercator runs on QtConcurrent
  // worker threads (grids) and on ROS callback threads (markers).
  std::mutex transform_cache_mutex_;
  TransformCache transform_cache_;

};

} // namespace ros
} // namespace camp

#endif
