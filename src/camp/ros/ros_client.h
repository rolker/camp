#ifndef CAMP_ROS_CLIENT_H
#define CAMP_ROS_CLIENT_H

#include "ros_common.h"

namespace camp_ros
{

template<typename T>
class ROSClient: public T
{
public:
  ROSClient(T *parent = nullptr)
    :T(parent)
  {
    
  }

  virtual void onNodeUpdated()
  {

  }
public slots:
  void nodeStarted(rclcpp::Node::SharedPtr node, tf2_ros::Buffer::SharedPtr buffer)
  {
    node_ = node;
    transform_buffer_ = buffer;
    // [#59] On shutdown ROSLink re-emits rosConnected with a null node
    // (roslink.cpp nodeShuttingDown) to signal teardown. Subclass
    // onNodeUpdated() overrides create subscriptions off node_, so running it
    // with a null node dereferences a null rclcpp::Node — the segfault seen on
    // SIGINT exit. Skip the update when the node is gone.
    if(node_)
      this->onNodeUpdated();
  }

protected:
  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer::SharedPtr transform_buffer_;

};

}

#endif