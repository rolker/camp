#ifndef CAMP_ROS_NODE_H
#define CAMP_ROS_NODE_H

#include <QObject>

#include "ros_common.h"

namespace camp_ros
{

class NodeManager;

class NodeThread: public QObject
{
  Q_OBJECT
public:
  NodeThread();

public slots:
  // Starts the ROS node.
  void start();

signals:
  void started(rclcpp::Node::SharedPtr node, tf2_ros::Buffer::SharedPtr);
  void shuttingDown();

private:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::TransformListener> transform_listener_;
};

} // namespace camp_ros

#endif
