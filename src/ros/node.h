#ifndef CAMP_ROS_NODE_H
#define CAMP_ROS_NODE_H

#include <QObject>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"

namespace camp_ros
{

class NodeManager;

class Node: public QObject, public rclcpp::Node
{
  Q_OBJECT
public:
  Node();



public slots:
  // Starts the ROS node by creating a node handle.
  void start(NodeManager* node_manager);

signals:
  void started();
  void shuttingDown();

private:
  std::unique_ptr<tf2_ros::TransformListener> transform_listener_;
};

} // namespace camp_ros

#endif
