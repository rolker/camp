#ifndef CAMP_ROS_NODE_H
#define CAMP_ROS_NODE_H

#include "../tools/layer_manager.h"
#include <QThread>
#include "ros_common.h"

namespace camp
{

namespace tools
{
  class ToolsManager;
}

namespace ros
{

class GraphThread;

/// Manages the ROS Node listing topics and services.
/// Also manages the ROS node thread and provides a tf2 buffer.
class Node: public tools::LayerManager
{
  Q_OBJECT
public:
  Node(tools::ToolsManager* tools_manager);
  ~Node();

  enum { Type = map::RosNodeType};

  int type() const override
  {
    // Enable the use of qgraphicsitem_cast with this item.
    return Type;
  }

  // This static method should be called before the QApplication is 
  // created. This calls ros::init which can modify argv and argc which are also
  // passed to QApplication. QApplication docs state that argv and argc should
  // not be changed once the application is started.
  static void init(int& argc, char **argv);

  tf2_ros::Buffer::SharedPtr transformBuffer();
  rclcpp::Node::SharedPtr node();

  GraphThread* graphThread() const;

signals:
  void startNode();

  void shuttingDownRos();

public slots:
  void nodeStarted(rclcpp::Node::SharedPtr node, tf2_ros::Buffer::SharedPtr buffer);
  void nodeShuttingDown();

private:
  QThread node_thread_;
  GraphThread* graph_thread_ = nullptr;

  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer::SharedPtr transform_buffer_;

};

}  // namespace ros

} // namespace camp

#endif
