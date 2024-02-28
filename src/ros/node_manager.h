#ifndef CAMP_ROS_NODE_MANAGER_H
#define CAMP_ROS_NODE_MANAGER_H

#include "../tools/layer_manager.h"
#include <QThread>
#include "ros_common.h"

namespace tools
{
  class ToolsManager;
}

namespace camp_ros
{

// Monitors status of the ROS core and starts/stop the ROS Node.
class NodeManager: public tools::LayerManager
{
  Q_OBJECT
public:
  NodeManager(tools::ToolsManager* tools_manager);
  ~NodeManager();

  enum { Type = map::NodeManagerType};

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

public slots:
  void nodeStarted(rclcpp::Node::SharedPtr node, tf2_ros::Buffer::SharedPtr buffer);
  void nodeShuttingDown();

signals:
  void startNode();

  // Signal emitted with the current list of available topics.
  // Topics are listed in a map of topic name as keys and type as values.
  void topicsAvailable(QMap<QString, QString> topics);

  void shuttingDownRos();

private:
  QThread node_thread_;

  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer::SharedPtr transform_buffer_;

};

} // namespace camp_ros

#endif
