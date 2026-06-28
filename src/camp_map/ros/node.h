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
  /// Spawns its own ROS node thread (standalone use).
  Node(tools::ToolsManager* tools_manager);

  /// Adopts an externally-owned node + tf buffer (e.g. an app that already runs
  /// its own ROS connection) instead of spawning a thread. The node's lifetime
  /// is the owner's responsibility; this Node won't call rclcpp::shutdown().
  /// Manager creation is deferred to the event loop so it runs after this
  /// MapItem is fully constructed.
  Node(tools::ToolsManager* tools_manager, rclcpp::Node::SharedPtr node, tf2_ros::Buffer::SharedPtr buffer);

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

  // False when adopting an external node: the destructor must not stop a thread
  // it never started nor call rclcpp::shutdown() on a node it doesn't own.
  bool owns_thread_ = true;

  // Whether to create the geometry (PolygonStamped) manager. An adopting host
  // may already render PolygonStamped itself (e.g. a collision-zone overlay) and
  // not want a second, generic renderer of the same topics.
  bool create_geometry_manager_ = true;

  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer::SharedPtr transform_buffer_;

};

}  // namespace ros

} // namespace camp

#endif
