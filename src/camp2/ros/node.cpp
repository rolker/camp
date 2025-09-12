#include "node.h"
#include <rclcpp/rclcpp.hpp>
#include <QTimer>
#include "node_thread.h"
#include "../tools/tools_manager.h"
#include "grids/grid_manager.h"
#include "markers/markers_manager.h"
#include "names_manager.h"
#include "graph_thread.h"


#include <QDebug>

namespace camp
{

namespace ros
{

void Node::init(int &argc, char ** argv)
{
  rclcpp::init(argc, argv);
  qRegisterMetaType<rclcpp::Node::SharedPtr>();
  qRegisterMetaType<tf2_ros::Buffer::SharedPtr>();
}

Node::Node(tools::ToolsManager* tools_manager):
  tools::LayerManager(tools_manager, "ROS")
{
  // static init method should be called before QApplication is created so ros::init
  // can deal with command line args before passing them to the application.
  //assert(ros::isInitialized());

  NodeThread* node = new NodeThread();
  node->moveToThread(&node_thread_);
  connect(&node_thread_, &QThread::finished, node, &QObject::deleteLater);
  connect(this, &Node::startNode, node, &NodeThread::start);
  connect(node, &NodeThread::started, this, &Node::nodeStarted);
  connect(node, &NodeThread::shuttingDown, this, &Node::nodeShuttingDown);

  node_thread_.start();
  emit startNode();
}

Node::~Node()
{
  emit shuttingDownRos();
  rclcpp::shutdown();
  node_thread_.quit();
  node_thread_.wait();
}

void Node::nodeStarted(rclcpp::Node::SharedPtr node, tf2_ros::Buffer::SharedPtr buffer)
{
  node_ = node;
  transform_buffer_ = buffer;
  graph_thread_ = new GraphThread(this);

  new grids::GridManager(this);
  new markers::MarkersManager(this);

  //new NodesManager(this);
  //new ServicesManager(this);
  new TopicsManager(this);

  graph_thread_->start();
}

void Node::nodeShuttingDown()
{
  qDebug() << "ROS node shutting down";
}

tf2_ros::Buffer::SharedPtr Node::transformBuffer()
{
  return transform_buffer_;
}

rclcpp::Node::SharedPtr Node::node()
{
  return node_;
}

GraphThread* Node::graphThread() const
{
  return graph_thread_;
}

} // namespace ros

} // namespace camp
