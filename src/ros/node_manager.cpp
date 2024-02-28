#include "node_manager.h"
#include <rclcpp/rclcpp.hpp>
#include <QTimer>
#include "node_thread.h"
#include "../tools/tools_manager.h"
#include "grids/grid_manager.h"
#include "markers/markers_manager.h"

#include <QDebug>

namespace camp_ros
{

void NodeManager::init(int &argc, char ** argv)
{
  rclcpp::init(argc, argv);
  qRegisterMetaType<rclcpp::Node::SharedPtr>();
  qRegisterMetaType<tf2_ros::Buffer::SharedPtr>();
}

NodeManager::NodeManager(tools::ToolsManager* tools_manager):
  tools::LayerManager(tools_manager, "ROS")
{
  // static init method should be called before QApplication is created so ros::init
  // can deal with command line args before passing them to the application.
  //assert(ros::isInitialized());

  NodeThread* node = new NodeThread();
  node->moveToThread(&node_thread_);
  connect(&node_thread_, &QThread::finished, node, &QObject::deleteLater);
  connect(this, &NodeManager::startNode, node, &NodeThread::start);
  connect(node, &NodeThread::started, this, &NodeManager::nodeStarted);
  connect(node, &NodeThread::shuttingDown, this, &NodeManager::nodeShuttingDown);


  node_thread_.start();
  emit startNode();
}

NodeManager::~NodeManager()
{
  emit shuttingDownRos();
  rclcpp::shutdown();
  node_thread_.quit();
  node_thread_.wait();
}

void NodeManager::nodeStarted(rclcpp::Node::SharedPtr node, tf2_ros::Buffer::SharedPtr buffer)
{
  node_ = node;
  transform_buffer_ = buffer;

  GridManager* grid_manager = new GridManager(this);
  //connect(this, &NodeManager::topicsAvailable, grid_manager, &GridManager::updateTopics);

  MarkersManager* markers_manager = new MarkersManager(this);
}

void NodeManager::nodeShuttingDown()
{
  qDebug() << "ROS node shutting down";
}


tf2_ros::Buffer::SharedPtr NodeManager::transformBuffer()
{
  return transform_buffer_;
}

rclcpp::Node::SharedPtr NodeManager::node()
{
  return node_;
}

} // namespace camp_ros
