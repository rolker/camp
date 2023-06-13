#include "node_manager.h"
#include <ros/ros.h>
#include <QTimer>
#include "node.h"
#include "../tools/tools_manager.h"
#include "grids/grid_manager.h"
#include "markers/markers_manager.h"

#include <QDebug>

namespace camp_ros
{

void NodeManager::init(int &argc, char ** argv)
{
  ros::init(argc, argv, "CCOMAutonomousMissionPlanner", ros::init_options::AnonymousName);
}

NodeManager::NodeManager(tools::ToolsManager* tools_manager):
  tools::LayerManager(tools_manager, "ROS"),transform_buffer_(ros::Duration(60.0))
{
  // static init method should be called before QApplication is created so ros::init
  // can deal with command line args before passing them to the application.
  assert(ros::isInitialized());


  QTimer* timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &NodeManager::checkRosCore);
  timer->start(1000);

  Node* node = new Node();
  node->moveToThread(&node_thread_);
  connect(&node_thread_, &QThread::finished, node, &QObject::deleteLater);
  connect(this, &NodeManager::startNode, node, &Node::start);
  connect(node, &Node::shuttingDown, this, &NodeManager::nodeShuttingDown);


  GridManager* grid_manager = new GridManager(this);
  //connect(this, &NodeManager::topicsAvailable, grid_manager, &GridManager::updateTopics);

  MarkersManager* markers_manager = new MarkersManager(this);

  node_thread_.start();
  emit startNode(this);
}

NodeManager::~NodeManager()
{
  emit shuttingDownRos();
  ros::shutdown();
  node_thread_.quit();
  node_thread_.wait();
}

void NodeManager::checkRosCore()
{
  QString status = "(";
  status += ros::master::getURI().c_str();
  if(ros::master::check())
  {
    status += " online)";
    ros::master::V_TopicInfo topic_info;
    ros::master::getTopics(topic_info);
    QMap<QString, QString> topics;
    for(const auto t: topic_info)
      topics[t.name.c_str()] = t.datatype.c_str();
    emit topicsAvailable(topics);
  }
  else
    status += " offline)";
  setStatus(status);
}

void NodeManager::nodeStarted()
{
}

void NodeManager::nodeShuttingDown()
{
  qDebug() << "ROS node shutting down";
}


tf2_ros::Buffer& NodeManager::transformBuffer()
{
  return transform_buffer_;
}


} // namespace camp_ros
