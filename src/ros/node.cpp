#include "node.h"

#include "node_manager.h"

namespace camp_ros
{

Node::Node()
{
}

void Node::start(NodeManager* node_manager)
{
  ros::AsyncSpinner spinner(10);
  spinner.start();
  //if(!node_handle_)  
  node_handle_ = std::make_unique<ros::NodeHandle>();
  transform_listener_ = std::make_unique<tf2_ros::TransformListener>(node_manager->transformBuffer());
  emit started();
  ros::waitForShutdown();
  emit shuttingDown();
}



} // namespace camp_ros
