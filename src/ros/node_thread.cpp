#include "node_thread.h"

#include "node_manager.h"

namespace camp_ros
{

NodeThread::NodeThread()
{

}

void NodeThread::start()
{
  node_ = std::make_shared<rclcpp::Node>("camp");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_);

  auto buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  transform_listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer);
  
  emit started(node_, buffer);

  executor.spin();

  emit shuttingDown();
}



} // namespace camp_ros
