#include "node_thread.h"
#include "ros_context.h"
#include "tf2_ros/create_timer_ros.h"
//#include "node_manager.h"

namespace camp_ros
{

NodeThread::NodeThread()
{

}

NodeThread::~NodeThread() = default;

void NodeThread::start()
{
  node_ = std::make_shared<rclcpp::Node>("camp");

  buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());

  // From: https://docs.ros.org/en/kilted/Tutorials/Intermediate/Tf2/Using-Stamped-Datatypes-With-Tf2-Ros-MessageFilter.html
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node_->get_node_base_interface(),
    node_->get_node_timers_interface()
  );
  buffer_->setCreateTimerInterface(timer_interface);

  transform_listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_, node_);

  // Construct RosContext (creates the realtime/scene callback groups on the
  // node) before adding the node to the executor, so the executor sees the
  // groups during add_node.
  context_ = std::make_shared<RosContext>(node_, buffer_);
  RosContext::setInstance(context_);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_);

  emit started(node_, buffer_);

  executor.spin();

  RosContext::clearInstance();
  context_.reset();

  emit shuttingDown();
}



} // namespace camp_ros
