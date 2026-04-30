#ifndef CAMP_ROS_CONTEXT_H
#define CAMP_ROS_CONTEXT_H

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

namespace camp_ros
{

/// Process-wide registry of the camp ROS node, TF buffer, and named callback
/// groups. Created once by `NodeThread` during startup and torn down on
/// shutdown. Replaces manual fan-out of node/buffer through Qt parent chains.
///
/// Thread-safe to read after `setInstance(...)` has been called and before
/// `clearInstance()`.
class RosContext
{
public:
  /// Latency-budget grouping for subscription callbacks.
  /// `Realtime` is reserved for watchdog inputs (heartbeat, nav, helm,
  /// mission status). `Scene` is for bulk display data (markers, grid, path,
  /// AIS contacts) whose callbacks may do heavier work and must not be
  /// allowed to starve realtime callbacks.
  enum class Group
  {
    Realtime,
    Scene
  };

  RosContext(rclcpp::Node::SharedPtr node, tf2_ros::Buffer::SharedPtr buffer);
  ~RosContext();

  RosContext(const RosContext&) = delete;
  RosContext& operator=(const RosContext&) = delete;

  rclcpp::Node::SharedPtr node() const { return node_; }
  tf2_ros::Buffer::SharedPtr buffer() const { return buffer_; }
  rclcpp::CallbackGroup::SharedPtr group(Group g) const;

  /// Returns nullptr until `setInstance(...)` has been called.
  static RosContext* instance();
  static void setInstance(RosContext* ctx);
  static void clearInstance();

private:
  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer::SharedPtr buffer_;
  rclcpp::CallbackGroup::SharedPtr realtime_group_;
  rclcpp::CallbackGroup::SharedPtr scene_group_;
};

} // namespace camp_ros

#endif
