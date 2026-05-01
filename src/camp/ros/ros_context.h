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
/// `instance()` returns a `shared_ptr` guarded by an internal mutex. Holding
/// the returned `shared_ptr` keeps the underlying `RosContext` alive even
/// across a concurrent `clearInstance()`, so dereferences are safe for the
/// scope of the local. The shared_ptr is empty until `setInstance(...)` has
/// been called and after `clearInstance()` has run with no other holders.
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

  /// Returns an empty `shared_ptr` until `setInstance(...)` has been called.
  /// The returned `shared_ptr` keeps the instance alive for its own scope,
  /// even if `clearInstance()` runs concurrently on another thread.
  static std::shared_ptr<RosContext> instance();
  static void setInstance(std::shared_ptr<RosContext> ctx);
  static void clearInstance();

private:
  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer::SharedPtr buffer_;
  rclcpp::CallbackGroup::SharedPtr realtime_group_;
  rclcpp::CallbackGroup::SharedPtr scene_group_;
};

} // namespace camp_ros

#endif
