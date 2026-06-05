#ifndef CAMP_ROS_CONTEXT_H
#define CAMP_ROS_CONTEXT_H

#include <atomic>
#include <cstddef>
#include <memory>
#include <vector>
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
  /// mission status) — light, non-blocking callbacks that share one thread.
  /// `Scene` is for light overlay display (markers, collision zones). Heavy
  /// or potentially-blocking display streams (costmap/grid, path) must NOT
  /// share a group with these — they take a dedicated group from
  /// `nextDedicatedGroup()` so a slow/blocking callback can't starve the rest.
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

  /// Returns one of a fixed pool of dedicated MutuallyExclusive callback
  /// groups, handed out round-robin. Use for heavy or potentially-blocking
  /// display streams (e.g. costmap/grid, path) so a slow callback runs in its
  /// own group and can run concurrently with the shared Scene/Realtime groups,
  /// rather than serializing behind them.
  ///
  /// Caveat: this isolates *groups*, not threads. The MultiThreadedExecutor has
  /// a fixed thread pool, so concurrency is bounded by the thread count, and the
  /// round-robin reuse means that once more streams take groups than the pool
  /// size, two streams share a group and serialize again. Size the pool for the
  /// number of heavy streams. The pool is created at construction (before the
  /// node joins the executor), so the groups are visible to the executor.
  rclcpp::CallbackGroup::SharedPtr nextDedicatedGroup();

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
  std::vector<rclcpp::CallbackGroup::SharedPtr> dedicated_pool_;
  std::atomic<std::size_t> dedicated_index_{0};
};

} // namespace camp_ros

#endif
