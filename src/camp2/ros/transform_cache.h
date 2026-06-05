#ifndef CAMP_ROS_TRANSFORM_CACHE_H
#define CAMP_ROS_TRANSFORM_CACHE_H

#include <chrono>
#include <string>
#include <unordered_map>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>

namespace camp
{
namespace ros
{

/// A last-known earth<-frame transform plus the (steady-clock) instant it was
/// resolved, so a stale entry can be aged out.
struct CachedTransform
{
  geometry_msgs::msg::TransformStamped transform;
  std::chrono::steady_clock::time_point stamp;
};

/// frame_id -> last good earth<-frame transform.
using TransformCache = std::unordered_map<std::string, CachedTransform>;

/// Resolve earth<-frame_id at the latest available time, tolerating a transient
/// TF gap.
///
/// On a successful lookup the cache entry for `frame_id` is refreshed
/// (transform + `now`) and returned. On a lookup miss the cached transform is
/// reused while `now - cached.stamp <= staleness_budget`, so a momentary gap
/// holds the last placement instead of dropping the frame. The
/// `tf2::TransformException` is rethrown only on cold start (no cache entry for
/// `frame_id`) or once the gap exceeds the budget, leaving callers to drop as
/// they would have before.
///
/// Pure w.r.t. its inputs (`now` and the cache are passed in) so the
/// fresh/reuse/expire/cold-start branches are unit-testable without a node or
/// the system clock. Not internally synchronised — callers sharing a cache
/// across threads must serialise (see Layer::transform_cache_mutex_).
geometry_msgs::msg::TransformStamped lookupEarthTransformCached(
    tf2_ros::Buffer& buffer,
    TransformCache& cache,
    const std::string& frame_id,
    std::chrono::steady_clock::time_point now,
    std::chrono::duration<double> staleness_budget);

}  // namespace ros
}  // namespace camp

#endif  // CAMP_ROS_TRANSFORM_CACHE_H
