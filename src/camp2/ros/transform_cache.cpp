#include "transform_cache.h"

#include <tf2/time.h>
#include <tf2/exceptions.h>

namespace camp
{
namespace ros
{

geometry_msgs::msg::TransformStamped lookupEarthTransformCached(
    tf2_ros::Buffer& buffer,
    TransformCache& cache,
    const std::string& frame_id,
    std::chrono::steady_clock::time_point now,
    std::chrono::duration<double> staleness_budget)
{
  try
  {
    auto transform = buffer.lookupTransform("earth", frame_id, tf2::TimePointZero);
    cache[frame_id] = {transform, now};
    return transform;
  }
  catch (const tf2::TransformException&)
  {
    // Transient gap: hold the last good placement rather than drop the frame.
    // A zero-timeout lookup that drops on every miss caused white flicker /
    // dropped maps; reusing a quasi-static earth<-frame transform across a
    // brief hiccup is imperceptible. Cold start (no cache entry) or an
    // over-budget gap rethrows so the caller drops as it would have before.
    auto it = cache.find(frame_id);
    if (it != cache.end() && now - it->second.stamp <= staleness_budget)
      return it->second.transform;
    throw;
  }
}

}  // namespace ros
}  // namespace camp
