#include "ros_context.h"

#include <mutex>

namespace camp_ros
{
namespace
{
std::mutex g_instance_mutex;
std::shared_ptr<RosContext> g_instance;
} // namespace

RosContext::RosContext(rclcpp::Node::SharedPtr node, tf2_ros::Buffer::SharedPtr buffer)
: node_(std::move(node)),
  buffer_(std::move(buffer)),
  realtime_group_(node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
  scene_group_(node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
{
  // Pool of dedicated MutuallyExclusive groups for heavy/blocking display
  // streams (handed out by nextDedicatedGroup()). Created here — before the
  // node joins the executor — so the MultiThreadedExecutor picks them up.
  // Sized generously; on a 16-thread box this leaves ample headroom.
  constexpr std::size_t kDedicatedGroupCount = 8;
  dedicated_pool_.reserve(kDedicatedGroupCount);
  for (std::size_t i = 0; i < kDedicatedGroupCount; ++i)
    dedicated_pool_.push_back(
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
}

RosContext::~RosContext() = default;

rclcpp::CallbackGroup::SharedPtr RosContext::group(Group g) const
{
  switch (g)
  {
    case Group::Realtime:
      return realtime_group_;
    case Group::Scene:
      return scene_group_;
  }
  return nullptr;
}

rclcpp::CallbackGroup::SharedPtr RosContext::nextDedicatedGroup()
{
  if (dedicated_pool_.empty())
    return nullptr;
  const std::size_t idx =
    dedicated_index_.fetch_add(1) % dedicated_pool_.size();
  return dedicated_pool_[idx];
}

std::shared_ptr<RosContext> RosContext::instance()
{
  std::lock_guard<std::mutex> lock(g_instance_mutex);
  return g_instance;
}

void RosContext::setInstance(std::shared_ptr<RosContext> ctx)
{
  std::lock_guard<std::mutex> lock(g_instance_mutex);
  g_instance = std::move(ctx);
}

void RosContext::clearInstance()
{
  std::lock_guard<std::mutex> lock(g_instance_mutex);
  g_instance.reset();
}

} // namespace camp_ros
