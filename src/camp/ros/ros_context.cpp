#include "ros_context.h"

#include <mutex>

namespace camp_ros
{
namespace
{
std::mutex g_instance_mutex;
RosContext* g_instance = nullptr;
} // namespace

RosContext::RosContext(rclcpp::Node::SharedPtr node, tf2_ros::Buffer::SharedPtr buffer)
: node_(std::move(node)),
  buffer_(std::move(buffer)),
  realtime_group_(node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
  scene_group_(node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
{
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

RosContext* RosContext::instance()
{
  std::lock_guard<std::mutex> lock(g_instance_mutex);
  return g_instance;
}

void RosContext::setInstance(RosContext* ctx)
{
  std::lock_guard<std::mutex> lock(g_instance_mutex);
  g_instance = ctx;
}

void RosContext::clearInstance()
{
  std::lock_guard<std::mutex> lock(g_instance_mutex);
  g_instance = nullptr;
}

} // namespace camp_ros
