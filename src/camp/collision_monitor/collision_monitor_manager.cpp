#include "collision_monitor_manager.h"

#include <optional>

#include <QColor>
#include <QTimer>

#include "collision_monitor/collision_monitor.h"
#include "ros/ros_context.h"

namespace
{
// Match the bridged/boat-side topic basenames robustly to whatever namespace
// prefix the udp_bridge applies (e.g. /bizzy/collision_slowdown_polygon or
// .../collision_monitor/slowdown_polygon).
bool isCollisionPolygon(const std::string& name, const char* kind)
{
  return name.find("collision") != std::string::npos &&
         name.find(kind) != std::string::npos &&
         name.find("polygon") != std::string::npos;
}
}  // namespace

CollisionMonitorManager::CollisionMonitorManager(QObject* parent):
  camp_ros::ROSObject(parent)
{
  scan_timer_ = new QTimer(this);
  connect(scan_timer_, &QTimer::timeout, this, &CollisionMonitorManager::scanForSources);
  scan_timer_->start(1000);

  state_timeout_ = new QTimer(this);
  state_timeout_->setSingleShot(true);
  connect(state_timeout_, &QTimer::timeout, this, &CollisionMonitorManager::clearActiveStates);
}

CollisionMonitorManager::~CollisionMonitorManager()
{
}

void CollisionMonitorManager::scanForSources()
{
  if(!node_)
    return;

  auto topics = node_->get_topic_names_and_types();
  for(const auto& topic: topics)
  {
    const auto& name = topic.first;
    for(const auto& type: topic.second)
    {
      if(type == "geometry_msgs/msg/PolygonStamped")
      {
        const bool is_slow = isCollisionPolygon(name, "slowdown");
        const bool is_stop = isCollisionPolygon(name, "stop");
        if((is_slow || is_stop) && monitors_.find(name) == monitors_.end())
        {
          // [#59 PR5] No QWidget parent — the zone renders via its GeoGraphicsItem
          // under the "Collision Monitor" Map layer (m_anchor); the per-zone
          // widget UI is unused now that visibility is the layer's checkbox.
          auto* monitor = new CollisionMonitor(nullptr, m_anchor);
          monitor->nodeStarted(node_, transform_buffer_);
          // stop zone = red, slowdown zone = amber.
          monitor->setKind(is_stop ? CollisionMonitor::Kind::Stop : CollisionMonitor::Kind::Slowdown);
          monitor->setColor(is_stop ? QColor(220, 30, 30) : QColor(240, 180, 0));
          monitor->setTopic(name);
          monitor->setObjectName(name.c_str());
          monitors_[name] = monitor;
        }
      }
      else if(type == "nav2_msgs/msg/CollisionMonitorState" && !state_bridge_)
      {
        rclcpp::CallbackGroup::SharedPtr cbg;
        auto ctx = camp_ros::RosContext::instance();
        if(ctx)
          cbg = ctx->group(camp_ros::RosContext::Group::Scene);

        state_bridge_ = std::make_unique<
          camp_ros::PlainTopicBridge<nav2_msgs::msg::CollisionMonitorState, uint8_t>>(
          node_, name, rclcpp::QoS(1), cbg,
          [](const nav2_msgs::msg::CollisionMonitorState& msg) -> std::optional<uint8_t>
          {
            return msg.action_type;
          },
          this,
          [this](uint8_t action_type) { this->onState(action_type); });
      }
    }
  }
}

void CollisionMonitorManager::onState(uint8_t action_type)
{
  using State = nav2_msgs::msg::CollisionMonitorState;
  // Light each zone exactly when the monitor reports its own action. The
  // monitor reports a single highest-priority action per cycle, so as an
  // obstacle closes the operator sees the slowdown zone fill (amber), then the
  // stop zone (red). CollisionMonitorState also carries polygon_name, but it's
  // the Nav2 polygon instance name and shares no string with the published
  // topic, so it can't be mapped to a zone client-side; action_type is the
  // reliable discriminator. All zones of the matching kind are lit (correct
  // for the single-zone-per-kind deployment, and honest for multi-zone setups
  // where we can't tell which same-kind zone fired).
  for(const auto& entry: monitors_)
  {
    const bool active = (entry.second->kind() == CollisionMonitor::Kind::Stop)
                          ? (action_type == State::STOP)
                          : (action_type == State::SLOWDOWN);
    entry.second->setActive(active);
  }

  // Restart the staleness watchdog: a fresh state message means the link is
  // live. 2 s comfortably covers the VPN-throttled 1 Hz state rate.
  state_timeout_->start(2000);
}

void CollisionMonitorManager::clearActiveStates()
{
  for(const auto& entry: monitors_)
    entry.second->setActive(false);
}

void CollisionMonitorManager::updateBackground()
{
  // [#59 PR6] Zones are parented to the persistent scene anchor at creation and
  // stay there. Forward so each zone repaints on a chart change (the zone's own
  // updateBackground no longer reparents — it just triggers a repaint).
  for(auto& entry: monitors_)
    entry.second->updateBackground();
}
