#ifndef COLLISION_MONITOR_MANAGER_H
#define COLLISION_MONITOR_MANAGER_H

#include <cstdint>
#include <map>
#include <memory>
#include <string>

#include "ros/ros_widget.h"
#include "ros/topic_bridge.h"
#include "ui_collision_monitor_manager.h"
#include "nav2_msgs/msg/collision_monitor_state.hpp"

class QTimer;
class QGraphicsItem;
class BackgroundRaster;
class CollisionMonitor;

/// Discovers and displays the Nav2 Collision Monitor danger zones. Scans for
/// the slowdown/stop PolygonStamped topics and auto-creates a CollisionMonitor
/// overlay for each (no manual UI step, mirroring GridManager/MarkersManager),
/// and subscribes to the CollisionMonitorState topic to drive each zone's
/// active (filled) rendering by gating state.
class CollisionMonitorManager: public camp_ros::ROSWidget
{
  Q_OBJECT
public:
  explicit CollisionMonitorManager(QWidget* parent = nullptr);
  ~CollisionMonitorManager();

  // [#59 PR6] Persistent scene-origin anchor (AVP::originAnchor()) that the
  // collision zones parent to, so they render independent of any loaded chart.
  void setAnchor(QGraphicsItem* anchor) { m_anchor = anchor; }

public slots:
  void updateBackground(BackgroundRaster* bg);

private slots:
  void scanForSources();

private:
  // Receiver for the state bridge. Runs on the Qt main thread.
  void onState(uint8_t action_type);
  // Sets every zone idle (used when the state topic goes stale).
  void clearActiveStates();

  Ui::CollisionMonitorManager ui_;

  std::map<std::string, CollisionMonitor*> monitors_;

  std::unique_ptr<camp_ros::PlainTopicBridge<nav2_msgs::msg::CollisionMonitorState, uint8_t>> state_bridge_;

  QTimer* scan_timer_ = nullptr;
  // Single-shot watchdog: if no CollisionMonitorState arrives within its
  // interval (e.g. the link drops over the horizon), clear all zones so a
  // stale STOP/SLOWDOWN fill can't linger and mislead the operator.
  QTimer* state_timeout_ = nullptr;
  QGraphicsItem* m_anchor = nullptr;
};

#endif
