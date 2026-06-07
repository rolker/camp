#ifndef COLLISION_MONITOR_H
#define COLLISION_MONITOR_H

#include <memory>
#include <vector>

#include <QColor>
#include <QGeoCoordinate>

#include "ros/ros_widget.h"
#include "ros/topic_bridge.h"
#include "geographicsitem.h"
#include "collision_monitor/collision_monitor_converter.h"
#include "ui_collision_monitor.h"
#include "geometry_msgs/msg/polygon_stamped.hpp"

class BackgroundRaster;

/// One Nav2 Collision Monitor danger zone (slowdown or stop), rendered as a
/// georeferenced polygon on the map. Drawn as a thin outline when idle and
/// filled when the zone is actively gating motion. The active state is driven
/// externally (by CollisionMonitorManager, from nav2_msgs/CollisionMonitorState)
/// via setActive().
class CollisionMonitor: public camp_ros::ROSWidget, public GeoGraphicsItem
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  /// Which Collision Monitor action this zone corresponds to, so gating state
  /// can be routed to the right zone(s).
  enum class Kind { Slowdown, Stop };

  CollisionMonitor(QWidget* parent = nullptr, QGraphicsItem* parentItem = nullptr);

  QRectF boundingRect() const override;
  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;
  int type() const override { return CollisionMonitorType; }

  /// Base color for this zone's outline (and fill when active).
  void setColor(QColor color);

  void setKind(Kind kind) { kind_ = kind; }
  Kind kind() const { return kind_; }

  using PayloadT = std::shared_ptr<camp_collision_monitor::CollisionPolygonPayload>;

public slots:
  /// Subscribe to a geometry_msgs/PolygonStamped topic for this zone.
  void setTopic(std::string topic);
  /// Toggle filled (active) vs outline-only (idle) rendering.
  void setActive(bool active);
  void visibilityChanged();
  void updateBackground(BackgroundRaster* bg);

private:
  // Receiver slot for the TF bridge. Runs on the Qt main thread.
  void onPolygonPayload(PayloadT data);

  QPainterPath polygonPath() const;

  Ui::CollisionMonitor ui_;

  // TF-gated bridge: PolygonStamped (base-frame) gated on TF to "earth",
  // converted to geographic vertices on the executor thread, dispatched to
  // onPolygonPayload on the Qt main thread. Held as a member so the receiver
  // (this) outlives it, per the TopicBridge lifetime contract.
  std::unique_ptr<camp_ros::TfTopicBridge<geometry_msgs::msg::PolygonStamped, PayloadT>> bridge_;

  std::vector<QGeoCoordinate> points_;
  QColor color_ = QColor(240, 180, 0);
  Kind kind_ = Kind::Slowdown;
  bool active_ = false;
  bool is_visible_ = false;
};

#endif
