#ifndef CAMP_FOOTPRINT_H
#define CAMP_FOOTPRINT_H

#include <memory>
#include <string>
#include <vector>

#include <QColor>
#include <QGeoCoordinate>

#include "ros/ros_object.h"
#include "ros/topic_bridge.h"
#include "geographicsitem.h"
#include "collision_monitor/collision_monitor_converter.h"
#include "geometry_msgs/msg/polygon_stamped.hpp"

/// [#64] The boat's published footprint (nav2 local_costmap/published_footprint),
/// rendered as a georeferenced outline polygon on the map so the operator can
/// judge close-quarters clearance. A non-widget ROS object that draws via its
/// GeoGraphicsItem under the "Boat Footprint" Map layer. Mirrors CollisionMonitor
/// (PolygonStamped -> TF-gated bridge -> geographic vertices -> scene), minus the
/// danger-zone kind/active-state machinery.
class Footprint : public camp_ros::ROSObject, public GeoGraphicsItem
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  // Reuse the collision-monitor polygon payload (a generic "polygon as geographic
  // vertices"); it carries no collision-specific state.
  using PayloadT = std::shared_ptr<camp_collision_monitor::CollisionPolygonPayload>;

  Footprint(QObject* parent = nullptr, QGraphicsItem* parentItem = nullptr);

  QRectF boundingRect() const override;
  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;
  int type() const override { return FootprintType; }

  /// Subscribe to a geometry_msgs/PolygonStamped footprint topic.
  void setTopic(std::string topic);

  /// (Re)create the subscription when the ROS node becomes available.
  void onNodeUpdated() override;

public slots:
  /// Repaint on chart change (positions are absolute Web-Mercator).
  void updateBackground();

private:
  void onPolygonPayload(PayloadT data);
  QPainterPath polygonPath() const;

  std::string topic_;
  std::unique_ptr<camp_ros::TfTopicBridge<geometry_msgs::msg::PolygonStamped, PayloadT>> bridge_;
  std::vector<QGeoCoordinate> points_;
  QColor color_ = QColor(0, 160, 255);  // boat-blue outline
};

#endif
