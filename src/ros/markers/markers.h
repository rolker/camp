#ifndef CAMP_ROS_MARKERS_MARKERS_H
#define CAMP_ROS_MARKERS_MARKERS_H

#include "../layer.h"
#include <visualization_msgs/msg/marker_array.hpp>
#include <QGeoCoordinate>

namespace camp_ros
{

struct MarkerData
{
  visualization_msgs::msg::Marker marker;
  QPointF position;
  double rotation = 0.0;
};

class MarkerNamespace;

class Markers: public Layer
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)

public:
  Markers(MapItem* parent, NodeManager* node_manager, QString topic, QString topic_type);


signals:
  void newMarkerData(MarkerData data);

private:
  void markerArrayCallback(const visualization_msgs::msg::MarkerArray &data);
  void markerCallback(const visualization_msgs::msg::Marker &data);
  void addMarkers(const std::vector<visualization_msgs::msg::Marker> &markers);


  MarkerNamespace * markerNamespace(const QString& marker_namepsace) const;

private slots:
  void updateMarker(const MarkerData& data);

private:
  std::string topic_;

  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_subscription_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_subscription_;

};

} // namespace camp_ros

Q_DECLARE_METATYPE(camp_ros::MarkerData);

#endif
