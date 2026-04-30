#ifndef MARKERS_H
#define MARKERS_H

#include "ros/ros_widget.h"
#include "ros/tf_dispatcher.h"
#include "ros/topic_bridge.h"
#include "geographicsitem.h"
#include "markers/markers_converter.h"
#include "ui_markers.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


class Markers: public camp_ros::ROSWidget, public GeoGraphicsItem
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  Markers(QWidget* parent = nullptr, QGraphicsItem *parentItem = nullptr);
  QRectF boundingRect() const override;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
  int type() const override {return GridType;}

  void setPixelSize(double s);

  using MarkerData = camp_markers::MarkerPayload;

public slots:
  void setTopic(std::string topic, std::string type);
  void visibilityChanged();
  void updateBackground(BackgroundRaster * bg);

private:
  // Receiver slot for the TF dispatcher. Runs on the Qt main thread.
  void onMarkerPayload(std::shared_ptr<MarkerData> data);

  // Marker-array subscription path: feeds the dispatcher with each marker.
  // Runs on the ROS executor thread.
  void onMarkerArrayMessage(const visualization_msgs::msg::MarkerArray & data);

  void purgeExpiredMarkers();

  QPainterPath markerPath(const MarkerData& marker, BackgroundRaster* bg) const;

  Ui::Markers ui_;

  std::map<std::string, std::map<int32_t, std::shared_ptr<MarkerData>>> current_markers_;

  // TF-gated dispatcher shared by both topic types: each Marker is fed in,
  // gated on TF to "earth", converted to MarkerData on the executor thread,
  // and dispatched to onMarkerPayload on the Qt main thread.
  std::unique_ptr<camp_ros::TfDispatcher<visualization_msgs::msg::Marker, std::shared_ptr<MarkerData>>> marker_dispatcher_;
  message_filters::Subscriber<visualization_msgs::msg::Marker> marker_subscription_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_subscription_;

  double pixel_size_ = 1.0;
  bool is_visible_ = false;
};

#endif
