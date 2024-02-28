#ifndef MARKERS_H
#define MARKERS_H

#include "ros/ros_widget.h"
#include "geographicsitem.h"
#include "ui_markers.h"
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

signals:
  void newMarkersMadeAvailable();

public slots:
  void setTopic(std::string topic, std::string type);
  void visibilityChanged();
  void updateBackground(BackgroundRaster * bg);
  void newMarkersAvailable();

private:
  void markerArrayCallback(const visualization_msgs::msg::MarkerArray &data);
  void markerCallback(const visualization_msgs::msg::Marker &data);
  void addMarkers(const std::vector<visualization_msgs::msg::Marker> &markers);

  QGeoCoordinate getGeoCoordinate(const geometry_msgs::msg::Pose &pose, const std_msgs::msg::Header &header);

  struct MarkerData
  {
    visualization_msgs::msg::Marker marker;
    QGeoCoordinate position;
    QPointF local_position;
    double rotation;
  };

  QPainterPath markerPath(const MarkerData& marker, BackgroundRaster* bg) const;

  Ui::Markers ui_;

  std::map<std::string, std::map<int32_t, std::shared_ptr<MarkerData> > > current_markers_;
  std::vector<std::shared_ptr<MarkerData> > new_markers_;
  std::mutex new_markers_mutex_;

  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_subscription_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_subscription_;

  double pixel_size_ = 1.0;
  bool is_visible_ = false;

};

#endif
