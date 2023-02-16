#ifndef MARKERS_H
#define MARKERS_H

#include <QWidget>
#include "geographicsitem.h"
#include "ui_markers.h"
#include "visualization_msgs/MarkerArray.h"
#include "ros/ros.h"
#include <ros/callback_queue.h>

namespace tf2_ros
{
  class Buffer;
}

class Markers: public QWidget, public GeoGraphicsItem
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  Markers(QWidget* parent = nullptr, QGraphicsItem *parentItem = nullptr);
  QRectF boundingRect() const override;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
  int type() const override {return GridType;}
  void setTF2Buffer(tf2_ros::Buffer *buffer);
  void setPixelSize(double s);

signals:
  void newMarkersMadeAvailable();

public slots:
  void setTopic(std::string topic, std::string type);
  void visibilityChanged();
  void updateBackground(BackgroundRaster * bg);
  void newMarkersAvailable();

private:
  void markerArrayCallback(const visualization_msgs::MarkerArrayConstPtr &data);
  void markerCallback(const visualization_msgs::MarkerConstPtr &data);
  void addMarkers(const std::vector<visualization_msgs::Marker> &markers);

  QGeoCoordinate getGeoCoordinate(const geometry_msgs::Pose &pose, const std_msgs::Header &header);

  struct MarkerData
  {
    visualization_msgs::Marker marker;
    QGeoCoordinate position;
    QPointF local_position;
  };

  Ui::Markers ui_;

  std::map<std::string, std::map<int32_t, std::shared_ptr<MarkerData> > > current_markers_;
  std::vector<std::shared_ptr<MarkerData> > new_markers_;
  std::mutex new_markers_mutex_;

  ros::CallbackQueue ros_queue_;
  std::shared_ptr<ros::AsyncSpinner> spinner_;
  ros::Subscriber subscriber_;

  double pixel_size_ = 1.0;
  bool is_visible_ = false;

  tf2_ros::Buffer* tf_buffer_ = nullptr;
};

#endif
