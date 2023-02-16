#ifndef GRID_H
#define GRID_H

#include <QWidget>
#include "geographicsitem.h"
#include "ui_grid.h"
#include "nav_msgs/OccupancyGrid.h"
#include "grid_map_msgs/GridMap.h"
#include "ros/ros.h"
#include <ros/callback_queue.h>

namespace tf2_ros
{
  class Buffer;
}

class Grid: public QWidget, public GeoGraphicsItem
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  Grid(QWidget* parent = nullptr, QGraphicsItem *parentItem = nullptr);
  QRectF boundingRect() const override;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
  int type() const override {return GridType;}
  void setTF2Buffer(tf2_ros::Buffer *buffer);
  void setPixelSize(double s);

public slots:
  void setTopic(std::string topic, std::string type);
  void newGridAvailable();
  void visibilityChanged();
  void updateBackground(BackgroundRaster * bg);

signals:
  void newGridMadeAvaiable();

private:
  void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr &data);
  void gridMapCallback(const grid_map_msgs::GridMap::ConstPtr &data);
  QGeoCoordinate getGeoCoordinate(const geometry_msgs::Pose &pose, const std_msgs::Header &header);

  struct GridData
  {
    QImage grid_image;
    QGeoCoordinate center;
    float meters_per_pixel;
  };

  Ui::Grid ui_;

  std::shared_ptr<GridData> current_grid_; 
  std::shared_ptr<GridData> new_grid_;
  std::mutex new_grid_mutex_;


  ros::CallbackQueue ros_queue_;
  std::shared_ptr<ros::AsyncSpinner> spinner_;
  ros::Subscriber subscriber_;

  double pixel_size_ = 1.0;
  bool is_visible_ = false;

  tf2_ros::Buffer* tf_buffer_ = nullptr;
};

#endif
