#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

#include <QWidget>
#include "geographicsitem.h"
#include "ui_occupancy_grid.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"
#include <ros/callback_queue.h>

namespace tf2_ros
{
  class Buffer;
}

class OccupancyGrid: public QWidget, public GeoGraphicsItem
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  OccupancyGrid(QWidget* parent = nullptr, QGraphicsItem *parentItem = nullptr);
  QRectF boundingRect() const override;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
  int type() const override {return OccupancyGridType;}
  void setTF2Buffer(tf2_ros::Buffer *buffer);
  void setPixelSize(double s);

public slots:
  void setTopic(std::string topic);
  void newGridAvailable();

private:
  void dataCallback(const nav_msgs::OccupancyGrid::ConstPtr &data);

  struct GridData
  {
    QImage grid_image;
  };

  Ui::OccupancyGrid ui_;

  std::shared_ptr<GridData> current_grid_; 
  std::shared_ptr<GridData> new_grid_;
  std::mutex new_grid_mutex_;


  ros::CallbackQueue ros_queue_;
  std::shared_ptr<ros::AsyncSpinner> spinner_;
  ros::Subscriber subscriber_;

  double pixel_size_ = 1.0;

  tf2_ros::Buffer* tf_buffer_ = nullptr;
};

#endif