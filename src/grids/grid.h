#ifndef GRID_H
#define GRID_H

#include "ros/ros_widget.h"
#include "geographicsitem.h"
#include "ui_grid.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"

class Grid: public camp_ros::ROSWidget, public GeoGraphicsItem
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  Grid(QWidget* parent = nullptr, QGraphicsItem *parentItem = nullptr);
  QRectF boundingRect() const override;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
  int type() const override {return GridType;}
  void setPixelSize(double s);

public slots:
  void setTopic(std::string topic, std::string type);
  void newGridAvailable();
  void visibilityChanged();
  void updateBackground(BackgroundRaster * bg);

signals:
  void newGridMadeAvailable();

private:
  void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid &data);
  void gridMapCallback(const grid_map_msgs::msg::GridMap &data);
  QGeoCoordinate getGeoCoordinate(const geometry_msgs::msg::Pose &pose, const std_msgs::msg::Header &header);

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


  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_subscription_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_subscription_;

  double pixel_size_ = 1.0;
  bool is_visible_ = false;

  std::string topic_;
  std::string type_;
};

#endif
