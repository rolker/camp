#ifndef CAMP_ROS_GRIDS_OCCUPANCY_GRID_H
#define CAMP_ROS_GRIDS_OCCUPANCY_GRID_H

#include "../layer.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <QtConcurrent>

namespace camp_ros
{

struct OccupancyGridData
{
  QImage grid_image;
  QPointF origin;
  float meters_per_pixel = 1.0;
};

class OccupancyGrid: public Layer
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  OccupancyGrid(MapItem* parent, NodeManager* node_manager, QString topic);

signals:
  void occupancyGridUpdated(const OccupancyGridData &data);

private slots:
  void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid &grid);
  void processOccupancyGrid(const nav_msgs::msg::OccupancyGrid &grid);

  void updateOccupancyGrid(const OccupancyGridData &data);

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;

  QFuture<void> process_future_;
};

}  // namespace camp_ros

Q_DECLARE_METATYPE(nav_msgs::msg::OccupancyGrid)
Q_DECLARE_METATYPE(camp_ros::OccupancyGridData)

#endif  // CAMP_ROS_GRIDS_OCCUPANCY_GRID_H
