#ifndef CAMP_ROS_GRIDS_GRID_MAP_H
#define CAMP_ROS_GRIDS_GRID_MAP_H

#include "../layer.h"
#include "grid_map_msgs/msg/grid_map.hpp"
#include <QtConcurrent>

namespace camp_ros
{

struct GridMapLayerData
{
  QImage grid_image;
  QPointF center;
  float meters_per_pixel = 1.0;
  std::string layer_name;
  std::pair<double, double> range;
};

class GridLayer;

class GridMap: public Layer
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)

public:
  GridMap(MapItem* parent, NodeManager* node_manager, QString topic);

  
signals:
  void newLayerData(GridMapLayerData data);

private:
  void gridMapCallback(const grid_map_msgs::msg::GridMap &data);
  void processGridMap(const grid_map_msgs::msg::GridMap &data);

  QFuture<void> process_future_;

  GridLayer * gridLayer(const QString & layer_name) const;

private slots:
  void updateGridLayer(const GridMapLayerData& data);

private:
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr subscription_;  
  std::string topic_;

};

} // namespace camp_ros

Q_DECLARE_METATYPE(camp_ros::GridMapLayerData);

#endif
