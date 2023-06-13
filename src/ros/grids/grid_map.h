#ifndef CAMP_ROS_GRIDS_GRID_MAP_H
#define CAMP_ROS_GRIDS_GRID_MAP_H

#include "../layer.h"
#include "grid_map_msgs/GridMap.h"

namespace camp_ros
{

struct GridMapLayerData
{
  QImage grid_image;
  QPointF center;
  float meters_per_pixel = 1.0;
};

class GridMap: public Layer
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)

public:
  GridMap(MapItem* parent, NodeManager* node_manager, QString topic);

  //QRectF boundingRect() const override;
  //void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
  
signals:
  void newLayerData(GridMapLayerData data);

public slots:
  void updateGridLayer(const GridMapLayerData& data);

private:
  void gridMapCallback(const grid_map_msgs::GridMap::ConstPtr &data);
  
  std::string topic_;

};

} // namespace camp_ros

Q_DECLARE_METATYPE(camp_ros::GridMapLayerData);

#endif
