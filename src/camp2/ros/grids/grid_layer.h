#ifndef CAMP_ROS_GRIDS_GRID_LAYER_H
#define CAMP_ROS_GRIDS_GRID_LAYER_H

#include "../layer.h"

namespace camp
{
namespace ros
{
namespace grids
{

class GridMapLayerData;

class GridLayer : public Layer
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  GridLayer(MapItem* parent, NodeManager* node_manager, QString layer_name);

  enum { Type =  map::GridLayerType };

  int type() const override { return Type; }

public slots:
  void updateGridLayer(const GridMapLayerData& data);

};

} // namespace grids
} // namespace ros
} // namespace camp

#endif // CAMP_ROS_GRIDS_GRID_LAYER_H