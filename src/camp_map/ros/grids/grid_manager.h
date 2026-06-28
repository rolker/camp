#ifndef CAMP_ROS_GRIDS_GRID_MANAGER_H
#define CAMP_ROS_GRIDS_GRID_MANAGER_H

#include "../../tools/layer_manager.h"

namespace camp
{

namespace ros
{

namespace grids
{
class GridManager: public tools::LayerManager
{
  Q_OBJECT
public:
  GridManager(MapTool* parent);

public slots:
  void updateTopics();

private:
  std::map <std::string, bool> grids_;

};

} // namespace grids
} // namespace ros
} // namespace camp
#endif
