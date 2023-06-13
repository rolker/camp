#ifndef CAMP_ROS_GRIDS_GRID_MANAGER_H
#define CAMP_ROS_GRIDS_GRID_MANAGER_H

#include "../../tools/layer_manager.h"

namespace camp_ros
{
class NodeManager;

class GridManager: public tools::LayerManager
{
  Q_OBJECT
public:
  GridManager(NodeManager* parent);

public slots:
  void updateTopics(const QMap<QString, QString>& topics);

private:
  std::map <std::string, bool> grids_;

};

} // namespace camp_ros

#endif
