#ifndef CAMP_ROS_MARKERS_MARKERS_MANAGER_H
#define CAMP_ROS_MARKERS_MARKERS_MANAGER_H

#include "../../tools/layer_manager.h"
#include "../node_manager.h"

namespace camp_ros
{
class MarkersManager: public tools::LayerManager
{
  Q_OBJECT
public:
  MarkersManager(NodeManager* parent);

public slots:
  void updateTopics(const NodeManager::TopicMap& topics);

private:
  std::map <std::string, bool> markers_;

};

} // namespace camp_ros

#endif
