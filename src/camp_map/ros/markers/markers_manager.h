#ifndef CAMP_ROS_MARKERS_MARKERS_MANAGER_H
#define CAMP_ROS_MARKERS_MARKERS_MANAGER_H

#include "../../tools/layer_manager.h"

namespace camp
{
namespace ros
{
namespace markers
{

class MarkersManager: public tools::LayerManager
{
  Q_OBJECT
public:
  MarkersManager(MapTool* parent);

public slots:
  void updateTopics();

private:
  std::map <std::string, bool> markers_;

};

} // namespace markers
} // namespace ros
} // namespace camp

#endif
