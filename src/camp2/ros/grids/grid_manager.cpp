#include "grid_manager.h"
#include "../node_manager.h"
#include "grid_map.h"
#include "occupancy_grid.h"
#include "../../map/layer_list.h"

#include <QDebug>

namespace camp
{
namespace ros
{
namespace grids
{

GridManager::GridManager(NodeManager* parent):
  tools::LayerManager(parent, "Grid Manager")
{
  connect(parent, &NodeManager::topicsAvailable, this, &GridManager::updateTopics);
}

void GridManager::updateTopics(const NodeManager::TopicMap &topics)
{
  auto node_manager = qgraphicsitem_cast<NodeManager*>(parentItem());
  auto node = node_manager->node();
  if(!node)
    return;
  for(const auto& topic: topics)
  {
    if(node->count_publishers(topic.first) == 0)
      continue;
    for(const auto& type: topic.second)
    {
      if(type == "nav_msgs/msg/OccupancyGrid")
      {
        if(!grids_[topic.first])
        {
          auto layers = topLevelLayers();
          if(layers)
          {
            auto grid = new OccupancyGrid(layers, node_manager, topic.first.c_str());
            grids_[topic.first] = true;
          }
        }
      }
      if(type == "grid_map_msgs/msg/GridMap")
      {
        if(!grids_[topic.first])
        {
          auto layers = topLevelLayers();
          if(layers)
          {
            auto grid = new GridMap(layers, node_manager, topic.first.c_str());
            grids_[topic.first] = true;
          }

        }
      }
    }
  }
}

} // namespace grids
} // namespace ros
} // namespace camp
