#include "grid_manager.h"
#include "../node.h"
#include "grid_map.h"
#include "occupancy_grid.h"
#include "../../map/layer_list.h"
#include "../names_manager.h"

#include <QDebug>

namespace camp
{
namespace ros
{
namespace grids
{

GridManager::GridManager(MapTool* parent):
  tools::LayerManager(parent, "Grid Manager")
{
  auto topics_manager = new TopicsManager(this, "Topics");
  topics_manager->setTypeFilter({"nav_msgs/msg/OccupancyGrid", "grid_map_msgs/msg/GridMap"});
  connect(topics_manager, &TopicsManager::namesUpdated, this, &GridManager::updateTopics);
}

void GridManager::updateTopics()
{
  auto topic_manager = firstChildOfType<TopicsManager>();
  if(!topic_manager)
    return;
  auto topics = topic_manager->namesAndTypes();
  auto node_item = parentOfType<Node>();
  auto node = node_item->node();
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
            auto grid = new OccupancyGrid(layers, node_item, topic.first.c_str());
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
            auto grid = new GridMap(layers, node_item, topic.first.c_str());
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
