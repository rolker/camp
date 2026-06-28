#include "markers_manager.h"
#include "../node.h"
#include "../../map/layer_list.h"
#include "markers.h"
#include "../names_manager.h"

namespace camp
{
namespace ros
{
namespace markers
{

MarkersManager::MarkersManager(MapTool* parent):
  tools::LayerManager(parent, "Markers Manager")
{
  auto topics_manager = new TopicsManager(this, "Topics");
  topics_manager->setTypeFilter({"visualization_msgs/msg/MarkerArray", "visualization_msgs/msg/Marker"});
  connect(topics_manager, &TopicsManager::namesUpdated, this, &MarkersManager::updateTopics);
}

void MarkersManager::updateTopics()
{
  auto topic_manager = firstChildOfType<TopicsManager>();
  if(topic_manager)
  {
    auto topics = topic_manager->namesAndTypes();
    for(const auto& topic: topics)
    {
      for(const auto& type: topic.second)
      {
      if(type == "visualization_msgs/msg/MarkerArray" || type == "visualization_msgs/msg/Marker")
        {
          if(!markers_[topic.first])
          {
            auto layers = topLevelLayers();
            if(layers)
            {
              auto node = parentOfType<Node>();
              auto markers = new Markers(layers, node, topic.first.c_str(), type.c_str());
              markers_[topic.first] = true;
            }
          }
        }
      }
    }
  }
}

} // namespace markers
} // namespace ros
} // namespace camp
