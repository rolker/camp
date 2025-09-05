#include "markers_manager.h"
#include "../node_manager.h"
#include "../../map/layer_list.h"
#include "markers.h"

namespace camp
{
namespace ros
{
namespace markers
{

MarkersManager::MarkersManager(NodeManager* parent):
  tools::LayerManager(parent, "Markers Manager")
{
  connect(parent, &NodeManager::topicsAvailable, this, &MarkersManager::updateTopics);
}

void MarkersManager::updateTopics(const NodeManager::TopicMap &topics)
{
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
          auto node_manager = qgraphicsitem_cast<NodeManager*>(parentItem());
          auto markers = new Markers(layers, node_manager, topic.first.c_str(), type.c_str());
          markers_[topic.first] = true;
        }
      }
    }

    }
  }
  {
  }
}

} // namespace markers
} // namespace ros
} // namespace camp
