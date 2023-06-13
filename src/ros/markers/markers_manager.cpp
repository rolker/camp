#include "markers_manager.h"
#include "../node_manager.h"
#include "../../map/layer_list.h"
#include "markers.h"

namespace camp_ros
{

MarkersManager::MarkersManager(NodeManager* parent):
  tools::LayerManager(parent, "Markers Manager")
{
  connect(parent, &NodeManager::topicsAvailable, this, &MarkersManager::updateTopics);
}

void MarkersManager::updateTopics(const QMap<QString, QString> &topics)
{
  for(auto topic: topics.keys())
  {
    if(topics[topic] == "visualization_msgs/MarkerArray" || topics[topic] == "visualization_msgs/Marker")
    {
      if(!markers_[topic.toStdString()])
      {
        auto layers = topLevelLayers();
        if(layers)
        {
          auto node_manager = qgraphicsitem_cast<NodeManager*>(parentItem());
          auto markers = new Markers(layers, node_manager, topic, topics[topic]);
          markers_[topic.toStdString()] = true;
        }
      }
    }
  }
}


} // namespace camp_ros
