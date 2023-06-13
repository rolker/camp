#include "grid_manager.h"
#include "../node_manager.h"
#include "grid_map.h"
#include "../../map/layer_list.h"

#include <QDebug>

namespace camp_ros
{

GridManager::GridManager(NodeManager* parent):
  tools::LayerManager(parent, "Grid Manager")
{
  connect(parent, &NodeManager::topicsAvailable, this, &GridManager::updateTopics);
}

void GridManager::updateTopics(const QMap<QString, QString> &topics)
{
  for(auto topic: topics.keys())
  {
    if(topics[topic] == "nav_msgs/OccupancyGrid")
    {
      //qDebug() << topic << " type: " << topics[topic];
    }
    if(topics[topic] == "grid_map_msgs/GridMap")
    {
      if(!grids_[topic.toStdString()])
      {
        auto layers = topLevelLayers();
        if(layers)
        {
          auto node_manager = qgraphicsitem_cast<NodeManager*>(parentItem());
          auto grid = new GridMap(layers, node_manager, topic);
          grids_[topic.toStdString()] = true;
        }

      }
    }
  }
}

} // namespace camp_ros
