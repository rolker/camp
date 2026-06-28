#include "geometry_manager.h"
#include "../../tools/map_tool.h"
#include "../names_manager.h"
#include "../name.h"
#include <QMenu>
#include "polygon.h"
#include "../node.h"
#include "../../map/layer_list.h"

namespace camp
{
namespace ros
{
namespace geometry
{

GeometryManager::GeometryManager(MapTool* parent)
  : tools::LayerManager(parent, "Geometry Manager")
{
  auto topics_manager = new TopicsManager(this, "Topics");
  topics_manager->setTypeFilter({"geometry_msgs/msg/PolygonStamped",});
}

void GeometryManager::contextMenuForItem(MapItem* item, QMenu* menu)
{
  auto topic_name = qgraphicsitem_cast<Name*>(item);
  if(topic_name)
  {
    for(const auto& type : topic_name->types())
    {
      if(type == "geometry_msgs/msg/PolygonStamped")
      {
        auto action = menu->addAction("Create polygon layer");
        connect(action, &QAction::triggered, [this, topic_name]()
        {
          auto layers = topLevelLayers();
          if(layers)
          {
            auto node_item = parentOfType<Node>();
            if(!node_item)
              return;
            auto node = node_item->node();
            if(!node)
              return;
            auto layer = new Polygon(layers, node_item, topic_name->full_name().c_str());
            //layer->setObjectName(topic_name->basename().c_str());
          }
        });
      }
    }
  }
 

}


} // namespace geometry
} // namespace ros
} // namespace camp
