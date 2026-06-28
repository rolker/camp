#ifndef CAMP_ROS_GEOMETRY_GEOMETRY_MANAGER_H
#define CAMP_ROS_GEOMETRY_GEOMETRY_MANAGER_H

#include "../../tools/layer_manager.h"

namespace camp
{
namespace ros
{

namespace geometry
{
class GeometryManager: public tools::LayerManager
{
  Q_OBJECT
public:
  GeometryManager(MapTool* parent);

  enum { Type = map::RosNamesManagerType };

  int type() const override
  {
    // Enable the use of qgraphicsitem_cast with this item.
    return Type;
  }

  void contextMenuForItem(MapItem* item, QMenu* menu) override;

};

} // namespace geometry
} // namespace ros
} // namespace camp
#endif
