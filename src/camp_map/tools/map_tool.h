#ifndef TOOLS_MAP_TOOL_H
#define TOOLS_MAP_TOOL_H

#include "../map/map_item.h"

namespace camp
{
namespace tools
{

class MapTool: public map::MapItem
{
public:
  MapTool(MapItem* parent, const QString& object_name);

  enum { Type = map::MapToolType };

  int type() const override
  {
    // Enable the use of qgraphicsitem_cast with this item.
    return Type;
  }


};

} // namespace tools
} // namespace camp

#endif
