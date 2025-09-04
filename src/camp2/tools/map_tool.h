#ifndef TOOLS_MAP_TOOL_H
#define TOOLS_MAP_TOOL_H

#include "../map/map_item.h"

namespace tools
{

class MapTool: public map::MapItem
{
public:
  MapTool(MapItem* parent, const QString& object_name);

};

} // namespace tools

#endif
