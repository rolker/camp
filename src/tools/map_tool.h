#ifndef TOOLS_MAP_TOOL_H
#define TOOLS_MAP_TOOL_H

#include "../map/map_item.h"

namespace tools
{

class ToolsManager;

class MapTool: public map::MapItem
{
public:
  MapTool(ToolsManager* tools_manager, const QString& object_name);

};

} // namespace tools

#endif
