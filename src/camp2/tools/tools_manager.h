#ifndef TOOLS_TOOLS_MANAGER_H
#define TOOLS_TOOLS_MANAGER_H

#include "../map/map_item.h"

namespace tools
{

class ToolsManager: public map::MapItem
{
public:
  ToolsManager(map::MapItem *item);

  enum { Type = map::ToolsManagerType };

  int type() const override
  {
    // Enable the use of qgraphicsitem_cast with this item.
    return Type;
  }

};


} // namespace tools

#endif
