#include "layer_manager.h"
#include "tools_manager.h"

namespace tools
{

LayerManager::LayerManager(ToolsManager* tools_manager, const QString& object_name):
  MapTool(tools_manager, object_name)
{

}

} // namespace map
