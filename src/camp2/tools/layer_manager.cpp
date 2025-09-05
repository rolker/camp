#include "layer_manager.h"
#include "tools_manager.h"
#include "../map/map.h"
#include <QGraphicsScene>

namespace camp
{
namespace tools
{

LayerManager::LayerManager(MapItem* parent, const QString& object_name):
  MapTool(parent, object_name)
{

}

map::LayerList* LayerManager::topLevelLayers()
{
  return parentMap()->topLevelLayers();
}

} // namespace tools
} // namespace camp

