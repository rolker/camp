#ifndef BACKGROUND_BACKGROUND_MANAGER_H
#define BACKGROUND_BACKGROUND_MANAGER_H

#include "../tools/layer_manager.h"

namespace tools
{
  class ToolsManager;
}

namespace background
{

// Handles raster and vector layers used
// for base maps. 
class BackgroundManager: public tools::LayerManager
{
  Q_OBJECT
public:
  BackgroundManager(tools::ToolsManager* tools_manager);

  enum { Type = map::BackgroundManagerType};

  int type() const override
  {
    // Enable the use of qgraphicsitem_cast with this item.
    return Type;
  }

  // Create default base maps, such as Open Street Map layers.
  void createDefaultLayers();

  void contextMenu(QMenu* menu) override;

private slots:
  void openRaster();
};

} // namespace base_layers

#endif
