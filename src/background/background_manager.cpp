#include "background_manager.h"

#include "../map/layer_list.h"
#include "../map_tiles/map_tiles.h"
#include "../raster/raster_layer.h"
#include "../tools/tools_manager.h"
#include <QGraphicsScene>
#include <QMenu>
#include <QAction>
#include <QFileDialog>

namespace background
{

BackgroundManager::BackgroundManager(tools::ToolsManager* tools_manager):
  tools::LayerManager(tools_manager, "Background Manager")
{
}

void BackgroundManager::createDefaultLayers()
{
  map::LayerList* layers = topLevelLayers();
  if(layers)
  {
    // map_tiles::MapTiles* eo_tiles = new map_tiles::MapTiles(layers);
    // eo_tiles->setLabel("eo_tiles");
    // eo_tiles->setBaseUrl("http://localhost/eo_tiles/");
    // eo_tiles->setMinimumZoomLevel(2);
    // eo_tiles->setMaximumZoomLevel(6);
    // eo_tiles->setFlipY(true);

    map_tiles::MapTiles* openstreetmap = new map_tiles::MapTiles(layers, "openstreetmap");
    openstreetmap->setBaseUrl("https://tile.openstreetmap.org/");

    map_tiles::MapTiles* openseamap = new map_tiles::MapTiles(layers, "openseamap");
    openseamap->setBaseUrl("https://tiles.openseamap.org/seamark/");

    new raster::RasterLayer(layers, "/home/roland/data/BSB_ROOT/13283/13283_1.KAP");

    new raster::RasterLayer(layers, "/home/roland/data/BSB_ROOT/13283/13283_2.KAP");
  }
}

void BackgroundManager::contextMenu(QMenu* menu)
{
  auto open_raster_action = menu->addAction("Open raster");
  connect(open_raster_action, &QAction::triggered, this, &BackgroundManager::openRaster);
}

void BackgroundManager::openRaster()
{
  QString fname = QFileDialog::getOpenFileName(nullptr, tr("Open"));
  if(!fname.isEmpty())
  {
    auto layers = topLevelLayers();
    if(layers)
    {
      raster::RasterLayer* raster = new raster::RasterLayer(layers, fname);
    }
  }

}

} // namespace base_layers
