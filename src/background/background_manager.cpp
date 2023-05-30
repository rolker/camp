#include "background_manager.h"

#include "../map/layer_list.h"
#include "../map_tiles/map_tiles.h"
#include "../raster/raster_layer.h"
#include <QGraphicsScene>

namespace background
{

BackgroundManager::BackgroundManager(tools::ToolsManager* tools_manager):
  tools::LayerManager(tools_manager, "Background Manager")
{
}

void BackgroundManager::createDefaultLayers()
{
  map::LayerList* layers;
  for(auto item: scene()->items())
  {
    layers = qgraphicsitem_cast<map::LayerList*>(item);
    if(layers)
      break;
  }

  if(layers)
  {
    // map_tiles::MapTiles* eo_tiles = new map_tiles::MapTiles(layers);
    // eo_tiles->setLabel("eo_tiles");
    // eo_tiles->setBaseUrl("http://localhost/eo_tiles/");
    // eo_tiles->setMinimumZoomLevel(2);
    // eo_tiles->setMaximumZoomLevel(6);
    // eo_tiles->setFlipY(true);

    map_tiles::MapTiles* openstreetmap = new map_tiles::MapTiles(layers);
    openstreetmap->setLabel("openstreetmap");
    openstreetmap->setBaseUrl("https://tile.openstreetmap.org/");

    map_tiles::MapTiles* openseamap = new map_tiles::MapTiles(layers);
    openseamap->setLabel("openseamap");
    openseamap->setBaseUrl("https://tiles.openseamap.org/seamark/");

    raster::RasterLayer* test_chart = new raster::RasterLayer(layers);
    test_chart->loadFile("/home/roland/data/BSB_ROOT/13283/13283_1.KAP");

    test_chart = new raster::RasterLayer(layers);
    test_chart->loadFile("/home/roland/data/BSB_ROOT/13283/13283_2.KAP");
  }
}

} // namespace base_layers
