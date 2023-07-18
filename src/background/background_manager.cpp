#include "background_manager.h"

#include "../map/layer_list.h"
#include "../map_tiles/map_tiles.h"
#include "../map_tiles/osm.h"
#include "../raster/raster_layer.h"
#include "../tools/tools_manager.h"
#include <QGraphicsScene>
#include <QMenu>
#include <QAction>
#include <QFileDialog>
#include "../wmts/capabilities.h"

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
    new camp::map_tiles::MapTiles(layers, "openstreetmap", camp::osm::generateTileLayout("https://tile.openstreetmap.org/"));

    new camp::map_tiles::MapTiles(layers, "openseamap", camp::osm::generateTileLayout("https://tiles.openseamap.org/seamark/"));

    auto caps = new camp::wmts::Capabilities("NOAA_Charts", this);
    camp::map_tiles::MapTiles* noaa_charts = new camp::map_tiles::MapTiles(layers, "NOAA_charts");
    noaa_charts->setLayoutFromWMTS(*caps);
    caps->setUrl("https://gis.charttools.noaa.gov/arcgis/rest/services/MarineChart_Services/NOAACharts/MapServer/WMTS");

    new raster::RasterLayer(layers, "/home/roland/data/BSB_ROOT/13283/13283_1.KAP");

    new raster::RasterLayer(layers, "/home/roland/data/BSB_ROOT/13283/13283_2.KAP");
  }
}

QRectF BackgroundManager::boundingRect() const
{
  return QRectF(QPointF(-20037508.3427892, 20037508.3427892), QPointF(20037508.3427892, -20037508.3427892));

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
