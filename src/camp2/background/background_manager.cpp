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

namespace camp
{
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

    // [#99] NOAA nowCOAST weather radar (NEXRAD base reflectivity) as a stacked,
    // auto-refreshing WMTS overlay. Mirrors the NOAA charts block above, but:
    //   - default OFF (operator toggles it in the Layers tree),
    //   - ~0.65 opacity so it reads as a transparent overlay on the basemap, and
    //   - a 5-minute refresh (Phase 2) since radar imagery is time-varying.
    // Radar is a QPixmap MapTiles layer, so it rides the #98 tile lifecycle, NOT
    // the #96 GDAL raster path. On a fetch failure the tile stays blank (graceful
    // degradation in CachedFileLoader::downloadFinished — no crash, no UI block).
    // TODO: confirm endpoint — NOAA periodically restructures its ArcGIS service
    // paths; this nowCOAST radar WMTS URL was not verified live at implement time.
    auto radar_caps = new camp::wmts::Capabilities("NOAA_Radar", this);
    camp::map_tiles::MapTiles* radar = new camp::map_tiles::MapTiles(layers, "NOAA_radar");
    radar->setLayoutFromWMTS(*radar_caps);
    radar->setOpacity(0.65);   // transparent overlay on the basemap
    radar->setVisible(false);  // default OFF — operator opt-in via the layer tree
    radar->setRefreshInterval(5 * 60 * 1000);  // [#99 Phase 2] 5-minute cadence
    radar_caps->setUrl("https://nowcoast.noaa.gov/arcgis/rest/services/nowcoast/radar_meteo_imagery_nexrad_time/MapServer/WMTS");

    // new raster::RasterLayer(layers, "/home/roland/data/BSB_ROOT/13283/13283_1.KAP");

    // new raster::RasterLayer(layers, "/home/roland/data/BSB_ROOT/13283/13283_2.KAP");
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

} // namespace background
} // namespace camp
