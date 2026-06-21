#include "background_manager.h"

#include "../map/layer_list.h"
#include "../map_tiles/map_tiles.h"
#include "../map_tiles/osm.h"
#include "../raster/raster_layer.h"
#include "../raster/gggs_store_layer.h"
#include "../tools/tools_manager.h"
#include <QDir>
#include <QFileInfo>
#include <QGraphicsScene>
#include <QMenu>
#include <QAction>
#include <QFileDialog>
#include <QSettings>
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

    // [#99] Weather radar (NEXRAD base reflectivity) as a stacked, auto-refreshing
    // overlay. Source: NOAA NEXRAD base reflectivity, redistributed as XYZ
    // Web-Mercator (EPSG:3857) tiles by Iowa State University's Environmental
    // Mesonet (IEM). nowCOAST itself serves this product only via WMS (dynamic
    // GetMap), not tiled WMTS, so it does not fit the MapTiles z/x/y path; IEM's
    // tile cache does (same NEXRAD origin). Endpoint verified live 2026-06-18.
    // The "n0q" product alias always serves the LATEST frame, so the 5-minute
    // refresh (Phase 2) genuinely fetches fresh imagery (no timestamp pinning).
    // Layer is:
    //   - default OFF (operator toggles it in the Layers tree),
    //   - ~0.65 opacity so it reads as a transparent overlay on the basemap, and
    //   - refreshed every 5 minutes since radar imagery is time-varying.
    // Radar is a QPixmap MapTiles layer, so it rides the #98 tile lifecycle, NOT
    // the #96 GDAL raster path. On a fetch failure the tile stays blank (graceful
    // degradation in CachedFileLoader::downloadFinished — no crash, no UI block).
    camp::map_tiles::MapTiles* radar = new camp::map_tiles::MapTiles(layers, "nexrad_radar",
        camp::osm::generateTileLayout("https://mesonet.agron.iastate.edu/cache/tile.py/1.0.0/nexrad-n0q-900913/"));
    radar->setOpacity(0.65);   // transparent overlay on the basemap
    radar->setVisible(false);  // default OFF — operator opt-in via the layer tree
    radar->setRefreshInterval(5 * 60 * 1000);  // [#99 Phase 2] 5-minute cadence

    // new raster::RasterLayer(layers, "/home/roland/data/BSB_ROOT/13283/13283_1.KAP");

    // new raster::RasterLayer(layers, "/home/roland/data/BSB_ROOT/13283/13283_2.KAP");

    // [camp#90] Re-create persisted GGGS tile stores + plain rasters so they
    // auto-load each session (persisted in openTileStore / openRaster). Skip
    // entries that no longer exist on disk.
    // [camp#102] GggsTile now reads only extent/metadata in its ctor; the band
    // pixels load lazily off a QtConcurrent worker, kicked from the first paint()
    // of a *visible* tile-set (tile-sets default OFF). So opening a large
    // multi-epoch store no longer blocks startup — only layers the operator turns
    // on read pixels. A QFileSystemWatcher on each store picks up tiles/epochs
    // that land after open. Visible-region LOD render is still future work
    // (separate issue).
    QSettings settings;
    const QStringList store_roots = settings.value("GggsStores/roots").toStringList();
    for(const QString& root : store_roots)
      if(QDir(root).exists())
        new raster::GggsStoreLayer(layers, root);
    const QStringList raster_files = settings.value("GggsRasters/files").toStringList();
    for(const QString& fname : raster_files)
      if(QFileInfo::exists(fname))
        new raster::RasterLayer(layers, fname);
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
  auto open_tile_store_action = menu->addAction("Open tile store");
  connect(open_tile_store_action, &QAction::triggered, this, &BackgroundManager::openTileStore);
}

void BackgroundManager::openRaster()
{
  QString fname = QFileDialog::getOpenFileName(nullptr, tr("Open"));
  if(fname.isEmpty())
    return;
  auto layers = topLevelLayers();
  if(!layers)
    return;
  new raster::RasterLayer(layers, fname);

  // Persist the raster so it auto-loads next session (createDefaultLayers
  // re-creates it), matching the tile-store persistence.
  QSettings settings;
  QStringList files = settings.value("GggsRasters/files").toStringList();
  if(!files.contains(fname))
  {
    files.append(fname);
    settings.setValue("GggsRasters/files", files);
  }
}

void BackgroundManager::openTileStore()
{
  QString directory = QFileDialog::getExistingDirectory(nullptr, tr("Open tile store"));
  if(directory.isEmpty())
    return;
  auto layers = topLevelLayers();
  if(!layers)
    return;
  new raster::GggsStoreLayer(layers, directory);

  // Persist the store root so it auto-loads next session (createDefaultLayers
  // re-creates it), matching the chart-list persistence.
  QSettings settings;
  QStringList roots = settings.value("GggsStores/roots").toStringList();
  if(!roots.contains(directory))
  {
    roots.append(directory);
    settings.setValue("GggsStores/roots", roots);
  }
}

} // namespace background
} // namespace camp
