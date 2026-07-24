#include "background_manager.h"

#include "add_tile_layer_dialog.h"
#include "../map/layer_list.h"
#include "../map_tiles/map_tiles.h"
#include "../map_tiles/osm.h"
#include "../raster/raster_layer.h"
#include "../raster/gggs_tile_layer.h"
#include "../tools/tools_manager.h"
#include <QDir>
#include <QFileInfo>
#include <QGraphicsScene>
#include <QMenu>
#include <QAction>
#include <QFileDialog>
#include <QMessageBox>
#include <QSet>
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
    // [camp#117] Tile/WMTS layers are no longer hard-coded (the pre-#117 OSM/
    // OpenSeaMap/NOAA/NEXRAD blocks): the operator's chosen layers persist
    // under BackgroundTileLayers and are restored here, the same model GGGS
    // tile-sets and rasters already follow below. A one-time seed (OSM only —
    // operator decision 2026-07-24) keeps the first launch from being blank;
    // the seeded sentinel (not the ids list) gates it so removing every layer
    // sticks across restarts. Presets for the former hard-coded four (and
    // verified bathymetry sources) live in tile_layer_presets.h, added via the
    // "Add tile layer" context-menu action.
    QSettings settings;
    if(!settings.contains(tileLayerSeededKey()))
    {
      seedDefaultTileLayers(settings, layers);
      // [camp#117] Flush the seed to the backing store now. The seeded layer's
      // deferred readSettings() (itemConstructed's singleShot) runs through a
      // *separate* QSettings instance, so make the ordering explicit rather than
      // resting on this local instance's destruct-time sync.
      settings.sync();
    }
    const QStringList tile_ids = settings.value(tileLayerIdsKey()).toStringList();
    QSet<QString> restored_tiles;
    QStringList normalized_ids;   // deduped + constructible-only, rewritten below
    for(const QString& name : tile_ids)
    {
      if(restored_tiles.contains(name))
        continue;
      restored_tiles.insert(name);   // seen — drop any later duplicate of this name
      bool live = false;   // dedup against an already-live layer of this name
      for(map::MapItem* child : layers->childMapItems())
        if(auto* t = dynamic_cast<map_tiles::MapTiles*>(child))
          if(t->objectName() == name) { live = true; break; }
      if(live)
      {
        normalized_ids.append(name);   // a valid live layer already carries this id
        continue;
      }
      TileLayerPreset stored;
      stored.name = name;
      settings.beginGroup(tileLayerRootGroup());
      settings.beginGroup(tileLayerGroupKey(name));
      stored.type = settings.value("type").toString();
      stored.url = settings.value("url").toString();
      stored.layer_id = settings.value("layer_id").toString();
      stored.tile_matrix_set = settings.value("tile_matrix_set").toString();
      stored.refresh_ms = settings.value("refresh_ms", 0).toInt();
      settings.endGroup();
      settings.endGroup();
      if(createTileLayer(layers, stored))
        normalized_ids.append(name);   // constructed — keep; else drop the dead id
    }
    // [camp#117] Rewrite the persisted ids to the normalized set the restore just
    // proved out: duplicates (kept once) and entries whose type can't be built
    // (e.g. a "wms" id saved before #118) are dropped so they don't linger in
    // QSettings across future launches. Harmless at runtime, but keeps the stored
    // list honest. Only write when it actually changed to avoid churn.
    if(normalized_ids != tile_ids)
    {
      settings.setValue(tileLayerIdsKey(), normalized_ids);
      settings.sync();   // same flush discipline as the seed above
    }


    // [camp#104] Restore the operator's selected flat GGGS tile layers (ADR-0005).
    // Persistence moved off store *roots* (the retired nested GggsStoreLayer) and
    // onto the *selected* flat tile-sets the operator browsed in and added — each
    // a top-level GggsTileLayer, recreated here in stored order. Skip entries that
    // no longer exist on disk; dedup the list against itself AND any already-live
    // flat layer so a re-selected dir can't spawn a duplicate on restart.
    // [camp#102] GggsTile reads only extent/metadata in its ctor; the band pixels
    // load lazily off a QtConcurrent worker, kicked from the first paint() of a
    // *visible* tile-set (tile-sets default OFF). So restoring tile layers does
    // not block startup — only layers the operator turns on read pixels.
    // [camp#104] One-time reset of the retired store-roots key (ADR-0003 §4
    // no-back-compat / ADR-0005): drop it, do not migrate, so a stale value can't
    // resurrect a nested store tree.
    settings.remove("GggsStores/roots");
    const QStringList tile_dirs = settings.value("GggsTileLayers/dirs").toStringList();
    QSet<QString> restored;
    for(const QString& dir : tile_dirs)
    {
      if(restored.contains(dir) || !QDir(dir).exists())
        continue;
      bool live = false;   // dedup against an already-live flat layer on this dir
      for(map::MapItem* child : layers->childMapItems())
        if(auto* g = dynamic_cast<raster::GggsTileLayer*>(child))
          if(g->directory() == dir) { live = true; break; }
      if(live)
        continue;
      new raster::GggsTileLayer(layers, dir);
      restored.insert(dir);
    }
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
  // [camp#104] "Open tile store" is retired: GGGS stores are now browsed and
  // composed through the catalog browser tab (ADR-0005), not mounted from here.
  auto add_tile_action = menu->addAction("Add tile layer");
  connect(add_tile_action, &QAction::triggered, this, &BackgroundManager::addTileLayer);
}

map_tiles::MapTiles* BackgroundManager::createTileLayer(map::LayerList* layers, const TileLayerPreset& preset)
{
  if(preset.type == "xyz")
  {
    auto tiles = new map_tiles::MapTiles(layers, preset.name,
                                         osm::generateTileLayout(preset.url.toStdString()));
    if(preset.refresh_ms > 0)
      tiles->setRefreshInterval(preset.refresh_ms);
    return tiles;
  }
  if(preset.type == "wmts")
  {
    // Async ordering matters: the layer registers for the layout first, then
    // setUrl() comes LAST — the URL fetch is what fires ready().
    auto tiles = new map_tiles::MapTiles(layers, preset.name);
    // [camp#117] Parent Capabilities to the layer, not to this (the long-lived
    // BackgroundManager). MapTiles keeps only a non-owning pointer, so parenting
    // to `this` leaked one Capabilities per WMTS add/remove/re-add. As a child of
    // the layer it still outlives construction (the layer is not destroyed here)
    // and is destroyed with the layer on removal — matching its actual lifetime.
    auto caps = new wmts::Capabilities(preset.name, tiles);
    tiles->setLayoutFromWMTS(*caps, preset.layer_id, preset.tile_matrix_set);
    caps->setUrl(preset.url);
    if(preset.refresh_ms > 0)
      tiles->setRefreshInterval(preset.refresh_ms);
    return tiles;
  }
  // No constructible layer for this type (e.g. "wms" until #118).
  return nullptr;
}

map_tiles::MapTiles* BackgroundManager::addTileLayerFromPreset(const TileLayerPreset& preset)
{
  auto layers = topLevelLayers();
  if(!layers)
    return nullptr;
  // The name is the persistence identity — an empty one would persist an empty
  // ids entry. The dialog gates this, but this method is public (tests,
  // programmatic callers), so guard here too.
  if(preset.name.isEmpty())
    return nullptr;
  // One layer per name: the name is the persistence identity (ids list +
  // settings group + MapItem/<settingsKey>), so a duplicate would alias state.
  QSettings settings;
  if(settings.value(tileLayerIdsKey()).toStringList().contains(preset.name))
    return nullptr;
  for(map::MapItem* child : layers->childMapItems())
    if(auto* t = dynamic_cast<map_tiles::MapTiles*>(child))
      if(t->objectName() == preset.name)
        return nullptr;

  auto tiles = createTileLayer(layers, preset);
  if(!tiles)
    return nullptr;

  // Apply the preset's default presentation state now AND write it into the
  // layer's own settings group: readSettings() runs deferred (itemConstructed's
  // singleShot timer) and would otherwise clobber these with its 1.0/true
  // defaults. From here on the standard MapItem mechanism owns opacity/visible.
  tiles->setOpacity(preset.opacity);
  tiles->setVisible(preset.visible);
  persistTileLayerPresentation(settings, tiles->settingsKey(), preset);

  persistTileLayer(settings, preset);
  return tiles;
}

void BackgroundManager::addTileLayer()
{
  AddTileLayerDialog dialog;
  if(dialog.exec() != QDialog::Accepted)
    return;
  const TileLayerPreset selection = dialog.selection();
  if(!addTileLayerFromPreset(selection))
    // The dialog's OK-gate blocks inert/empty selections, so a refusal here
    // means a name collision with an existing/persisted layer — say so instead
    // of closing silently.
    QMessageBox::warning(nullptr, tr("Add tile layer"),
                         tr("A tile layer named \"%1\" already exists.").arg(selection.name));
}

void BackgroundManager::seedDefaultTileLayers(QSettings& settings, map::LayerList* layers)
{
  // OSM only (operator decision 2026-07-24): a single basemap so the first
  // launch is not blank; everything else is operator-added from presets. The
  // sentinel is written even if the preset table were to lose "openstreetmap" —
  // seeding must never re-fire.
  for(const auto& preset : builtinPresets())
    if(preset.name == "openstreetmap")
    {
      persistTileLayer(settings, preset);
      // Write the presentation group under the key the restored layer will read.
      // settingsKey() defaults to itemID() (parent path + objectName), so the
      // restore loop's MapTiles named preset.name under `layers` resolves to
      // exactly this key — the seed thus round-trips presentation like the add
      // path, even if a future seed preset carries non-default opacity/visible.
      persistTileLayerPresentation(settings, layers->itemID() + "/" + preset.name, preset);
      break;
    }
  settings.setValue(tileLayerSeededKey(), true);
}

void BackgroundManager::persistTileLayerPresentation(QSettings& settings,
                                                     const QString& settings_key,
                                                     const TileLayerPreset& preset)
{
  settings.beginGroup("MapItem");
  settings.beginGroup(settings_key);
  settings.setValue("opacity", preset.opacity);
  settings.setValue("visible", preset.visible);
  settings.endGroup();
  settings.endGroup();
}

void BackgroundManager::persistTileLayer(QSettings& settings, const TileLayerPreset& preset)
{
  QStringList ids = settings.value(tileLayerIdsKey()).toStringList();
  if(!ids.contains(preset.name))
  {
    ids.append(preset.name);
    settings.setValue(tileLayerIdsKey(), ids);
  }
  settings.beginGroup(tileLayerRootGroup());
  settings.beginGroup(tileLayerGroupKey(preset.name));
  settings.setValue("type", preset.type);
  settings.setValue("url", preset.url);
  settings.setValue("refresh_ms", preset.refresh_ms);
  if(!preset.layer_id.isEmpty())
    settings.setValue("layer_id", preset.layer_id);
  if(!preset.tile_matrix_set.isEmpty())
    settings.setValue("tile_matrix_set", preset.tile_matrix_set);
  settings.endGroup();
  settings.endGroup();
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

} // namespace background
} // namespace camp
