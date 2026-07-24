#ifndef BACKGROUND_BACKGROUND_MANAGER_H
#define BACKGROUND_BACKGROUND_MANAGER_H

#include "../tools/layer_manager.h"
#include "tile_layer_presets.h"

class QSettings;

namespace camp
{

namespace map_tiles
{
  class MapTiles;
}

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

  // Set a bounding rectangle to cover the world
  QRectF boundingRect() const override;

  // Create default base maps, such as Open Street Map layers.
  void createDefaultLayers();

  void contextMenu(QMenu* menu) override;

  // [camp#117] Create a tile layer from a preset (or the dialog's Custom
  // fields), apply its default presentation state, and persist it for restore
  // on the next start. Public so tests can exercise the add path without the
  // dialog. Returns the created layer, or nullptr if the preset's type is not
  // constructible (e.g. "wms" until #118) or a layer of that name is already
  // persisted/live.
  map_tiles::MapTiles* addTileLayerFromPreset(const TileLayerPreset& preset);

private slots:
  void openRaster();
  void addTileLayer();   // [camp#117] context-menu action -> AddTileLayerDialog

private:
  // [camp#117] Instantiate a MapTiles from persisted/preset construction
  // parameters. XYZ layers get their layout synchronously; WMTS layers follow
  // the async Capabilities path (caps parented to the LAYER, setLayoutFromWMTS,
  // THEN setUrl — the URL fetch fires ready(), so it must come last). Returns
  // nullptr for a type with no constructible layer (e.g. "wms" until #118).
  map_tiles::MapTiles* createTileLayer(map::LayerList* layers, const TileLayerPreset& preset);

  // [camp#117] Seed the default layer set (OSM only — operator decision
  // 2026-07-24) into QSettings. Called once, when the seeded sentinel is
  // absent (covers both fresh installs and upgrades from the hard-coded era);
  // the sentinel — not the ids list — gates it, so an operator who removes
  // every layer stays at zero layers across restarts. Takes the caller's
  // QSettings and the LayerList the restore loop will build under, so the seed
  // reads/writes through the same instance the outer restore uses and can write
  // the presentation group under the exact key the restored layer will read.
  void seedDefaultTileLayers(QSettings& settings, map::LayerList* layers);

  // [camp#117] Write a layer's construction parameters to the
  // BackgroundTileLayers group and add its name to the ids list.
  void persistTileLayer(QSettings& settings, const TileLayerPreset& preset);

  // [camp#117] Write a layer's default presentation state (opacity/visible) to
  // its MapItem/<settings_key> group — the same group Layer::readSettings reads
  // on the deferred itemConstructed pass. Shared by the add path (keyed by the
  // live layer's settingsKey()) and the seed (keyed by the key the restored
  // layer will resolve to), so a non-default-presentation preset round-trips
  // whether it is added or seeded.
  void persistTileLayerPresentation(QSettings& settings, const QString& settings_key,
                                    const TileLayerPreset& preset);
};

} // namespace background
} // namespace camp

#endif
