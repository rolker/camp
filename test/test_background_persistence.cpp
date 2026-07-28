// [camp#117] Background tile-layer persistence (ADR-0003 addendum).
//
// Tile/WMTS background layers moved from hard-coded createDefaultLayers()
// blocks to QSettings persistence (BackgroundTileLayers), with a one-time
// OSM-only seed. These tests pin:
//   - fresh settings -> seed produces exactly one layer (openstreetmap) and
//     writes the seeded sentinel;
//   - existing key -> no re-seed; persisted entries restored, deduped;
//   - add-from-preset -> layer created with the preset's presentation
//     defaults, construction params round-trip through a restart;
//   - remove -> the layer is dropped from ids + its group (the must-fix:
//     removal happens on the layer, not BackgroundManager) and stays gone,
//     WITHOUT re-triggering the seed (sentinel, not ids, gates seeding);
//   - the AddTileLayerDialog greys out inert (WMS, pre-#118) presets.
//
// No network fires: layouts are built but nothing paints tiles offscreen, and
// WMTS capabilities fetches go through CachedFileLoader which is never pumped.

#include <gtest/gtest.h>

#include <QApplication>
#include <QSettings>

#include "background/background_manager.h"
#include "background/add_tile_layer_dialog.h"
#include "background/tile_layer_presets.h"
#include "map/map.h"
#include "map/layer_list.h"
#include "map/map_item.h"
#include "map_tiles/map_tiles.h"
#include "tools/tools_manager.h"

#include <QListWidget>

using camp::background::AddTileLayerDialog;
using camp::background::BackgroundManager;
using camp::background::TileLayerPreset;
using camp::background::builtinPresets;
using camp::background::tileLayerIdsKey;
using camp::background::tileLayerSeededKey;
using camp::map::Map;
using camp::map::MapItem;
using camp::map_tiles::MapTiles;

namespace
{

int tileLayersNamed(camp::map::LayerList* layers, const QString& name)
{
  int n = 0;
  for(MapItem* child : layers->childMapItems())
    if(auto* t = dynamic_cast<MapTiles*>(child))
      if(t->objectName() == name)
        ++n;
  return n;
}

int tileLayerCount(camp::map::LayerList* layers)
{
  int n = 0;
  for(MapItem* child : layers->childMapItems())
    if(dynamic_cast<MapTiles*>(child))
      ++n;
  return n;
}

BackgroundManager* backgroundManager(Map& map)
{
  return map.toolsManager()->firstChildOfType<BackgroundManager>(true);
}

TileLayerPreset presetNamed(const QString& name)
{
  for(const auto& preset : builtinPresets())
    if(preset.name == name)
      return preset;
  return {};
}

}  // namespace

// Fresh settings: the seed fires once, produces exactly the OSM layer, and
// writes the sentinel. None of the pre-#117 hard-coded layers reappear.
TEST(BackgroundPersistence, FreshSettingsSeedsOsmOnly)
{
  QSettings().clear();
  Map map;
  EXPECT_EQ(tileLayerCount(map.topLevelLayers()), 1);
  EXPECT_EQ(tileLayersNamed(map.topLevelLayers(), "openstreetmap"), 1);
  EXPECT_EQ(QSettings().value(tileLayerIdsKey()).toStringList(),
            QStringList{"openstreetmap"});
  EXPECT_TRUE(QSettings().value(tileLayerSeededKey()).toBool());
}

// A persisted set restores as-is — no re-seed on top, duplicates collapsed.
TEST(BackgroundPersistence, RestoreExistingNoReseedAndDedup)
{
  QSettings settings;
  settings.clear();
  settings.setValue(tileLayerSeededKey(), true);
  settings.setValue(tileLayerIdsKey(), QStringList{"custom_a", "custom_a"});
  settings.beginGroup("BackgroundTileLayers");
  settings.beginGroup("custom_a");
  settings.setValue("type", "xyz");
  settings.setValue("url", "https://tiles.example.org/");
  settings.endGroup();
  settings.endGroup();

  Map map;
  EXPECT_EQ(tileLayerCount(map.topLevelLayers()), 1);
  EXPECT_EQ(tileLayersNamed(map.topLevelLayers(), "custom_a"), 1);
  EXPECT_EQ(tileLayersNamed(map.topLevelLayers(), "openstreetmap"), 0);
}

// Add-from-preset applies the preset's presentation defaults immediately and
// the construction parameters round-trip: a second Map restores the layer.
TEST(BackgroundPersistence, AddFromPresetRoundTrips)
{
  QSettings().clear();
  {
    Map map;
    MapTiles* radar = backgroundManager(map)->addTileLayerFromPreset(presetNamed("nexrad_radar"));
    ASSERT_NE(radar, nullptr);
    EXPECT_DOUBLE_EQ(radar->opacity(), 0.65);
    EXPECT_FALSE(radar->isVisible());
    // Pump the event loop so the deferred readSettings fires (MapItem's ctor
    // schedules itemConstructed via QTimer::singleShot(0)). The add path wrote
    // the presentation group, so readSettings must restore 0.65/hidden rather
    // than clobber to the 1.0/visible defaults — without the group write this
    // would regress here, not just on restart.
    QCoreApplication::processEvents();
    EXPECT_DOUBLE_EQ(radar->opacity(), 0.65);
    EXPECT_FALSE(radar->isVisible());
    // Re-adding the same preset is refused: name = persistence identity.
    EXPECT_EQ(backgroundManager(map)->addTileLayerFromPreset(presetNamed("nexrad_radar")),
              nullptr);
  }
  Map map;
  EXPECT_EQ(tileLayersNamed(map.topLevelLayers(), "nexrad_radar"), 1);
  EXPECT_EQ(tileLayersNamed(map.topLevelLayers(), "openstreetmap"), 1);

  // Presentation truly round-trips through QSettings: the restore path only
  // builds the layer (default 1.0/visible), then the deferred readSettings pulls
  // back the persisted 0.65/hidden. Pump the loop and confirm on the restored layer.
  MapTiles* restored = nullptr;
  for(MapItem* child : map.topLevelLayers()->childMapItems())
    if(auto* t = dynamic_cast<MapTiles*>(child))
      if(t->objectName() == "nexrad_radar")
        restored = t;
  ASSERT_NE(restored, nullptr);
  QCoreApplication::processEvents();
  EXPECT_DOUBLE_EQ(restored->opacity(), 0.65);
  EXPECT_FALSE(restored->isVisible());
}

// A preset with no constructible type is persisted-nothing and creates
// nothing. ([camp#118] the builtin WMS presets are live now — GEBCO
// constructs via the per-tile GetMap path — so this uses a synthetic type.)
TEST(BackgroundPersistence, UnconstructibleTypeRefused)
{
  QSettings().clear();
  Map map;
  TileLayerPreset bogus;
  bogus.name = "bogus_layer";
  bogus.type = "notatype";
  bogus.url = "https://tiles.example.org/";
  EXPECT_EQ(backgroundManager(map)->addTileLayerFromPreset(bogus), nullptr);
  EXPECT_FALSE(QSettings().value(tileLayerIdsKey()).toStringList().contains("bogus_layer"));
}

// [camp#118] A live WMS preset (GEBCO) constructs through the per-tile GetMap
// path and round-trips like any other tile layer.
TEST(BackgroundPersistence, WmsPresetConstructsAndRoundTrips)
{
  QSettings().clear();
  {
    Map map;
    ASSERT_NE(backgroundManager(map)->addTileLayerFromPreset(presetNamed("GEBCO_bathymetry")),
              nullptr);
  }
  Map map;
  EXPECT_EQ(tileLayersNamed(map.topLevelLayers(), "GEBCO_bathymetry"), 1);
}

// An empty preset name is refused outright (the name IS the persistence
// identity; the dialog gates this, but the method is public).
TEST(BackgroundPersistence, EmptyPresetNameRefused)
{
  QSettings().clear();
  Map map;
  EXPECT_EQ(backgroundManager(map)->addTileLayerFromPreset({}), nullptr);
  EXPECT_EQ(QSettings().value(tileLayerIdsKey()).toStringList(),
            QStringList{"openstreetmap"});
}

// The must-fix: removing a persisted tile layer de-persists it (ids + group)
// and it stays gone on the next start — the seed does NOT re-fire, because the
// sentinel, not the ids list, gates seeding.
TEST(BackgroundPersistence, RemoveDePersistsAndSticks)
{
  QSettings().clear();
  {
    Map map;
    ASSERT_EQ(tileLayersNamed(map.topLevelLayers(), "openstreetmap"), 1);
    for(MapItem* child : map.topLevelLayers()->childMapItems())
      if(auto* t = dynamic_cast<MapTiles*>(child))
        if(t->objectName() == "openstreetmap")
        {
          // removeFromMap() calls onRemovedFromMap() synchronously before
          // scheduling the delete, so the de-persist is observable immediately.
          t->removeFromMap();
          break;
        }
    EXPECT_TRUE(QSettings().value(tileLayerIdsKey()).toStringList().isEmpty());
  }
  Map map;
  EXPECT_EQ(tileLayerCount(map.topLevelLayers()), 0);
}

// A MapTiles that was never persisted must not disturb the persisted set when
// removed (the generic-class guard in MapTiles::onRemovedFromMap).
TEST(BackgroundPersistence, UnpersistedLayerRemovalIsNoOp)
{
  QSettings().clear();
  Map map;
  auto* extra = new MapTiles(map.topLevelLayers(), "scratch_overlay");
  extra->removeFromMap();
  EXPECT_EQ(QSettings().value(tileLayerIdsKey()).toStringList(),
            QStringList{"openstreetmap"});
}

// The dialog greys out inert presets (WMS until #118) and returns the selected
// preset's construction parameters intact.
TEST(BackgroundPersistence, DialogDisablesInertPresetsAndSelects)
{
  QSettings().clear();
  AddTileLayerDialog dialog;
  auto* list = dialog.findChild<QListWidget*>();
  ASSERT_NE(list, nullptr);
  const auto presets = builtinPresets();
  // + 1: the trailing "Custom..." row.
  ASSERT_EQ(list->count(), presets.size() + 1);
  for(int i = 0; i < presets.size(); ++i)
    EXPECT_EQ(bool(list->item(i)->flags() & Qt::ItemIsEnabled), presets[i].enabled)
        << "row " << i << " (" << presets[i].name.toStdString() << ")";

  for(int i = 0; i < presets.size(); ++i)
    if(presets[i].name == "NOAA_bluetopo")
      list->setCurrentRow(i);
  const TileLayerPreset picked = dialog.selection();
  EXPECT_EQ(picked.type, QString("wmts"));
  EXPECT_EQ(picked.layer_id, QString("bluetopo:bathymetry"));
  EXPECT_EQ(picked.tile_matrix_set, QString("EPSG:3857"));
}

int main(int argc, char** argv)
{
  // Map builds a QGraphicsScene and MapItem ctors touch qApp; offscreen keeps
  // it headless in CI. The test org/app name keeps QSettings out of the
  // developer's real camp settings.
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  QCoreApplication::setOrganizationName("camp_test");
  QCoreApplication::setApplicationName("test_background_persistence");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
