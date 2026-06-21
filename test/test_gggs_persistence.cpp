// [camp#104] Persistence round-trip for the flat-layer reset (ADR-0005).
//
// Persistence moved off store roots (GggsStores/roots, the retired nested
// GggsStoreLayer) and onto the selected flat layers (GggsTileLayers/dirs). These
// tests pin:
//   - select -> the tile-set dir is persisted (dir-unique);
//   - restore -> Map construction's createDefaultLayers() recreates a flat
//     GggsTileLayer per still-existing dir, deduped (no duplicate on a repeated
//     dir);
//   - remove -> the dir is dropped from the key;
//   - the old GggsStores/roots key is ignored and cleared once on startup.
//
// Empty tile-set directories keep this GL/GDAL-free.

#include <gtest/gtest.h>

#include <QApplication>
#include <QSettings>
#include <QTemporaryDir>

#include "map/map.h"
#include "map/layer.h"
#include "map/layer_list.h"
#include "map/map_item.h"
#include "raster/gggs_store_source.h"
#include "raster/gggs_tile_layer.h"

using camp::map::Map;
using camp::map::MapItem;
using camp::raster::GggsStoreSource;
using camp::raster::GggsTileLayer;

namespace
{

int gggsLayersOn(camp::map::LayerList* layers, const QString& dir)
{
  int n = 0;
  for(MapItem* child : layers->childMapItems())
    if(auto* g = dynamic_cast<GggsTileLayer*>(child))
      if(g->directory() == dir)
        ++n;
  return n;
}

}  // namespace

// Selecting a tile-set persists its directory under GggsTileLayers/dirs (once).
TEST(GggsPersistence, SelectPersistsDir)
{
  QSettings().clear();
  Map map;
  QTemporaryDir tileset;
  ASSERT_TRUE(tileset.isValid());

  GggsStoreSource source;
  source.instantiate(map.topLevelLayers(), tileset.path());

  QStringList dirs = QSettings().value("GggsTileLayers/dirs").toStringList();
  EXPECT_EQ(dirs, QStringList{tileset.path()});

  // Re-select: still persisted exactly once (dir-unique).
  source.instantiate(map.topLevelLayers(), tileset.path());
  EXPECT_EQ(QSettings().value("GggsTileLayers/dirs").toStringList(),
            QStringList{tileset.path()});
}

// createDefaultLayers() (run by Map construction) restores a flat GggsTileLayer
// for each persisted dir, deduping a repeated dir to a single layer.
TEST(GggsPersistence, RestoreRecreatesFlatLayerDeduped)
{
  QSettings().clear();
  QTemporaryDir tileset;
  ASSERT_TRUE(tileset.isValid());

  // A duplicate entry must still restore exactly one layer.
  QSettings().setValue("GggsTileLayers/dirs",
                       QStringList{tileset.path(), tileset.path()});

  Map map;   // ctor -> BackgroundManager::createDefaultLayers() restore path
  EXPECT_EQ(gggsLayersOn(map.topLevelLayers(), tileset.path()), 1);
}

// A persisted dir that no longer exists on disk is skipped on restore.
TEST(GggsPersistence, RestoreSkipsMissingDir)
{
  QSettings().clear();
  QString gone;
  {
    QTemporaryDir tileset;
    ASSERT_TRUE(tileset.isValid());
    gone = tileset.path();
  }   // tileset removed from disk here
  QSettings().setValue("GggsTileLayers/dirs", QStringList{gone});

  Map map;
  EXPECT_EQ(gggsLayersOn(map.topLevelLayers(), gone), 0);
}

// Removing a flat layer drops its dir from GggsTileLayers/dirs.
TEST(GggsPersistence, RemoveDePersistsDir)
{
  QSettings().clear();
  Map map;
  QTemporaryDir tileset;
  ASSERT_TRUE(tileset.isValid());

  GggsStoreSource source;
  camp::map::Layer* layer = source.instantiate(map.topLevelLayers(), tileset.path());
  ASSERT_NE(layer, nullptr);
  ASSERT_EQ(QSettings().value("GggsTileLayers/dirs").toStringList(),
            QStringList{tileset.path()});

  // removeFromMap() calls onRemovedFromMap() synchronously before scheduling the
  // delete, so the de-persist is observable immediately.
  layer->removeFromMap();
  EXPECT_TRUE(QSettings().value("GggsTileLayers/dirs").toStringList().isEmpty());
}

// The retired GggsStores/roots key is ignored (no nested store revived) and is
// cleared once on startup so a stale value can't resurface.
TEST(GggsPersistence, OldRootsKeyIgnoredAndCleared)
{
  QSettings().clear();
  QTemporaryDir old_store;
  ASSERT_TRUE(old_store.isValid());
  QSettings().setValue("GggsStores/roots", QStringList{old_store.path()});

  Map map;   // must not read the old key, and clears it

  EXPECT_TRUE(QSettings().value("GggsStores/roots").toStringList().isEmpty())
      << "old store-roots key must be cleared on startup";
  // No flat layers either (we only set the retired key).
  EXPECT_EQ(gggsLayersOn(map.topLevelLayers(), old_store.path()), 0);
}

int main(int argc, char** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  QCoreApplication::setOrganizationName("camp_test");
  QCoreApplication::setApplicationName("test_gggs_persistence");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
