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
// Empty tile-set directories keep the dir-persistence tests GL/GDAL-free.
//
// [camp#108] The selected band also round-trips: writeSettings() persists the
// 1-indexed band and readSettings() restores it via applyBand(). The band tests
// below mirror the visibility round-trip pattern (test_gggs_visibility.cpp) — a
// tiny subclass exposes the protected settings hooks so the round-trip is
// asserted synchronously, without the deferred itemConstructed() timer — and use
// the 2-band synthetic GeoTIFF helper from test_gggs_band_select.cpp so the layer
// reports bandCount() == 2 and setBand(2) is in range. The band INTEGER round-trip
// (setBand -> writeSettings -> readSettings -> band()) needs no GL: applyBand()
// sets band_ before any GL work and skips textures when no context exists, so
// these tests RUN — not SKIP — in-container.

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include <gdal_priv.h>

#include <QApplication>
#include <QDir>
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

// [camp#108] Exposes the protected settings hooks so the band round-trip is
// testable without the deferred itemConstructed() timer (mirrors
// test_gggs_visibility.cpp's TestableGggsTileLayer).
class TestableGggsTileLayer: public GggsTileLayer
{
public:
  using GggsTileLayer::GggsTileLayer;
  using GggsTileLayer::readSettings;
  using GggsTileLayer::writeSettings;
};

// [camp#108] Write a 2-band UInt16 north-up GeoTIFF at @p path so a layer over its
// directory reports bandCount() == 2 and setBand(2) is in range (adapted from the
// test_gggs_band_select.cpp helper; disjoint per-band ranges, NoData = 0 on each).
QString writeTwoBandTile(const QString& path,
                         int w, int h, const double geo[6],
                         const std::vector<uint16_t>& band1,
                         const std::vector<uint16_t>& band2)
{
  if(GDALGetDriverCount() == 0)
    GDALAllRegister();
  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GTiff");
  GDALDataset* ds = driver->Create(path.toUtf8().constData(), w, h, 2, GDT_UInt16, nullptr);
  ds->SetGeoTransform(const_cast<double*>(geo));
  GDALRasterBand* b1 = ds->GetRasterBand(1);
  b1->SetNoDataValue(0);
  CPLErr err = b1->RasterIO(GF_Write, 0, 0, w, h,
                            const_cast<uint16_t*>(band1.data()), w, h, GDT_UInt16, 0, 0);
  if(err == CE_None)
  {
    GDALRasterBand* b2 = ds->GetRasterBand(2);
    b2->SetNoDataValue(0);
    err = b2->RasterIO(GF_Write, 0, 0, w, h,
                       const_cast<uint16_t*>(band2.data()), w, h, GDT_UInt16, 0, 0);
  }
  GDALClose(ds);
  return err == CE_None ? path : QString();
}

// [camp#108] A small 2-band tile-set in the directory @p dirPath; the layer over it
// reports bandCount() == 2. Disjoint per-band ranges as in test_gggs_band_select.cpp.
bool writeTwoBandTileSet(const QString& dirPath)
{
  const int w = 8, h = 8;
  const double geo[6] = {-71.40, 0.0001, 0.0, 43.00, 0.0, -0.0001};
  std::vector<uint16_t> band1(w * h), band2(w * h);
  for(int r = 0; r < h; ++r)
    for(int c = 0; c < w; ++c)
    {
      band1[r * w + c] = uint16_t(10 + (c % 4) * 10);   // [10, 40]
      band2[r * w + c] = uint16_t(1000 + r * 400);      // [1000, 3800]
    }
  return !writeTwoBandTile(QDir(dirPath).filePath("13_0_0.tif"),
                           w, h, geo, band1, band2).isEmpty();
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

// [camp#108] The selected band round-trips through QSettings: a layer over a
// 2-band tile-set switched to band 2 (public setBand) persists it via
// writeSettings(); a fresh layer over the SAME directory (same itemID) restores
// band 2 via readSettings(). A regression that silently dropped the band back to 1
// on reload would pass every other persistence test — this is the one that catches
// it. GL-free: applyBand() sets band_ before any texture work, so band() is
// observable in-container without an offscreen GL context.
TEST(GggsPersistence, BandRoundTrips)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);

  QTemporaryDir tileset;
  ASSERT_TRUE(tileset.isValid());
  ASSERT_TRUE(writeTwoBandTileSet(tileset.path()));

  // First layer: defaults to band 1, operator switches to band 2 (which persists).
  {
    auto* layer = new TestableGggsTileLayer(layers, tileset.path());
    ASSERT_TRUE(layer->valid());
    ASSERT_EQ(layer->bandCount(), 2);
    EXPECT_EQ(layer->band(), 1);          // default before any selection
    layer->setBand(2);
    EXPECT_EQ(layer->band(), 2);
    layer->writeSettings();               // persist the band-2 choice
  }

  // A fresh layer over the SAME directory restores the persisted band 2.
  {
    auto* layer = new TestableGggsTileLayer(layers, tileset.path());
    ASSERT_EQ(layer->bandCount(), 2);
    layer->readSettings();
    EXPECT_EQ(layer->band(), 2)
        << "restored band must match the persisted selection, not fall back to 1";
  }
}

// [camp#108] The DEFAULT round-trips too: a layer that never sets a band persists
// and restores band 1 — no spurious change. Sanity: a fresh layer reading a
// cleared settings group also defaults to band 1. No tiles needed (band_ defaults
// to 1 regardless), so this stays GL/GDAL-free like the dir-persistence tests.
TEST(GggsPersistence, BandDefaultRoundTrips)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);

  QTemporaryDir tileset;
  ASSERT_TRUE(tileset.isValid());

  // A layer that never touches the band persists the default.
  {
    auto* layer = new TestableGggsTileLayer(layers, tileset.path());
    EXPECT_EQ(layer->band(), 1);
    layer->writeSettings();
  }

  // A fresh layer restores band 1 (persisted default, not a spurious change).
  {
    auto* layer = new TestableGggsTileLayer(layers, tileset.path());
    layer->readSettings();
    EXPECT_EQ(layer->band(), 1);
  }

  // Sanity: reading an empty (cleared) group yields the default band 1.
  QSettings().clear();
  {
    auto* layer = new TestableGggsTileLayer(layers, tileset.path());
    layer->readSettings();
    EXPECT_EQ(layer->band(), 1);
  }
}

// [camp#126] Two stores under DIFFERENT roots that share BOTH path components
// (survey_a/bathymetry/processed and survey_b/bathymetry/processed) resolve to the
// SAME parent/leaf display name — and thus, under a shared parent LayerList, the
// SAME itemID(). Per-layer prefs used to key on itemID(), so the two would collide
// on ONE QSettings group and clobber each other's visible/colormap/band. Now they
// key on settingsKey() (the directory), so each persists independently. This test
// pins the clash is fixed: set distinct visible/colormap/band on one and confirm
// the other is unaffected. GL-free (band_ is set before any texture work; empty/
// loaded tiles need no offscreen GL for the integer round-trip), GDAL-only.
TEST(GggsPersistence, SameDisplayNameDistinctPersistence)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);

  QTemporaryDir baseA, baseB;
  ASSERT_TRUE(baseA.isValid());
  ASSERT_TRUE(baseB.isValid());
  const QString dirA = baseA.filePath("bathymetry/processed");
  const QString dirB = baseB.filePath("bathymetry/processed");
  ASSERT_TRUE(QDir().mkpath(dirA));
  ASSERT_TRUE(QDir().mkpath(dirB));
  ASSERT_TRUE(writeTwoBandTileSet(dirA));
  ASSERT_TRUE(writeTwoBandTileSet(dirB));

  // Same display name + same itemID, but DISTINCT settings keys — the crux of the
  // fix. The old itemID()-keyed group would have been shared between the two.
  {
    auto* a = new TestableGggsTileLayer(layers, dirA);
    auto* b = new TestableGggsTileLayer(layers, dirB);
    ASSERT_EQ(a->objectName(), QString("bathymetry/processed"));
    ASSERT_EQ(a->objectName(), b->objectName());
    ASSERT_EQ(a->itemID(), b->itemID());
    EXPECT_NE(a->settingsKey(), b->settingsKey());

    // Give A non-default visible/colormap/band; leave B at its defaults; persist.
    ASSERT_EQ(a->bandCount(), 2);
    a->setVisible(true);
    a->setColormap(camp::map::ColorMap::Viridis);
    a->setBand(2);
    a->writeSettings();

    b->setVisible(false);
    b->writeSettings();   // B keeps its defaults (band 1)
  }

  // Fresh layers re-read their OWN groups: A's changes did not bleed into B.
  {
    auto* a = new TestableGggsTileLayer(layers, dirA);
    auto* b = new TestableGggsTileLayer(layers, dirB);
    a->readSettings();
    b->readSettings();

    EXPECT_TRUE(a->isVisible());
    EXPECT_EQ(a->band(), 2);

    EXPECT_FALSE(b->isVisible()) << "B's visibility must be independent of A's";
    EXPECT_EQ(b->band(), 1) << "B's band must be independent of A's";
  }
}

// [camp#126] Directory identity is normalized to ONE canonical absolute form, so
// the three authorities that treat the directory as identity agree even when the
// caller passes a non-canonical string variant (a trailing slash here). Without
// canonicalization, GggsStoreSource::instantiate() dedups on the raw string while
// settingsKey() normalizes via QDir::absolutePath() — so a trailing-slash variant
// would dedup as a DISTINCT layer yet collide on ONE settings group. This pins:
// (a) directory()/settingsKey() of a trailing-slash layer match the canonical
// form; (b) instantiate() dedups the variant against the canonical layer (one
// layer, not two); (c) the persisted dirs list stores the canonical path.
// GL/GDAL-free (empty tile-set dir).
TEST(GggsPersistence, NonCanonicalDirNormalizes)
{
  QSettings().clear();
  Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);

  QTemporaryDir tileset;
  ASSERT_TRUE(tileset.isValid());
  const QString canonical = QDir(tileset.path()).absolutePath();
  const QString variant = canonical + '/';   // non-canonical (trailing slash)
  ASSERT_NE(variant, canonical);

  // (a) A layer built from the trailing-slash variant normalizes directory_ to the
  // canonical form, so directory() and settingsKey() match a canonical-form layer.
  {
    GggsTileLayer canonLayer(layers, canonical);
    GggsTileLayer variantLayer(layers, variant);
    EXPECT_EQ(variantLayer.directory(), canonical);
    EXPECT_EQ(variantLayer.directory(), canonLayer.directory());
    EXPECT_EQ(variantLayer.settingsKey(), canonLayer.settingsKey());
  }

  // (b) instantiate() with the variant then the canonical form yields ONE deduped
  // layer (the second call returns the existing layer, not a duplicate), and (c)
  // the persisted restore list holds exactly the canonical path.
  GggsStoreSource source;
  camp::map::Layer* first = source.instantiate(layers, variant);
  ASSERT_NE(first, nullptr);
  camp::map::Layer* second = source.instantiate(layers, canonical);
  EXPECT_EQ(first, second) << "canonical re-select must dedup against the variant";
  EXPECT_EQ(gggsLayersOn(layers, canonical), 1);
  EXPECT_EQ(QSettings().value("GggsTileLayers/dirs").toStringList(),
            QStringList{canonical})
      << "persisted dir must be the canonical path, not the trailing-slash variant";
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
