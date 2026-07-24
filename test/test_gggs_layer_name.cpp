// [camp#126] A flat store layer's tree-view name is the last two path components
// ("parent/leaf"), so two stores that share a leaf (.../sidescan/processed and
// .../bathymetry/processed) stay distinguishable. The name is the layer's
// QObject name (set in the GggsTileLayer ctor). These tests pin the DISPLAY name
// only. Per-layer persistence (visible/colormap/band) is keyed on the DIRECTORY
// via GggsTileLayer::settingsKey() — not on this display name, which two distinct
// stores can share — so the display label and the persistence key are decoupled;
// that directory-keyed independence is covered in test_gggs_persistence.cpp.
//
// GL/GDAL-free: an empty (or non-existent) tile-set directory yields no tiles, so
// the ctor takes the "(no tiles)" branch and never touches the offscreen GL path.
// These tests therefore RUN — not SKIP — in-container.

#include <gtest/gtest.h>

#include <QApplication>
#include <QDir>
#include <QSettings>
#include <QTemporaryDir>

#include "map/map.h"
#include "map/layer_list.h"
#include "raster/gggs_tile_layer.h"

using camp::raster::GggsTileLayer;

namespace
{

// A nested store path "<tmp>/stores/sidescan/processed" is named by its last two
// components: "sidescan/processed".
TEST(GggsLayerName, ParentSlashLeaf)
{
  QSettings().clear();
  camp::map::Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);

  QTemporaryDir base;
  ASSERT_TRUE(base.isValid());
  const QString dir = base.filePath("stores/sidescan/processed");
  ASSERT_TRUE(QDir().mkpath(dir));               // empty -> no tiles, GL-free ctor

  auto* layer = new GggsTileLayer(layers, dir);
  EXPECT_EQ(layer->objectName(), QString("sidescan/processed"));
}

// A root-level directory has no parent component, so the name falls back to just
// the leaf. The directory need not exist (an absent dir lists no tiles).
TEST(GggsLayerName, BareLeafFallback)
{
  QSettings().clear();
  camp::map::Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);

  auto* layer = new GggsTileLayer(layers, "/processed");
  EXPECT_EQ(layer->objectName(), QString("processed"));
}

}  // namespace

int main(int argc, char** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  // [camp#117] Map's ctor now writes the BackgroundTileLayers seed into QSettings;
  // a test org/app name keeps that out of the developer's real camp settings.
  QCoreApplication::setOrganizationName("camp_test");
  QCoreApplication::setApplicationName("test_gggs_layer_name");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
