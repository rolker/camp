// [camp#102] Tests for the GGGS tile-set leaf's default-off visibility and its
// persistence round-trip (the highest-risk non-GL behavior of the lazy/async
// change). A GGGS tile-set leaf must default to NOT visible (so opening a large
// store doesn't paint — and therefore doesn't load pixels for — every tile-set),
// but a persisted on/off choice must still round-trip via QSettings.
//
// The GggsTileLayer's readSettings()/writeSettings() are protected and normally
// driven by MapItem::itemConstructed() on a deferred timer. A tiny test subclass
// exposes them so we can assert the round-trip synchronously, without an event
// loop. No GL / GDAL is needed (the layer holds no tiles here).

#include <gtest/gtest.h>

#include <QApplication>
#include <QSettings>
#include <QTemporaryDir>

#include "map/map.h"
#include "map/layer_list.h"
#include "raster/gggs_tile_layer.h"

namespace
{

// Exposes the protected settings hooks so the round-trip is testable without the
// deferred itemConstructed() timer.
class TestableGggsTileLayer: public camp::raster::GggsTileLayer
{
public:
  using camp::raster::GggsTileLayer::GggsTileLayer;
  using camp::raster::GggsTileLayer::readSettings;
  using camp::raster::GggsTileLayer::writeSettings;
};

}  // namespace

// Default (no persisted value): a GGGS tile-set leaf comes up NOT visible.
TEST(GggsVisibilityTest, DefaultsOff)
{
  QSettings().clear();
  camp::map::Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);

  // An empty directory -> a valid layer object (no tiles) with a stable itemID.
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  auto* layer = new TestableGggsTileLayer(layers, dir.path());
  layer->readSettings();   // would run via itemConstructed()'s deferred timer

  EXPECT_FALSE(layer->isVisible());
}

// A persisted visibility choice round-trips: turning a leaf ON and writing
// settings, then re-reading, restores visible == true (the default-off only
// changes the first-run default, not the persistence path).
TEST(GggsVisibilityTest, PersistedVisibilityRoundTrips)
{
  QSettings().clear();
  camp::map::Map map;
  camp::map::LayerList* layers = map.topLevelLayers();
  ASSERT_NE(layers, nullptr);

  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());

  // First layer: operator turns it ON and the choice is persisted.
  {
    auto* layer = new TestableGggsTileLayer(layers, dir.path());
    layer->readSettings();
    EXPECT_FALSE(layer->isVisible());   // default off
    layer->setVisible(true);
    layer->writeSettings();             // persist the ON choice
  }

  // A fresh layer with the SAME directory (same itemID) reads the persisted ON.
  {
    auto* layer = new TestableGggsTileLayer(layers, dir.path());
    layer->readSettings();
    EXPECT_TRUE(layer->isVisible());    // persisted choice wins over the default
  }
}

int main(int argc, char** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  QCoreApplication::setOrganizationName("camp_test");
  QCoreApplication::setApplicationName("test_gggs_visibility");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
