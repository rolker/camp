// [camp#168] Headless white-box test for SonarLiveCacheManager's spawned-source
// tracking. The 2026-07-23 field incident: removing the live coverage layer left
// its entry in `sources_` forever, so updateTopics() never respawned it and only
// a full camp restart brought it back. The fix clears the entry when the layer
// is destroyed. This test exercises the manager's REAL set and REAL
// `connect(destroyed -> erase)` wiring via the protected trackSpawnedLayer()
// seam (updateTopics() itself needs a live ROS Node ancestor, out of scope for a
// headless test) — deleting the connect line in trackSpawnedLayer() fails this
// test. Offscreen QApplication + Map harness as test_sonar_live_eviction;
// GL-free, ROS-free.

#include <gtest/gtest.h>

#include <string>

#include <QApplication>
#include <QCoreApplication>
#include <QObject>

#include "map/map.h"
#include "map/layer_list.h"
#include "tools/layer_manager.h"
#include "ros/live_coverage/sonar_live_cache_manager.h"

using camp::map::Map;
using camp::ros::live_coverage::SonarLiveCacheManager;

namespace
{

// Promote the protected tracking seam for white-box testing.
class TestManager: public SonarLiveCacheManager
{
public:
  using SonarLiveCacheManager::SonarLiveCacheManager;
  using SonarLiveCacheManager::trackSpawnedLayer;
};

// Build the minimal MapItem ancestry the manager requires (MapItem asserts a
// non-null parent): Map -> LayerList -> host LayerManager -> manager.
TestManager* makeManager(Map& map)
{
  auto layers = map.topLevelLayers();
  EXPECT_NE(layers, nullptr);
  auto host = new camp::tools::LayerManager(layers, "test_host_tool");
  return new TestManager(host);
}

}  // namespace

TEST(SonarLiveCacheManagerRespawn, DirectDeleteClearsTrackingAndAllowsRetrack)
{
  Map map;
  TestManager* manager = makeManager(map);

  const std::string base = "/cube_bathymetry";
  EXPECT_FALSE(manager->isSourceTracked(base));

  // The tracking contract is QObject-generic; a plain QObject stands in for the
  // spawned SonarLiveCacheLayer.
  auto layer = new QObject();
  manager->trackSpawnedLayer(base, layer);
  EXPECT_TRUE(manager->isSourceTracked(base));

  delete layer;  // ~QObject fires `destroyed` synchronously
  EXPECT_FALSE(manager->isSourceTracked(base));

  // Re-add after removal — the exact flow that failed in the field.
  auto layer2 = new QObject();
  manager->trackSpawnedLayer(base, layer2);
  EXPECT_TRUE(manager->isSourceTracked(base));
  delete layer2;
  EXPECT_FALSE(manager->isSourceTracked(base));
}

TEST(SonarLiveCacheManagerRespawn, DeleteLaterClearsTrackingOnceEventLoopDrains)
{
  Map map;
  TestManager* manager = makeManager(map);

  const std::string base = "/cube_bathymetry";
  auto layer = new QObject();
  manager->trackSpawnedLayer(base, layer);
  EXPECT_TRUE(manager->isSourceTracked(base));

  // The operator-remove path: Layer::removeFromMap() schedules deleteLater().
  // The entry legitimately lingers until the event loop drains the deferred
  // delete — then it must be gone.
  layer->deleteLater();
  EXPECT_TRUE(manager->isSourceTracked(base));
  QCoreApplication::sendPostedEvents(nullptr, QEvent::DeferredDelete);
  EXPECT_FALSE(manager->isSourceTracked(base));
}

int main(int argc, char** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  // [camp#117] Map's ctor writes the BackgroundTileLayers seed into QSettings;
  // a test org/app name keeps that out of the developer's real camp settings.
  QCoreApplication::setOrganizationName("camp_test");
  QCoreApplication::setApplicationName("test_sonar_live_cache_manager_respawn");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
