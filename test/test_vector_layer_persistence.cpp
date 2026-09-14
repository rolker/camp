// [camp#22 / ADR-0003 §4] Persistence of the read-only vector-layer list.
//
// A Layers-tab layer is app state, not mission data: the operator expects a
// vector layer back when CAMP reopens, whether or not a mission file is loaded —
// the same contract charts have under `backgrounds/files`. The half that is easy
// to forget and loud when missing is REMOVAL: a layer the operator removed from
// the Layers tab must stay removed, or it returns on the next launch and they
// remove it again, forever (camp#90/#117 are exactly that bug for charts and
// tile layers).
//
// Harness note: the list is owned by AutonomousVehicleProject::persistVectorLayers(),
// which no test can construct — its translation unit pulls in the whole mission
// tree plus the platform and mission managers. The mechanism it writes through
// lives in camp_map (vector_layer.h) precisely so the add / de-dup / remove rules
// and the settings round trip can be exercised; the project's use of them is one
// call each on the add and remove paths, verified by reading the source. This is
// the same seam decision as camp::mission::resolveInsertionParent for the
// insertion-parent fix in this issue.

#include <gtest/gtest.h>

#include <QApplication>
#include <QSettings>
#include <QTemporaryDir>

#include "map/layer_list.h"
#include "map/map.h"
#include "vector/vector_layer.h"

using camp::vector::VectorLayer;
using camp::vector::persistedVectorLayerFiles;
using camp::vector::vectorLayerFilesKey;
using camp::vector::withVectorLayerFile;
using camp::vector::withoutVectorLayerFile;
using camp::vector::writePersistedVectorLayerFiles;

namespace
{

// What AutonomousVehicleProject::persistVectorLayers() does: rebuild the whole
// key from the layers it tracks, de-duped, in order. Written here as the tracked
// list so the add/remove sequence can be driven without the project.
void persist(const QStringList& tracked)
{
  QStringList files;
  for(const QString& file : tracked)
    files = withVectorLayerFile(files, file);
  writePersistedVectorLayerFiles(files);
}

}  // namespace

// Adding is append-if-absent: re-adding a loaded file cannot stack a duplicate,
// which is what keeps a restore-then-open sequence from growing the list every
// launch.
TEST(VectorLayerPersistence, AddDeDupsByFilename)
{
  QStringList files;
  files = withVectorLayerFile(files, "/data/a.geojson");
  files = withVectorLayerFile(files, "/data/b.geojson");
  files = withVectorLayerFile(files, "/data/a.geojson");
  EXPECT_EQ(files, (QStringList{"/data/a.geojson", "/data/b.geojson"}));

  // A list that already contains duplicates (written by an older build) collapses
  // on the next write rather than being preserved.
  const QStringList dirty{"/data/a.geojson", "/data/a.geojson", "/data/b.geojson"};
  EXPECT_EQ(withVectorLayerFile(dirty, "/data/b.geojson"),
            (QStringList{"/data/a.geojson", "/data/b.geojson"}));
}

// Removal drops every entry for the file — not just the first.
TEST(VectorLayerPersistence, RemoveDropsEveryEntry)
{
  const QStringList files{"/data/a.geojson", "/data/b.geojson", "/data/a.geojson"};
  EXPECT_EQ(withoutVectorLayerFile(files, "/data/a.geojson"), QStringList{"/data/b.geojson"});
  EXPECT_TRUE(withoutVectorLayerFile(QStringList{"/data/a.geojson"}, "/data/a.geojson").isEmpty());
}

// The full operator sequence: add two, "restart" (re-read the key), remove one,
// "restart" again — the removed one stays gone and the other is still there.
TEST(VectorLayerPersistence, RemoveDePersistsAndSticks)
{
  QSettings().clear();

  QStringList tracked{"/data/a.geojson", "/data/b.geojson"};
  persist(tracked);
  EXPECT_EQ(persistedVectorLayerFiles(), tracked);

  // Restart: the restore path reads the key and rebuilds its tracking from it.
  QStringList restored = persistedVectorLayerFiles();
  EXPECT_EQ(restored.size(), 2);

  // The operator removes one through the Layers tab; the project drops its entry
  // and re-persists the whole list.
  restored.removeAll("/data/a.geojson");
  persist(restored);
  EXPECT_EQ(persistedVectorLayerFiles(), QStringList{"/data/b.geojson"});

  // Second restart: still gone. (The bug this guards against restores it.)
  EXPECT_FALSE(persistedVectorLayerFiles().contains("/data/a.geojson"));
  EXPECT_TRUE(persistedVectorLayerFiles().contains("/data/b.geojson"));
}

// The key is the documented one and round-trips through QSettings.
TEST(VectorLayerPersistence, KeyRoundTrips)
{
  QSettings().clear();
  EXPECT_EQ(vectorLayerFilesKey(), QStringLiteral("vectorLayers/files"));
  EXPECT_TRUE(persistedVectorLayerFiles().isEmpty());
  writePersistedVectorLayerFiles(QStringList{"/data/c.geojson"});
  EXPECT_EQ(QSettings().value(vectorLayerFilesKey()).toStringList(),
            QStringList{"/data/c.geojson"});
}

// Per-layer style is keyed by the FULL PATH, not the basename: two files named
// the same in different directories keep separate styles (camp#126's collision).
TEST(VectorLayerPersistence, SettingsKeyIsPathNotBasename)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  ASSERT_TRUE(QDir(dir.path()).mkpath("survey_a"));
  ASSERT_TRUE(QDir(dir.path()).mkpath("survey_b"));
  const QString a = dir.filePath("survey_a/candidates.geojson");
  const QString b = dir.filePath("survey_b/candidates.geojson");

  camp::map::Map map;
  auto* layer_a = new VectorLayer(map.topLevelLayers(), a);
  auto* layer_b = new VectorLayer(map.topLevelLayers(), b);

  // Same display name (the basename) — different settings groups.
  EXPECT_EQ(layer_a->objectName(), layer_b->objectName());
  EXPECT_NE(layer_a->settingsKey(), layer_b->settingsKey());

  delete layer_a;
  delete layer_b;
}

int main(int argc, char** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  QCoreApplication::setOrganizationName("camp_test");
  QCoreApplication::setApplicationName("test_vector_layer_persistence");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
