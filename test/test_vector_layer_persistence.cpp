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

#include <algorithm>
#include <vector>

#include <QApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSettings>
#include <QTemporaryDir>

#include "map/layer_list.h"
#include "map/map.h"
#include "vector/vector_layer.h"

using camp::vector::VectorLayer;
using camp::vector::persistedVectorLayerFiles;
using camp::vector::rebuildPersistedVectorLayerFiles;
using camp::vector::vectorLayerFilesKey;
using camp::vector::canonicalVectorLayerPath;
using camp::vector::withVectorLayerFile;
using camp::vector::withVectorLayerFilePromoted;
using camp::vector::withoutVectorLayerFile;
using camp::vector::writePersistedVectorLayerFiles;

namespace
{

// Exposes the protected settings hooks so the style round-trip can be asserted
// synchronously, without waiting for MapItem::itemConstructed's deferred
// readSettings() — the pattern test_range_persist.cpp and test_gggs_persistence.cpp
// use for the raster layers.
class TestableVectorLayer: public VectorLayer
{
public:
  using VectorLayer::VectorLayer;
  using VectorLayer::readSettings;
  using VectorLayer::writeSettings;
};

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

// [camp#22 round-5 should-fix] PER-LAYER STYLE round-trips through QSettings.
//
// The path-based settings key (camp#126) exists so a style survives reopening the
// same file, and SettingsKeyIsPathNotBasename only checks that two layers' keys
// DIFFER. A mistyped value key, a group mismatch, or a lost readSettings() call
// would leave every persisted style silently forgotten with the suite green — the
// operator sets colour-by-field once and finds it gone on the next launch.
TEST(VectorLayerPersistence, StyleRoundTripsThroughSettings)
{
  QSettings().clear();
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = dir.filePath("candidates.geojson");

  camp::map::Map map;
  auto* written = new TestableVectorLayer(map.topLevelLayers(), path);
  // The setters persist as they go (each calls writeSettings()); the explicit
  // call is what an operator-driven session ends with either way.
  written->setColorField(QStringLiteral("depth"));
  written->setSizeField(QStringLiteral("confidence"));
  written->setColormap("plasma");
  written->writeSettings();
  ASSERT_NE(written->colormap(), std::string("viridis")) << "fixture must differ from the default";

  // A second layer on the SAME FILE — what reopening is — restores all three.
  auto* restored = new TestableVectorLayer(map.topLevelLayers(), path);
  ASSERT_EQ(restored->settingsKey(), written->settingsKey())
      << "two layers on one file must share a settings key or nothing can round-trip";
  restored->readSettings();
  EXPECT_EQ(restored->colorField(), QStringLiteral("depth"));
  EXPECT_EQ(restored->sizeField(), QStringLiteral("confidence"));
  EXPECT_EQ(restored->colormap(), std::string("plasma"));

  // And a layer on a DIFFERENT file is untouched by it — the point of keying on
  // the path rather than the basename.
  auto* other = new TestableVectorLayer(map.topLevelLayers(), dir.filePath("other.geojson"));
  other->readSettings();
  EXPECT_TRUE(other->colorField().isEmpty());
  EXPECT_TRUE(other->sizeField().isEmpty());
  EXPECT_EQ(other->colormap(), std::string("viridis")) << "the default palette";

  delete other;
  delete restored;
  delete written;
}

// [camp#22 must-fix 1] A drag-reorder in the Layers tab must NOT un-persist the
// layer; only a real removal may.
//
// `Map::setMapItemParent()` implements a reorder as beginRemoveRows +
// beginInsertRows, so an owner watching the model's `rowsAboutToBeRemoved` — which
// is how this was first written — sees a reorder as a removal and deletes the file
// from `vectorLayers/files`. The layer then does not come back on the next launch,
// and nothing about the operator's gesture said "remove". The reorder-safe hook is
// `Layer::onRemovedFromMap()`, which `Layer::removeFromMap()` calls and a reorder
// never reaches — the same hook RasterLayer (camp#90) and GggsTileLayer (camp#104)
// de-persist through.
//
// The owner is `AutonomousVehicleProject`, which no test can construct (see the
// harness note at the top of this file), so its two lines of wiring — track the
// layer, and re-persist from `VectorLayer::removedFromMap` — are reproduced here
// verbatim. What is under test is the SIGNAL's discrimination: that it fires for
// `removeFromMap()` and stays silent for a reorder.
TEST(VectorLayerPersistence, ReorderKeepsFilePersistedRemovalDropsIt)
{
  QSettings().clear();
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());

  auto writeFeature = [&dir](const QString& name) -> QString
  {
    const QString path = dir.filePath(name);
    QFile file(path);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
      return QString();
    file.write(R"({"type": "FeatureCollection", "features": [
      {"type": "Feature", "geometry": {"type": "Point", "coordinates": [-70.7, 43.1]},
       "properties": {}}]})");
    file.close();
    return path;
  };
  const QString a = writeFeature("a.geojson");
  const QString b = writeFeature("b.geojson");
  ASSERT_FALSE(a.isEmpty());
  ASSERT_FALSE(b.isEmpty());

  std::vector<VectorLayer*> tracked;
  camp::map::Map map;

  // AutonomousVehicleProject::persistVectorLayers(): the single writer rebuilds
  // the whole key from the layers it tracks.
  auto persistTracked = [&tracked]()
  {
    QStringList files;
    for(const VectorLayer* layer : tracked)
      files = withVectorLayerFile(files, layer->filename());
    writePersistedVectorLayerFiles(files);
  };
  // AutonomousVehicleProject::openVectorLayer() + onVectorLayerRemoved().
  auto open = [&](const QString& filename)
  {
    auto* layer = new VectorLayer(map.topLevelLayers(), filename);
    QObject::connect(layer, &VectorLayer::removedFromMap, layer, [&tracked, &persistTracked, layer]()
    {
      tracked.erase(std::remove(tracked.begin(), tracked.end(), layer), tracked.end());
      persistTracked();
    });
    tracked.push_back(layer);
    persistTracked();
    return layer;
  };

  VectorLayer* layer_a = open(a);
  VectorLayer* layer_b = open(b);
  ASSERT_EQ(persistedVectorLayerFiles(), (QStringList{a, b}));

  // The gesture that used to lose the file: drag layer_a below layer_b. The model
  // detaches and re-inserts the row; the layer is still on the map afterwards.
  map.setMapItemParent(layer_a, map.topLevelLayers(), 2);
  EXPECT_EQ(tracked.size(), 2u) << "a reorder must not touch the bookkeeping";
  EXPECT_EQ(persistedVectorLayerFiles(), (QStringList{a, b}))
      << "a reorder must not un-persist the layer";
  EXPECT_EQ(layer_a->parentMapItem(), map.topLevelLayers()) << "the layer is still on the map";

  // A real removal still de-persists — the half that must keep working.
  layer_b->removeFromMap();
  EXPECT_EQ(tracked.size(), 1u);
  EXPECT_EQ(persistedVectorLayerFiles(), QStringList{a});

  // removeFromMap() defers the delete; let it run before the Map goes away.
  QCoreApplication::processEvents();
}

// [camp#22 must-fix 2] A layer that was UNAVAILABLE at startup and has since been
// reopened must be removable — and stay removed.
//
// The unavailable list exists so that "the network share was not mounted when CAMP
// started" does not silently delete the operator's layer: the entry is carried
// forward and written back on every rebuild. The trap is that it was written back
// FOREVER. Nothing dropped the path once the file came back, so after the operator
// reopened it and then removed it through the Layers tab, the rebuild still found
// it on the unavailable branch, re-persisted it, and the layer returned on the next
// launch — camp#90/#117's exact bug class, in the mechanism added to avoid it.
//
// The fix is in AutonomousVehicleProject::openVectorLayer(): a path confirmed to
// exist leaves the unavailable list. The sequence below is that lifecycle driven
// against the rule the project now calls.
TEST(VectorLayerPersistence, ReopenedUnavailableFileCanBeRemoved)
{
  const QString shared = QStringLiteral("/mnt/share/survey.geojson");
  const QString local = QStringLiteral("/home/op/local.geojson");

  // Launch 1: the share is not mounted. The entry is remembered, in its position.
  QStringList restoredOrder{shared, local};
  QStringList unavailable{shared};
  QStringList loaded{local};
  EXPECT_EQ(rebuildPersistedVectorLayerFiles(restoredOrder, unavailable, loaded),
            (QStringList{shared, local}))
      << "an unreachable file keeps its slot rather than being forgotten";

  // The share is mounted and the operator opens the file. openVectorLayer() drops
  // it from the unavailable list because it now exists.
  unavailable.removeAll(shared);
  loaded << shared;
  EXPECT_EQ(rebuildPersistedVectorLayerFiles(restoredOrder, unavailable, loaded),
            (QStringList{shared, local}))
      << "reopening must not move the layer";

  // Now the removal that used not to stick.
  loaded.removeAll(shared);
  EXPECT_EQ(rebuildPersistedVectorLayerFiles(restoredOrder, unavailable, loaded),
            QStringList{local})
      << "a removed layer must not be written back through the unavailable branch";

  // And it stays gone on the next launch, which restores from what was written.
  const QStringList relaunchOrder{local};
  EXPECT_EQ(rebuildPersistedVectorLayerFiles(relaunchOrder, QStringList{}, QStringList{local}),
            QStringList{local});
}

// [camp#22 round-9 should-fix] THE RESTORE'S STATE IS WHOLE BEFORE THE FIRST FILE
// IS OPENED — which is what keeps an interrupted restore from erasing the entries
// it never reached.
//
// The hazard: AutonomousVehicleProject::openVectorLayer() ends in
// persistVectorLayers(), which rewrites the whole key from the restored order,
// the unavailable list and the layers tracked so far. While those two lists were
// built INSIDE the open loop, the write at iteration k knew nothing of entries
// k+1..n — and each write is a fresh QSettings whose destructor syncs, so the
// truncated list reached disk once per restored layer. The opens are asynchronous
// parses that are still running while the loop opens later layers, so a
// worker-side abort mid restore left the operator's list permanently short: the
// camp#90/#117 class ("a layer I did not remove is gone") from the other side.
//
// planVectorLayerRestore() is the seam that removes it: the classification runs
// over the WHOLE persisted list first, opens nothing, and writes nothing. The
// project then opens the plan's files with persistence suppressed and persists
// once, after the loop. The second half of this test is the hazard itself,
// asserted so the reason the plan has to be complete cannot be quietly lost.
TEST(VectorLayerPersistence, RestorePlanIsCompleteBeforeAnyFileIsOpened)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString first = dir.filePath("first.geojson");
  const QString second = dir.filePath("second.geojson");
  const QString third = dir.filePath("third.geojson");
  const QString missing = dir.filePath("unmounted_share.geojson");
  for(const QString& path : {first, second, third})
  {
    QFile file(path);
    ASSERT_TRUE(file.open(QIODevice::WriteOnly)) << "could not create " << path.toStdString();
    file.write("{}");
    file.close();
  }
  ASSERT_FALSE(QFileInfo::exists(missing));

  const QStringList persisted{first, second, missing, third,
                              QStringLiteral("/vsicurl/https://host/remote.geojson")};
  writePersistedVectorLayerFiles(persisted);

  const camp::vector::VectorLayerRestorePlan plan =
      camp::vector::planVectorLayerRestore(persistedVectorLayerFiles());

  // Every non-/vsi entry is in the order of record, in its persisted slot — for
  // the LAST one as much as the first, which is the whole point.
  EXPECT_EQ(plan.order, (QStringList{canonicalVectorLayerPath(first),
                                     canonicalVectorLayerPath(second),
                                     canonicalVectorLayerPath(missing),
                                     canonicalVectorLayerPath(third)}));
  EXPECT_EQ(plan.unavailable, QStringList{canonicalVectorLayerPath(missing)});
  EXPECT_EQ(plan.openable, (QStringList{canonicalVectorLayerPath(first),
                                        canonicalVectorLayerPath(second),
                                        canonicalVectorLayerPath(third)}));

  // Planning is read-only: the key on disk is untouched until the restore has run
  // to completion.
  EXPECT_EQ(persistedVectorLayerFiles(), persisted)
      << "planning the restore must not write the persisted key";

  // The restore's own persist, run ONCE after the loop, reproduces the list.
  EXPECT_EQ(rebuildPersistedVectorLayerFiles(plan.order, plan.unavailable, plan.openable),
            (QStringList{canonicalVectorLayerPath(first), canonicalVectorLayerPath(second),
                         canonicalVectorLayerPath(missing), canonicalVectorLayerPath(third)}));

  // ...and the hazard it replaces. A persist that runs while the state is only
  // populated as far as entry k — which is what the per-layer persist inside the
  // open loop did — writes a list with every later entry GONE. Nothing about the
  // rebuild rule can save it: an order entry that is neither loaded nor
  // unavailable is a layer the operator REMOVED, and dropping it is the rule that
  // makes removal stick. Only a complete order, persisted once, is safe.
  const QStringList order_so_far{canonicalVectorLayerPath(first)};
  EXPECT_EQ(rebuildPersistedVectorLayerFiles(order_so_far, QStringList{}, order_so_far),
            order_so_far)
      << "the mid-loop write is exactly as lossy as it looks; the fix is not to make it";
  EXPECT_FALSE(rebuildPersistedVectorLayerFiles(order_so_far, QStringList{}, order_so_far)
                   .contains(canonicalVectorLayerPath(third)));

  QSettings().remove(vectorLayerFilesKey());
}

// The counterpart that must keep working: an entry still unavailable at removal
// time has no layer to remove, so it is carried forward — the deliberate
// consequence documented on restorePersistedVectorLayers().
TEST(VectorLayerPersistence, StillMissingFileIsCarriedForward)
{
  const QString shared = QStringLiteral("/mnt/share/survey.geojson");
  const QString local = QStringLiteral("/home/op/local.geojson");
  EXPECT_EQ(rebuildPersistedVectorLayerFiles(QStringList{shared, local},
                                             QStringList{shared},
                                             QStringList{local}),
            (QStringList{shared, local}));
}

// [camp#22 round-3 must-fix] The same bug through the one spelling that does not
// canonicalise: a DANGLING SYMLINK.
//
// canonicalFilePath() is empty for a path that does not resolve, so the entry is
// remembered under its RAW spelling. When the target appears and the operator
// opens that same symlink, the path in hand is the resolved TARGET — an exact
// `removeAll(fname)` misses the raw entry, the rebuild keeps finding it on the
// unavailable branch and writes it back, and removing the reopened layer does not
// stick. withoutVectorLayerFile() drops by canonical-equivalent identity, which
// is what openVectorLayer() now calls.
TEST(VectorLayerPersistence, ReopenedDanglingSymlinkCanBeRemoved)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString target = dir.filePath("survey.geojson");
  const QString link = dir.filePath("latest.geojson");
  ASSERT_TRUE(QFile::link(target, link)) << "could not create the symlink fixture";

  // Launch 1: the target does not exist yet, so the link dangles and the entry is
  // remembered under the raw spelling of the link.
  ASSERT_FALSE(QFileInfo::exists(link));
  const QString stored = canonicalVectorLayerPath(link);
  EXPECT_EQ(stored, link) << "a dangling symlink cannot canonicalise; the raw path is the identity";
  QStringList unavailable{stored};

  // The target appears. Opening the SAME link now yields the resolved target.
  {
    QFile file(target);
    ASSERT_TRUE(file.open(QIODevice::WriteOnly));
    file.write("{}");
  }
  const QString opened = canonicalVectorLayerPath(link);
  EXPECT_NE(opened, stored) << "the link now resolves, so the identity changed spelling";

  // The purge openVectorLayer() performs.
  unavailable = withoutVectorLayerFile(unavailable, opened);
  EXPECT_TRUE(unavailable.isEmpty())
      << "the raw-spelling entry names the same file and must be dropped";

  // With the list current, removing the reopened layer sticks.
  const QStringList restoredOrder{stored};
  EXPECT_EQ(rebuildPersistedVectorLayerFiles(restoredOrder, unavailable, QStringList{}),
            QStringList{})
      << "a removed layer must not return through the unavailable branch";
}

// [camp#22 round-5 should-fix] The reopened dangling symlink keeps its SLOT, not
// just its removability.
//
// The sibling of the test above, one step further on: the order of record holds
// the RAW spelling the link was remembered under, while the reopened layer is
// tracked under the resolved TARGET. rebuildPersistedVectorLayerFiles() matches
// order against loaded filenames by exact string, so without the promotion the
// raw entry misses, its slot is skipped, and the trailing append loop puts the
// reopened layer LAST — [link, B] persists as [B, target]. Layer order is the
// operator's stacking order, so that is a silent reshuffle of their map.
TEST(VectorLayerPersistence, ReopenedDanglingSymlinkKeepsItsSlot)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString target = dir.filePath("survey.geojson");
  const QString link = dir.filePath("latest.geojson");
  const QString other = dir.filePath("other.geojson");
  ASSERT_TRUE(QFile::link(target, link)) << "could not create the symlink fixture";
  for(const QString& path : {other})
  {
    QFile file(path);
    ASSERT_TRUE(file.open(QIODevice::WriteOnly));
    file.write("{}");
  }

  // Launch 1: the link dangles. It is remembered, FIRST, under its raw spelling;
  // the other file loads.
  ASSERT_FALSE(QFileInfo::exists(link));
  const QString stored = canonicalVectorLayerPath(link);
  ASSERT_EQ(stored, link);
  QStringList restoredOrder{stored, canonicalVectorLayerPath(other)};
  QStringList unavailable{stored};
  QStringList loaded{canonicalVectorLayerPath(other)};
  EXPECT_EQ(rebuildPersistedVectorLayerFiles(restoredOrder, unavailable, loaded),
            (QStringList{stored, canonicalVectorLayerPath(other)}));

  // The target appears and the operator opens the same link: the path in hand is
  // now the resolved target.
  {
    QFile file(target);
    ASSERT_TRUE(file.open(QIODevice::WriteOnly));
    file.write("{}");
  }
  const QString opened = canonicalVectorLayerPath(link);
  ASSERT_NE(opened, stored);

  // The two steps openVectorLayer() performs on a promotion.
  unavailable = withoutVectorLayerFile(unavailable, opened);
  restoredOrder = withVectorLayerFilePromoted(restoredOrder, opened);
  loaded << opened;

  EXPECT_EQ(rebuildPersistedVectorLayerFiles(restoredOrder, unavailable, loaded),
            (QStringList{opened, canonicalVectorLayerPath(other)}))
      << "the reopened layer must keep the slot it was persisted in, not move to the end";
}

// The promotion must leave every OTHER entry exactly where it was, and must not
// invent an entry for a file that was never in the order.
TEST(VectorLayerPersistence, PromotionRewritesOnlyTheMatchingEntry)
{
  const QString shared = QStringLiteral("/mnt/share/survey.geojson");
  const QString local = QStringLiteral("/home/op/local.geojson");
  EXPECT_EQ(withVectorLayerFilePromoted(QStringList{shared, local}, shared),
            (QStringList{shared, local}))
      << "an entry already stored under its own identity is untouched";
  EXPECT_EQ(withVectorLayerFilePromoted(QStringList{shared, local},
                                        QStringLiteral("/other.gpkg")),
            (QStringList{shared, local}))
      << "an unrelated path must not disturb the list";
  EXPECT_TRUE(withVectorLayerFilePromoted(QStringList{}, shared).isEmpty())
      << "promotion adds nothing; it only rewrites what is already there";
  // [camp#22 round-8 suggestion] The resolve stops at the matching entry, but the
  // exact-string comparison does not: an identical duplicate anywhere in the list
  // is still collapsed, and the entries around it keep their order.
  EXPECT_EQ(withVectorLayerFilePromoted(QStringList{shared, local, shared}, shared),
            (QStringList{shared, local}))
      << "an exact duplicate must still collapse into the promoted slot";
}

// The exact-match removal withoutVectorLayerFile() also has to keep doing, and the
// entries it must LEAVE ALONE.
TEST(VectorLayerPersistence, WithoutVectorLayerFileKeepsUnrelatedEntries)
{
  const QString shared = QStringLiteral("/mnt/share/survey.geojson");
  const QString local = QStringLiteral("/home/op/local.geojson");
  EXPECT_EQ(withoutVectorLayerFile(QStringList{shared, local}, shared), QStringList{local});
  EXPECT_EQ(withoutVectorLayerFile(QStringList{shared, local}, QStringLiteral("/other.gpkg")),
            (QStringList{shared, local}))
      << "an unrelated path must not disturb the list";
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
