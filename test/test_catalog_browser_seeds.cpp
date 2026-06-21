// [camp#104] Browse-seed persistence for catalog::CatalogBrowser (ADR-0005,
// browser-seed amendment). Orthogonal to the flat-layer persistence reset: these
// tests pin only that the *browsed store root(s)* survive a restart so the Stores
// tab repopulates its tree — they spawn no layers and touch no GggsTileLayers/dirs.
//
// Pinned behavior:
//   - seedRoot() persists the (sourceId, root) seed under CatalogBrowser/seeds,
//     and a fresh browser + restoreSeeds() rebuilds the top-level node;
//   - restoreSeeds() skips a seed whose root no longer exists (no crash);
//   - seeding the same root twice yields one top-level node / one persisted seed;
//   - removeSeed() de-persists, so a fresh restore yields no node;
//   - removeSeed() of a *middle* row shifts the model rows and the parallel seeds_
//     identically, so the survivors stay matched to the correct roots;
//   - restoreSeeds() skips a malformed (no-delimiter) seed without crashing.
//
// Discovery only stats directories for `*.tif` filenames, so an empty placeholder
// tile keeps this widget/model-level and GL/GDAL-free. QSettings is scoped to a
// test org/app in main() so it never touches the real config.

#include <gtest/gtest.h>

#include <memory>

#include <QAbstractItemModel>
#include <QApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSettings>
#include <QTemporaryDir>
#include <QTreeView>

#include "catalog/catalog_browser.h"
#include "catalog/catalog_source.h"
#include "raster/gggs_store_source.h"

using camp::catalog::CatalogBrowser;
using camp::catalog::CatalogSource;
using camp::raster::GggsStoreSource;

namespace
{

// Create an empty placeholder tile `<dir>/<name>` — discovery matches only the
// `*.tif` filename, so the content is irrelevant (no GL/GDAL).
void touchTif(const QString& dir, const QString& name)
{
  QDir().mkpath(dir);
  QFile f(QDir(dir).filePath(name));
  ASSERT_TRUE(f.open(QIODevice::WriteOnly));
  f.close();
}

// Register a GggsStoreSource on @p browser and return a borrowed pointer (the
// browser owns it) so a test can drive seedRoot() directly.
GggsStoreSource* addGggsSource(CatalogBrowser& browser)
{
  auto source = std::make_unique<GggsStoreSource>();
  GggsStoreSource* borrowed = source.get();
  browser.addSource(std::move(source));
  return borrowed;
}

}  // namespace

// seedRoot() persists the seed; a fresh browser + restoreSeeds() rebuilds it.
TEST(CatalogBrowserSeeds, PersistRoundTrip)
{
  QSettings().clear();
  QTemporaryDir store;
  ASSERT_TRUE(store.isValid());
  touchTif(store.path() + "/bathymetry/tileset", "0_0_0.tif");

  {
    CatalogBrowser browser;
    GggsStoreSource* source = addGggsSource(browser);
    EXPECT_TRUE(browser.seedRoot(source, store.path()));
    EXPECT_EQ(browser.topLevelCount(), 1);
  }

  const QStringList seeds = QSettings().value("CatalogBrowser/seeds").toStringList();
  ASSERT_EQ(seeds.size(), 1);
  EXPECT_EQ(seeds.first(), QStringLiteral("gggs\t") + store.path());

  CatalogBrowser restored;
  addGggsSource(restored);
  restored.restoreSeeds();
  EXPECT_EQ(restored.topLevelCount(), 1)
      << "restoreSeeds() must rebuild the browsed root's top-level node";
}

// A persisted root that no longer exists is skipped on restore (no node, no crash).
TEST(CatalogBrowserSeeds, RestoreSkipsMissingRoot)
{
  QSettings().clear();
  QSettings().setValue("CatalogBrowser/seeds",
                       QStringList{QStringLiteral("gggs\t/no/such/store/root")});

  CatalogBrowser browser;
  addGggsSource(browser);
  browser.restoreSeeds();   // must not crash
  EXPECT_EQ(browser.topLevelCount(), 0);
}

// Seeding the same root twice yields one top-level node and one persisted seed.
TEST(CatalogBrowserSeeds, SeedDedup)
{
  QSettings().clear();
  QTemporaryDir store;
  ASSERT_TRUE(store.isValid());
  touchTif(store.path() + "/sidescan/tileset", "0_0_0.tif");

  CatalogBrowser browser;
  GggsStoreSource* source = addGggsSource(browser);
  EXPECT_TRUE(browser.seedRoot(source, store.path()));
  EXPECT_TRUE(browser.seedRoot(source, store.path()));   // dedup: re-select, not re-add

  EXPECT_EQ(browser.topLevelCount(), 1);
  EXPECT_EQ(QSettings().value("CatalogBrowser/seeds").toStringList().size(), 1);
}

// removeSeed() drops the model row and de-persists; a fresh restore yields nothing.
TEST(CatalogBrowserSeeds, RemoveDePersists)
{
  QSettings().clear();
  QTemporaryDir store;
  ASSERT_TRUE(store.isValid());
  touchTif(store.path() + "/bathymetry/tileset", "0_0_0.tif");

  {
    CatalogBrowser browser;
    GggsStoreSource* source = addGggsSource(browser);
    ASSERT_TRUE(browser.seedRoot(source, store.path()));
    ASSERT_EQ(QSettings().value("CatalogBrowser/seeds").toStringList().size(), 1);

    browser.removeSeed(0);
    EXPECT_EQ(browser.topLevelCount(), 0);
    EXPECT_TRUE(QSettings().value("CatalogBrowser/seeds").toStringList().isEmpty())
        << "removeSeed() must de-persist the browsed root";
  }

  CatalogBrowser restored;
  addGggsSource(restored);
  restored.restoreSeeds();
  EXPECT_EQ(restored.topLevelCount(), 0);
}

// The seeds_<->top-level-row invariant under a MIDDLE removal. RemoveDePersists
// only exercises index 0; a mid-list erase must shift the model rows and the
// parallel seeds_ identically, so the survivors stay matched to the correct roots
// (no drift to the wrong root).
TEST(CatalogBrowserSeeds, MidListRemovalKeepsAlignment)
{
  QSettings().clear();
  QTemporaryDir a, b, c;   // three distinct roots; their basenames are unique
  ASSERT_TRUE(a.isValid() && b.isValid() && c.isValid());
  touchTif(a.path() + "/bathymetry/tileset", "0_0_0.tif");
  touchTif(b.path() + "/bathymetry/tileset", "0_0_0.tif");
  touchTif(c.path() + "/bathymetry/tileset", "0_0_0.tif");

  CatalogBrowser browser;
  GggsStoreSource* source = addGggsSource(browser);
  ASSERT_TRUE(browser.seedRoot(source, a.path()));
  ASSERT_TRUE(browser.seedRoot(source, b.path()));
  ASSERT_TRUE(browser.seedRoot(source, c.path()));
  ASSERT_EQ(browser.topLevelCount(), 3);

  browser.removeSeed(1);   // erase the MIDDLE root (B / index-1 top-level row)
  ASSERT_EQ(browser.topLevelCount(), 2);

  // Survivors are A and C IN ORDER. Verify the model rows directly (a top-level
  // node's DisplayRole is the root's basename) — this catches a drift where
  // seeds_ erased index 1 but the model erased some other row.
  auto* view = browser.findChild<QTreeView*>();
  ASSERT_NE(view, nullptr);
  QAbstractItemModel* model = view->model();
  ASSERT_NE(model, nullptr);
  EXPECT_EQ(model->data(model->index(0, 0)).toString(),
            QFileInfo(a.path()).fileName());
  EXPECT_EQ(model->data(model->index(1, 0)).toString(),
            QFileInfo(c.path()).fileName());

  // ...and the persisted seeds are exactly A and C, in order (seeds_ shifted the
  // same way the model rows did).
  const QStringList seeds = QSettings().value("CatalogBrowser/seeds").toStringList();
  ASSERT_EQ(seeds.size(), 2);
  EXPECT_EQ(seeds.at(0), QStringLiteral("gggs\t") + a.path());
  EXPECT_EQ(seeds.at(1), QStringLiteral("gggs\t") + c.path());
}

// restoreSeeds() must skip a malformed (no-delimiter) persisted entry without
// crashing or mis-keying — exercises the `tab < 0` decode-skip branch
// (catalog_browser.cpp). A well-formed entry alongside it still restores, so the
// skip is surgical.
TEST(CatalogBrowserSeeds, RestoreSkipsMalformedEntry)
{
  QSettings().clear();
  QTemporaryDir store;
  ASSERT_TRUE(store.isValid());
  touchTif(store.path() + "/bathymetry/tileset", "0_0_0.tif");

  // A no-tab entry ("garbage") and an empty entry are both malformed (tab < 0);
  // only the well-formed "gggs\t<root>" should rebuild a top-level node.
  QSettings().setValue("CatalogBrowser/seeds",
                       QStringList{QStringLiteral("garbage"), QString(),
                                   QStringLiteral("gggs\t") + store.path()});

  CatalogBrowser browser;
  addGggsSource(browser);
  browser.restoreSeeds();   // must not crash on the malformed entries

  EXPECT_EQ(browser.topLevelCount(), 1)
      << "only the well-formed seed restores; malformed entries add no node";
}

int main(int argc, char** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  QCoreApplication::setOrganizationName("camp_test");
  QCoreApplication::setApplicationName("test_catalog_browser_seeds");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
