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
//   - removeSeed() de-persists, so a fresh restore yields no node.
//
// Discovery only stats directories for `*.tif` filenames, so an empty placeholder
// tile keeps this widget/model-level and GL/GDAL-free. QSettings is scoped to a
// test org/app in main() so it never touches the real config.

#include <gtest/gtest.h>

#include <memory>

#include <QApplication>
#include <QDir>
#include <QFile>
#include <QSettings>
#include <QTemporaryDir>

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

int main(int argc, char** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  QCoreApplication::setOrganizationName("camp_test");
  QCoreApplication::setApplicationName("test_catalog_browser_seeds");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
