// [camp#104] Discovery-model coverage for raster::GggsStoreSource (ADR-0005).
//
// GggsStoreSource::discover() folder-scans a store root into a generic
// catalog::CatalogItem tree: a directory holding `*.tif` directly is a
// selectable tile-set leaf (key = its path); a directory with tiles only deeper
// is a group; tile-free subtrees are excluded. These tests build a temp store
// and assert the resulting tree shape, leaf payloads, and exclusions. No GL /
// GDAL is needed — discovery only stats directories for `*.tif` names, so the
// tiles can be empty placeholder files.

#include <gtest/gtest.h>

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QTemporaryDir>

#include "catalog/catalog_item.h"
#include "raster/gggs_store_source.h"

using camp::catalog::CatalogItem;
using camp::raster::GggsStoreSource;

namespace
{

// Create an empty placeholder tile file `<dir>/<name>` (content irrelevant to
// discovery, which only matches the `*.tif` filename).
void touchTif(const QString& dir, const QString& name)
{
  QDir().mkpath(dir);
  QFile f(QDir(dir).filePath(name));
  ASSERT_TRUE(f.open(QIODevice::WriteOnly));
  f.close();
}

// Find a child of `parent` by display name, or nullptr.
const CatalogItem* childNamed(const CatalogItem* parent, const QString& name)
{
  for(int i = 0; i < parent->childCount(); ++i)
    if(parent->child(i)->name() == name)
      return parent->child(i);
  return nullptr;
}

}  // namespace

// A modality -> maturity -> tile-set store discovers as nested groups with the
// tile-set directories as selectable leaves; tile-free subtrees are excluded.
TEST(CatalogSourceTest, DiscoversStoreTree)
{
  QTemporaryDir store;
  ASSERT_TRUE(store.isValid());
  const QString root = store.path();

  const QString tileset_a = root + "/bathymetry/draft/tileset_a";
  const QString tileset_b = root + "/sidescan/tileset_b";
  touchTif(tileset_a, "0_0_0.tif");
  touchTif(tileset_b, "0_0_0.tif");
  touchTif(tileset_b, "0_0_1.tif");
  QDir().mkpath(root + "/empty/deeper");        // no tiles anywhere -> excluded

  GggsStoreSource source;
  std::unique_ptr<CatalogItem> tree = source.discover(root);
  ASSERT_NE(tree, nullptr);

  // Root is a group with exactly the two tile-bearing modalities (empty/ pruned).
  EXPECT_FALSE(tree->isLeaf());
  EXPECT_EQ(tree->childCount(), 2);
  EXPECT_EQ(childNamed(tree.get(), "empty"), nullptr) << "tile-free subtree must be excluded";

  const CatalogItem* bathy = childNamed(tree.get(), "bathymetry");
  ASSERT_NE(bathy, nullptr);
  EXPECT_FALSE(bathy->isLeaf());
  const CatalogItem* draft = childNamed(bathy, "draft");
  ASSERT_NE(draft, nullptr);
  const CatalogItem* leaf_a = childNamed(draft, "tileset_a");
  ASSERT_NE(leaf_a, nullptr);

  // The tile-set is a selectable leaf carrying the source id + its directory key.
  EXPECT_TRUE(leaf_a->isLeaf());
  EXPECT_EQ(leaf_a->sourceId(), QStringLiteral("gggs"));
  EXPECT_EQ(leaf_a->key(), tileset_a);
  EXPECT_EQ(leaf_a->childCount(), 0) << "tiles under a tile-set are not nested catalog nodes";

  const CatalogItem* sidescan = childNamed(tree.get(), "sidescan");
  ASSERT_NE(sidescan, nullptr);
  const CatalogItem* leaf_b = childNamed(sidescan, "tileset_b");
  ASSERT_NE(leaf_b, nullptr);
  EXPECT_TRUE(leaf_b->isLeaf());
  EXPECT_EQ(leaf_b->key(), tileset_b);
}

// A root that is itself a single tile-set (tiles directly in it) discovers as a
// single selectable leaf.
TEST(CatalogSourceTest, RootThatIsATileSetIsALeaf)
{
  QTemporaryDir store;
  ASSERT_TRUE(store.isValid());
  touchTif(store.path(), "0_0_0.tif");

  GggsStoreSource source;
  std::unique_ptr<CatalogItem> tree = source.discover(store.path());
  ASSERT_NE(tree, nullptr);
  EXPECT_TRUE(tree->isLeaf());
  EXPECT_EQ(tree->key(), store.path());
}

// A root with no tiles anywhere (and a non-existent path) yields no catalog.
TEST(CatalogSourceTest, EmptyOrMissingRootYieldsNothing)
{
  QTemporaryDir store;
  ASSERT_TRUE(store.isValid());
  QDir().mkpath(store.path() + "/just/dirs/no/tiles");

  GggsStoreSource source;
  EXPECT_EQ(source.discover(store.path()), nullptr);
  EXPECT_EQ(source.discover(store.path() + "/does-not-exist"), nullptr);
  EXPECT_EQ(source.discover(QString()), nullptr);
}

int main(int argc, char** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QCoreApplication app(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
