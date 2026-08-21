// [camp#104] Regression tests for GggsTileLayer::rescan() — the manual refresh
// wired to the "Rescan" context-menu action (the stopgap for the retired
// QFileSystemWatcher; ADR-0005).
//
// These pin down rescan()'s add / no-op CONTRACT, the surface a Round-2 review
// must-fix reshaped: rescan() now computes the new-tile set FIRST and only
// disturbs the layer (and any in-flight async pixel load) when there is at least
// one tile to add; a rescan that finds nothing is a non-destructive no-op
// returning false.
//
// Non-GL / non-async by construction: rescan()'s directory scan + extent merge
// run synchronously on the calling thread (the pixel load is only kicked from
// paint()/waitForLoad(), neither of which runs here), so these assert
// deterministically without an offscreen GL context or a running load worker.
//
// The ONE behavior these cannot pin down is the in-flight-load case the must-fix
// directly targets — rescan-finds-nothing must leave a still-running pixel-load
// worker alone. Forcing a deterministically in-flight worker would need a
// worker-pause seam plus a pixelsLoaded() accessor the production class
// deliberately does not expose; a timing-based variant would only detect the bug
// on the rounds where tiles happen to load slowly (false assurance otherwise).
// That path is covered by code review (see progress.md). What IS guarded here is
// the contract the fix preserves: no-new -> false + extent untouched; new ->
// true + extent grows.

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include <gdal_priv.h>

#include <QApplication>
#include <QDateTime>
#include <QFile>
#include <QFileInfo>
#include <QTemporaryDir>

#include "map/map.h"
#include "map/layer_list.h"
#include "raster/gggs_tile_layer.h"

namespace
{

// Write a north-up WGS84 GeoTIFF tile so GggsTile::valid() is true and its extent
// comes from the geotransform. lon0/lat0 place the NW corner, so successive tiles
// can be given disjoint extents (to observe the layer extent grow on rescan).
// [camp#194 review round 3] `value` is the constant sample written to every
// pixel (so the tile's data range is exactly [value, value] — the seam the
// aggregate-range regression tests assert on), and `n` the raster size, whose
// pixel size is scaled to keep the tile's geographic extent fixed: a rewrite at
// a different `n` therefore changes the FILE SIZE (an unambiguous
// fileChangedOnDisk() signal, independent of mtime granularity) without moving
// the tile.
QString writeTile(const QTemporaryDir& dir, const QString& name,
                  double lon0, double lat0, uint16_t value = 8000, int n = 16)
{
  if(GDALGetDriverCount() == 0)
    GDALAllRegister();
  const int w = n, h = n;
  const double pixel = 0.0001 * 16.0 / n;
  const double geo[6] = {lon0, pixel, 0.0, lat0, 0.0, -pixel};
  const QString path = dir.filePath(name);
  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GTiff");
  GDALDataset* ds = driver->Create(path.toUtf8().constData(), w, h, 1, GDT_UInt16, nullptr);
  ds->SetGeoTransform(const_cast<double*>(geo));
  GDALRasterBand* band = ds->GetRasterBand(1);
  band->SetNoDataValue(0);
  std::vector<uint16_t> samples(static_cast<size_t>(w) * h, value);
  const CPLErr err = band->RasterIO(GF_Write, 0, 0, w, h, samples.data(),
                                    w, h, GDT_UInt16, 0, 0);
  GDALClose(ds);
  return err == CE_None ? path : QString();
}

}  // namespace

// rescan() with nothing new is a non-destructive no-op: returns false and leaves
// the layer's extent untouched. This is the path the Round-2 must-fix made safe —
// it must NOT abort/disturb the layer (or a live load) when there is nothing to
// add. Idempotent: a second rescan with no change also returns false.
TEST(GggsRescanTest, NoNewTilesIsNoOp)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  ASSERT_FALSE(writeTile(dir, "13_0_0.tif", -71.40, 43.00).isEmpty());

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());
  const QRectF bounds_before = layer->sceneBounds();

  EXPECT_FALSE(layer->rescan());                  // nothing new
  EXPECT_EQ(layer->sceneBounds(), bounds_before); // extent untouched
  EXPECT_FALSE(layer->rescan());                  // idempotent
  EXPECT_EQ(layer->sceneBounds(), bounds_before);
}

// rescan() picks up a newly-landed tile: returns true and unions its extent into
// the layer bounds (west edge unchanged since the original tile is westmost; the
// extent widens eastward). A subsequent rescan with nothing new returns false.
TEST(GggsRescanTest, PicksUpNewlyLandedTile)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  ASSERT_FALSE(writeTile(dir, "13_0_0.tif", -71.40, 43.00).isEmpty());

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());
  const QRectF bounds_before = layer->sceneBounds();

  // A second tile to the east (disjoint extent) lands after the layer loaded.
  ASSERT_FALSE(writeTile(dir, "13_0_1.tif", -71.39, 43.00).isEmpty());

  EXPECT_TRUE(layer->rescan());                   // tile added
  EXPECT_DOUBLE_EQ(layer->sceneBounds().left(), bounds_before.left());  // westmost unchanged
  EXPECT_GT(layer->sceneBounds().width(), bounds_before.width());       // grew eastward
  EXPECT_FALSE(layer->rescan());                  // now nothing new
}

// [camp#112] loadDirectory() filters companion tiles (`_time`/`_source`): a tile
// directory holding the base value tile plus its companions enumerates EXACTLY
// the base tile. The companions are written at a disjoint (eastward) extent, so
// if they were (wrongly) loaded as tiles the layer's sceneBounds would widen —
// the layer instead matches a base-only layer's extent exactly.
TEST(GggsRescanTest, CompanionTilesAreNotLoaded)
{
  QTemporaryDir base_only;
  ASSERT_TRUE(base_only.isValid());
  ASSERT_FALSE(writeTile(base_only, "13_0_0.tif", -71.40, 43.00).isEmpty());

  camp::map::Map base_map;
  auto* base_layer =
    new camp::raster::GggsTileLayer(base_map.topLevelLayers(), base_only.path());
  ASSERT_TRUE(base_layer->valid());
  const QRectF base_bounds = base_layer->sceneBounds();

  QTemporaryDir with_companions;
  ASSERT_TRUE(with_companions.isValid());
  ASSERT_FALSE(writeTile(with_companions, "13_0_0.tif", -71.40, 43.00).isEmpty());
  // Companions at a disjoint eastward extent: a real GeoTIFF, excluded by NAME.
  ASSERT_FALSE(writeTile(with_companions, "13_0_0_time.tif", -71.39, 43.00).isEmpty());
  ASSERT_FALSE(writeTile(with_companions, "13_0_0_source.tif", -71.38, 43.00).isEmpty());

  camp::map::Map map;
  auto* layer =
    new camp::raster::GggsTileLayer(map.topLevelLayers(), with_companions.path());
  ASSERT_TRUE(layer->valid());
  // Only the base tile counted: same extent as the base-only layer (companions'
  // eastward extents did not widen it).
  EXPECT_EQ(layer->sceneBounds(), base_bounds)
      << "companion tiles must not be loaded as renderable tiles";
}

// [camp#112] rescan() filters companion tiles too: companions landing after the
// initial load are not treated as new renderable tiles, so rescan() no-ops
// (returns false) and leaves the extent untouched.
TEST(GggsRescanTest, RescanIgnoresCompanionTiles)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  ASSERT_FALSE(writeTile(dir, "13_0_0.tif", -71.40, 43.00).isEmpty());

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());
  const QRectF bounds_before = layer->sceneBounds();

  // Companions for the existing tile land afterward (disjoint eastward extents).
  ASSERT_FALSE(writeTile(dir, "13_0_0_time.tif", -71.39, 43.00).isEmpty());
  ASSERT_FALSE(writeTile(dir, "13_0_0_source.tif", -71.38, 43.00).isEmpty());

  EXPECT_FALSE(layer->rescan());                  // companions are not new tiles
  EXPECT_EQ(layer->sceneBounds(), bounds_before); // extent untouched
}

// [camp#194 review] A latched load failure must not be permanent. GggsTile
// latches loadFailed() so a dead tile stops wedging the loader — but the causes
// are not necessarily permanent (a transient NFS error; a producer replacing the
// file: uma's `enc_updater` rewrites the chart layer on a cron cycle,
// `overview_pyramid` does rename-aside directory swaps, both potentially under a
// running CAMP). Without a same-band retry path the repaired tile stays blank
// until CAMP restarts: rescan()'s known-path dedup skips the path, the load
// worker skips loadFailed() tiles, and a one-band store never calls setBand().
// rescan() therefore detects a CHANGED file (size/mtime) at a known path and
// refreshes the tile — metadata re-read, failure cleared, pixels re-read.
TEST(GggsRescanTest, RepairedTileRecoversWithoutRestart)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  ASSERT_FALSE(writeTile(dir, "13_0_0.tif", -71.40, 43.00).isEmpty());
  const QString repaired_path = writeTile(dir, "13_0_1.tif", -71.39, 43.00);
  ASSERT_FALSE(repaired_path.isEmpty());

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());

  // Truncate the second tile AFTER the metadata scan: it passed valid(), but its
  // pixels can never be read — loadPixels() latches the sticky failure.
  {
    QFile file(repaired_path);
    ASSERT_TRUE(file.open(QIODevice::WriteOnly | QIODevice::Truncate));
    ASSERT_TRUE(file.resize(0));
  }
  layer->waitForLoad();
  ASSERT_EQ(layer->pixelsLoadedCount(13), 1);
  ASSERT_TRUE(layer->status().contains("failed")) << layer->status().toStdString();

  // The latch holds against an ordinary re-kick — that is its purpose (no retry
  // storm on a dead tile).
  layer->waitForLoad();
  ASSERT_EQ(layer->pixelsLoadedCount(13), 1);

  // The producer now writes a good tile back at the SAME path.
  ASSERT_FALSE(writeTile(dir, "13_0_1.tif", -71.39, 43.00).isEmpty());

  EXPECT_TRUE(layer->rescan()) <<
    "rescan() did not notice the replaced file — a repaired tile stays blank "
    "until CAMP restarts";
  layer->waitForLoad();
  EXPECT_EQ(layer->pixelsLoadedCount(13), 2) <<
    "the repaired tile's pixels were not re-read: the sticky load failure "
    "survived the file swap";
  EXPECT_FALSE(layer->status().contains("failed")) <<
    "the layer still reports a failed tile after the repair: " <<
    layer->status().toStdString();

  // Nothing changed since the refresh re-stat'ed the file: back to a no-op.
  EXPECT_FALSE(layer->rescan());
}

// [camp#194 review round 3] The stat-gated refresh above does NOT cover the case
// the sticky-latch finding was raised against: a TRANSIENT read error on a file
// nobody rewrote. Size and mtime are then unchanged, so a stat-gated Rescan
// returns false forever and the tile stays blank for the session — the fix not
// landing, rather than a new defect. rescan() therefore retries every latched
// tile regardless of the stat.
//
// The transient error is staged by renaming the tile aside so loadPixels()'s
// GDALOpen() fails, then renaming it BACK: POSIX rename preserves the inode's
// size and mtime, so the restored file is byte-for-byte the file the tile
// stat'ed at construction — fileChangedOnDisk() is false and only the
// latched-failure branch can drive the recovery. The test asserts that premise
// explicitly rather than assuming it.
TEST(GggsRescanTest, TransientFailureOnUnchangedFileRetriesOnRescan)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  ASSERT_FALSE(writeTile(dir, "13_0_0.tif", -71.40, 43.00).isEmpty());
  const QString flaky_path = writeTile(dir, "13_0_1.tif", -71.39, 43.00);
  ASSERT_FALSE(flaky_path.isEmpty());

  const QFileInfo before(flaky_path);
  const qint64 size_before = before.size();
  const qint64 mtime_before = before.lastModified().toMSecsSinceEpoch();

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());

  // The "NFS blip": the file is unreachable exactly while the worker reads it.
  const QString aside = dir.filePath("13_0_1.tif.aside");
  ASSERT_TRUE(QFile::rename(flaky_path, aside));
  layer->waitForLoad();
  ASSERT_EQ(layer->pixelsLoadedCount(13), 1);
  ASSERT_TRUE(layer->status().contains("failed")) << layer->status().toStdString();

  // The blip passes: the SAME file is reachable again, with the SAME stat.
  ASSERT_TRUE(QFile::rename(aside, flaky_path));
  const QFileInfo after(flaky_path);
  ASSERT_EQ(after.size(), size_before);
  ASSERT_EQ(after.lastModified().toMSecsSinceEpoch(), mtime_before)
      << "premise broken: the restored file's stat changed, so this test would "
         "pass through the file-changed path instead of the latched-failure one";

  EXPECT_TRUE(layer->rescan()) <<
    "rescan() skipped a latched tile whose file never changed — a transient "
    "read error is still permanent for the session";
  layer->waitForLoad();
  EXPECT_EQ(layer->pixelsLoadedCount(13), 2) <<
    "the latched tile's pixels were not re-read after the transient failure "
    "cleared";
  EXPECT_FALSE(layer->status().contains("failed")) <<
    "the layer still reports a failed tile after the transient error cleared: "
    << layer->status().toStdString();

  // Nothing failed and nothing changed: back to a no-op (no Rescan churn on a
  // healthy store).
  EXPECT_FALSE(layer->rescan());
}

// [camp#194 review round 3] refreshFromFile() drops a tile's range and can
// REPLACE an already-loaded tile's data, but tilesReady()'s fold only ever
// WIDENS the layer aggregate — so without an explicit invalidation the old
// file's extremes stay in the Auto colormap range forever. On a bathymetry
// display that is a wrong range the operator reads as real depth. rescan()
// therefore recomputes the aggregate from the resident set after a refresh.
TEST(GggsRescanTest, RefreshRecomputesAutoRangeInsteadOfWidening)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  // Sole tile, constant 8000 -> aggregate range [8000, 8000].
  ASSERT_FALSE(writeTile(dir, "13_0_0.tif", -71.40, 43.00, 8000).isEmpty());

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());
  layer->waitForLoad();
  ASSERT_EQ(layer->pixelsLoadedCount(13), 1);
  ASSERT_FLOAT_EQ(layer->dataRange().first, 8000.0f);
  ASSERT_FLOAT_EQ(layer->dataRange().second, 8000.0f);

  // The producer replaces it with a disjoint, LOWER-valued grid (a different
  // raster size so the swap is unambiguous at mtime granularity).
  ASSERT_FALSE(writeTile(dir, "13_0_0.tif", -71.40, 43.00, 100, 32).isEmpty());

  EXPECT_TRUE(layer->rescan());
  layer->waitForLoad();
  EXPECT_EQ(layer->pixelsLoadedCount(13), 1);
  EXPECT_FLOAT_EQ(layer->dataRange().second, 100.0f) <<
    "the replaced tile's old maximum survived in the layer aggregate — Auto "
    "shows a colormap range no resident pixel occupies";
  EXPECT_FLOAT_EQ(layer->dataRange().first, 100.0f);
  EXPECT_FLOAT_EQ(layer->rangeHi(), 100.0f);
  EXPECT_FLOAT_EQ(layer->rangeLo(), 100.0f);
}

// [camp#194 review round 3] The recompute must NOT disturb an operator's Manual
// range override: it is deliberately independent of the data extents (camp#142).
TEST(GggsRescanTest, RefreshPreservesManualRangeOverride)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  ASSERT_FALSE(writeTile(dir, "13_0_0.tif", -71.40, 43.00, 8000).isEmpty());

  camp::map::Map map;
  auto* layer = new camp::raster::GggsTileLayer(map.topLevelLayers(), dir.path());
  ASSERT_TRUE(layer->valid());
  layer->waitForLoad();

  layer->setRangeOverride(0.0f, 50.0f);
  ASSERT_EQ(layer->rangeMode(), marine_colormap::RangeMode::Manual);

  ASSERT_FALSE(writeTile(dir, "13_0_0.tif", -71.40, 43.00, 100, 32).isEmpty());
  EXPECT_TRUE(layer->rescan());
  layer->waitForLoad();

  EXPECT_EQ(layer->rangeMode(), marine_colormap::RangeMode::Manual) <<
    "the post-refresh recompute dropped the operator's Manual range override";
  EXPECT_FLOAT_EQ(layer->rangeLo(), 0.0f);
  EXPECT_FLOAT_EQ(layer->rangeHi(), 50.0f);
  // The underlying data extents still tracked the replacement.
  EXPECT_FLOAT_EQ(layer->dataRange().second, 100.0f);
}

int main(int argc, char** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  // [camp#117] Map's ctor now writes the BackgroundTileLayers seed into QSettings;
  // a test org/app name keeps that out of the developer's real camp settings.
  QCoreApplication::setOrganizationName("camp_test");
  QCoreApplication::setApplicationName("test_gggs_rescan");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
