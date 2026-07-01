// [camp#121] Headless tests for the CAMP live tile cache (Part B). No Qt event
// loop, no ROS node, no boat:
//   1. warm-load round-trip          (SonarLiveTile write-through -> loadCacheDir)
//   2. patch-apply / dequantize      (SonarLiveTile::applyPatch, INT16 depth band)
//   3. downtime-gap reconcile        (request the new tile, prune the gone one)
//   4. prune timestamp-gate          (a held version newer than the catalog stays)
//
// Tests 3-4 exercise the node-boundary conversion (wire TileCatalog ->
// reconciler) plus marine_tiled_raster_store::TileCatalogReconciler. The GL render
// is NOT exercised here (it self-skips with no offscreen GL; see test_gggs_render).

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <vector>

#include <QTemporaryDir>

#include "marine_autonomy/gggs.h"
#include "marine_tiled_raster_store/tile_catalog.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include "marine_interfaces/msg/sonar_visualization_tile.hpp"
#include "marine_interfaces/msg/tile_catalog.hpp"
#include "marine_interfaces/msg/visualization_band.hpp"

#include "ros/live_coverage/sonar_live_tile.h"

namespace mi = marine_interfaces::msg;
namespace mtrs = marine_tiled_raster_store;
using camp::ros::live_coverage::SonarLiveTile;
using camp::ros::live_coverage::gridIndexFromTileIndex;
using camp::ros::live_coverage::tileIndexFromGridIndex;
using camp::ros::live_coverage::toNanoseconds;
using camp::ros::live_coverage::toReconcilerCatalog;

namespace
{

constexpr int kLevel = 10;
constexpr int kEdge = 8;   // small synthetic tile (loadFromGeoTiff has no kEdge gate)

builtin_interfaces::msg::Time timeFromSec(int32_t sec)
{
  builtin_interfaces::msg::Time t;
  t.sec = sec;
  t.nanosec = 0;
  return t;
}

void putInt16LE(std::vector<std::uint8_t>& bytes, int16_t value)
{
  bytes.push_back(static_cast<std::uint8_t>(value & 0xff));
  bytes.push_back(static_cast<std::uint8_t>((value >> 8) & 0xff));
}

// A full-tile INT16 "depth" patch for @p grid: every cell raw=base, with optional
// per-cell overrides keyed by (gggs_row, gggs_col). scale=0.01, nodata=-32768.
mi::SonarVisualizationTile makeDepthPatch(
  const gggs::GridIndex& grid, int16_t base, int32_t stamp_sec,
  const std::vector<std::tuple<int, int, int16_t>>& overrides = {})
{
  mi::SonarVisualizationTile msg;
  msg.header.stamp = timeFromSec(stamp_sec);
  msg.header.frame_id = "gggs";
  msg.index = tileIndexFromGridIndex(grid);
  msg.width = kEdge;
  msg.height = kEdge;
  msg.window_col = 0;
  msg.window_row = 0;
  msg.window_width = kEdge;
  msg.window_height = kEdge;

  mi::VisualizationBand band;
  band.name = "depth";
  band.dtype = mi::VisualizationBand::INT16;
  band.scale = 0.01;
  band.offset = 0.0;
  band.nodata = -32768.0;
  band.data.reserve(static_cast<size_t>(kEdge) * kEdge * 2);
  for(int r = 0; r < kEdge; ++r)
    for(int c = 0; c < kEdge; ++c)
    {
      int16_t raw = base;
      for(const auto& o : overrides)
        if(std::get<0>(o) == r && std::get<1>(o) == c)
          raw = std::get<2>(o);
      putInt16LE(band.data, raw);
    }
  msg.bands.push_back(band);
  return msg;
}

// North-up cell index for a GGGS (row, col) in a kEdge x kEdge tile.
size_t northUpIndex(int gggs_row, int col)
{
  return static_cast<size_t>((kEdge - 1) - gggs_row) * kEdge + col;
}

bool contains(const std::vector<gggs::GridIndex>& v, const gggs::GridIndex& g)
{
  return std::find(v.begin(), v.end(), g) != v.end();
}

}  // namespace

// 2. Patch-apply / dequantize. A full-tile INT16 depth patch dequantizes
// value = raw * scale, the NoData sentinel is excluded from the auto-range and
// lands flipped to north-up at the right cell.
TEST(SonarLiveCache, PatchApplyDequantize)
{
  const gggs::Level level(kLevel);
  const gggs::GridIndex grid = level.gridIndex(43.07, -70.76);
  ASSERT_TRUE(grid.valid());

  SonarLiveTile tile(grid, kEdge, kEdge);
  // base raw 200 -> 2.0; one peak raw 500 -> 5.0 at (gggs 0,0); one NoData at (1,1).
  tile.applyPatch(makeDepthPatch(grid, 200, 100,
                                 {{0, 0, 500}, {1, 1, -32768}}));

  const auto* band = tile.band("depth");
  ASSERT_NE(band, nullptr);
  ASSERT_EQ(band->data.size(), static_cast<size_t>(kEdge) * kEdge);

  // Dequantized values at the flipped (north-up) positions.
  EXPECT_FLOAT_EQ(band->data[northUpIndex(2, 2)], 2.0f);   // a plain base cell
  EXPECT_FLOAT_EQ(band->data[northUpIndex(0, 0)], 5.0f);   // the peak

  // NoData cell: stored as the dequantized sentinel and excluded from the range.
  EXPECT_FLOAT_EQ(band->data[northUpIndex(1, 1)], -327.68f);
  EXPECT_TRUE(band->has_nodata);
  EXPECT_FLOAT_EQ(band->data_min, 2.0f);
  EXPECT_FLOAT_EQ(band->data_max, 5.0f);

  // The tile version is the patch stamp flattened to ns.
  EXPECT_EQ(tile.version(), toNanoseconds(timeFromSec(100)));
}

// 1. Warm-load round-trip: write tiles through (Float32 GeoTIFF), then warm-load
// the directory and recover the tile count + data range.
TEST(SonarLiveCache, WarmLoadRoundTrip)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const gggs::Level level(kLevel);

  // Two distinct grids (well over a level-10 grid span apart).
  const gggs::GridIndex a = level.gridIndex(43.07, -70.76);
  const gggs::GridIndex b = level.gridIndex(43.50, -70.20);
  ASSERT_TRUE(a.valid());
  ASSERT_TRUE(b.valid());
  ASSERT_FALSE(a == b);

  for(const auto& grid : {a, b})
  {
    SonarLiveTile tile(grid, kEdge, kEdge);
    tile.applyPatch(makeDepthPatch(grid, 300, 100, {{0, 0, 700}}));   // 3.0 .. 7.0
    const std::string path =
      dir.filePath(QString("%1_%2_%3.tif")
                     .arg(static_cast<int>(grid.level()))
                     .arg(grid.row())
                     .arg(grid.column())).toStdString();
    ASSERT_TRUE(tile.writeToGeoTiff(path));
  }

  std::vector<SonarLiveTile> loaded =
    SonarLiveTile::loadCacheDir(dir.path().toStdString(), level);
  ASSERT_EQ(loaded.size(), 2u);

  for(const auto& tile : loaded)
  {
    const auto* band = tile.band("depth");
    ASSERT_NE(band, nullptr);
    EXPECT_FLOAT_EQ(band->data_min, 3.0f);
    EXPECT_FLOAT_EQ(band->data_max, 7.0f);
    // The GridIndex round-tripped through the geotransform.
    EXPECT_TRUE(tile.index() == a || tile.index() == b);
  }
}

// 3. Downtime-gap reconcile (the acceptance scenario). Before downtime the cache
// holds A,B,C; the boat (which was up while CAMP was down) now advertises A,B,D.
// Reconcile must request D (newly present) and prune C (gone), then converge.
TEST(SonarLiveCache, DowntimeGapReconcile)
{
  const gggs::Level level(kLevel);
  const gggs::GridIndex a = level.gridIndex(43.07, -70.76);
  const gggs::GridIndex b = level.gridIndex(43.50, -70.20);
  const gggs::GridIndex c = level.gridIndex(42.60, -71.30);
  const gggs::GridIndex d = level.gridIndex(44.10, -69.50);
  ASSERT_TRUE(a.valid() && b.valid() && c.valid() && d.valid());

  mtrs::TileCatalogReconciler reconciler;
  const mtrs::TileVersion v_old = toNanoseconds(timeFromSec(100));
  reconciler.markHave(a, v_old);
  reconciler.markHave(b, v_old);
  reconciler.markHave(c, v_old);

  // Catalog generation-time newer than the held versions, so prune-on-absence
  // applies to C.
  mi::TileCatalog catalog;
  catalog.header.stamp = timeFromSec(200);
  for(const auto& grid : {a, b, d})
  {
    mi::TileCatalogEntry entry;
    entry.index = tileIndexFromGridIndex(grid);
    entry.version = timeFromSec(100);
    catalog.entries.push_back(entry);
  }

  const mtrs::ReconcileResult result = reconciler.reconcile(toReconcilerCatalog(catalog));

  EXPECT_EQ(result.to_request.size(), 1u);
  EXPECT_TRUE(contains(result.to_request, d));
  EXPECT_FALSE(contains(result.to_request, a));
  EXPECT_FALSE(contains(result.to_request, b));

  EXPECT_EQ(result.to_prune.size(), 1u);
  EXPECT_TRUE(contains(result.to_prune, c));

  // Model the consumer acting on the result, then a second identical catalog must
  // converge to no work.
  reconciler.markHave(d, v_old);
  reconciler.drop(c);
  const mtrs::ReconcileResult after = reconciler.reconcile(toReconcilerCatalog(catalog));
  EXPECT_TRUE(after.to_request.empty());
  EXPECT_TRUE(after.to_prune.empty());
}

// 4. Prune timestamp-gate (ADR-0006 D4 / uma ADR-0008 D4b): a held tile whose
// version is NEWER than the catalog generation-time is NOT pruned even when absent
// — a late/reordered catalog cannot delete a just-pushed fresh tile.
TEST(SonarLiveCache, PruneTimestampGate)
{
  const gggs::Level level(kLevel);
  const gggs::GridIndex a = level.gridIndex(43.07, -70.76);
  const gggs::GridIndex b = level.gridIndex(43.50, -70.20);
  const gggs::GridIndex c = level.gridIndex(42.60, -71.30);
  ASSERT_TRUE(a.valid() && b.valid() && c.valid());

  mtrs::TileCatalogReconciler reconciler;
  reconciler.markHave(a, toNanoseconds(timeFromSec(100)));
  reconciler.markHave(b, toNanoseconds(timeFromSec(100)));
  // C is fresher than the catalog we're about to receive.
  reconciler.markHave(c, toNanoseconds(timeFromSec(300)));

  // Catalog at sec=200 lists only A,B — C is absent but newer than generation_time.
  mi::TileCatalog catalog;
  catalog.header.stamp = timeFromSec(200);
  for(const auto& grid : {a, b})
  {
    mi::TileCatalogEntry entry;
    entry.index = tileIndexFromGridIndex(grid);
    entry.version = timeFromSec(100);
    catalog.entries.push_back(entry);
  }

  const mtrs::ReconcileResult result = reconciler.reconcile(toReconcilerCatalog(catalog));
  EXPECT_FALSE(contains(result.to_prune, c));   // protected by the timestamp gate
  EXPECT_TRUE(result.to_prune.empty());
  EXPECT_TRUE(result.to_request.empty());       // A,B held at current version
}

// 5. [camp#160] foldChild decimates a fine tile into its coarse parent. A uniform
// child covers ~1/4 of the (same-sized) parent, so exactly (kEdge/2)^2 parent cells
// take the child value and the rest stay NoData (untouched by this one child).
TEST(SonarLiveCache, FoldChildDecimatesIntoParentQuadrant)
{
  const gggs::Level fine_level(kLevel);
  const gggs::GridIndex fine_idx = fine_level.gridIndex(43.07, -70.76);
  ASSERT_TRUE(fine_idx.valid());
  const gggs::GridIndex parent_idx = gggs::parent(fine_idx);
  ASSERT_TRUE(parent_idx.valid());

  // Uniform fine tile: every cell = 3.0 (raw 300, scale 0.01).
  SonarLiveTile fine(fine_idx, kEdge, kEdge);
  fine.applyPatch(makeDepthPatch(fine_idx, 300, 100));

  // Parent matches the fine tile's dimensions (standard pyramid).
  SonarLiveTile parent(parent_idx, kEdge, kEdge);
  parent.foldChild(fine);

  const auto* pb = parent.band("depth");
  ASSERT_NE(pb, nullptr);
  ASSERT_EQ(pb->data.size(), static_cast<size_t>(kEdge) * kEdge);

  int written = 0;
  for(float v : pb->data)
    if(v == 3.0f)
      ++written;
  EXPECT_EQ(written, (kEdge / 2) * (kEdge / 2));   // one quadrant, 2x2-decimated
  EXPECT_FLOAT_EQ(pb->data_min, 3.0f);             // uniform child -> uniform mean
  EXPECT_FLOAT_EQ(pb->data_max, 3.0f);
  EXPECT_TRUE(pb->has_nodata);
}

// 6. [camp#160] foldChild NoData handling: a 2x2 child block that is entirely NoData
// leaves its parent cell a NoData hole; a partial block averages only the finite
// samples.
TEST(SonarLiveCache, FoldChildPropagatesNoData)
{
  const gggs::Level fine_level(kLevel);
  const gggs::GridIndex fine_idx = fine_level.gridIndex(43.07, -70.76);
  const gggs::GridIndex parent_idx = gggs::parent(fine_idx);
  ASSERT_TRUE(parent_idx.valid());

  // Base 400 (= 4.0). One 2x2 child block (gggs rows 6-7, cols 0-1) entirely NoData
  // -> a single parent NoData hole. One extra NoData cell (7,2) makes an adjacent
  // parent cell a partial block averaging the 3 finite samples to 4.0.
  SonarLiveTile fine(fine_idx, kEdge, kEdge);
  fine.applyPatch(makeDepthPatch(fine_idx, 400, 100,
                                 {{7, 0, -32768}, {7, 1, -32768},
                                  {6, 0, -32768}, {6, 1, -32768},
                                  {7, 2, -32768}}));

  SonarLiveTile parent(parent_idx, kEdge, kEdge);
  parent.foldChild(fine);
  const auto* pb = parent.band("depth");
  ASSERT_NE(pb, nullptr);

  int written = 0;
  int holes = 0;
  for(float v : pb->data)
  {
    if(v == 4.0f)
      ++written;
    else if(pb->has_nodata && v == pb->nodata)
      ++holes;
  }
  // (kEdge/2)^2 parent cells are in the child's quadrant; the all-NoData block is a
  // hole, the rest (incl. the partial block averaged to 4.0) are written.
  EXPECT_EQ(written, (kEdge / 2) * (kEdge / 2) - 1);
  EXPECT_GE(holes, 1);
  EXPECT_FLOAT_EQ(pb->data_min, 4.0f);
  EXPECT_FLOAT_EQ(pb->data_max, 4.0f);
}
