// [camp#208] Regression: a multi-band live tile must keep its empty cells
// TRANSPARENT across a write/reload cycle.
//
// The bug this pins: bands each carry their own NoData sentinel on the wire
// (quantization needs an in-range integer), and the cache writer set that
// sentinel per band. GeoTIFF's TIFFTAG_GDAL_NODATA stores ONE value per
// dataset, so only the last band's sentinel survived and was applied to all of
// them. On reload the other bands had no recognised NoData, every untouched
// cell read back as real data, and a mostly-empty tile painted solid over the
// chart — an 8x8 degree apex tile covering New England, in the case that found
// this.
//
// The fix normalises every band's sentinel to NaN, so one slot is enough. These
// tests fail against the pre-fix writer: with distinct per-band sentinels, the
// band whose value lost the race reports ~100% valid cells after reload.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include <QDir>
#include <QTemporaryDir>

#include "marine_autonomy/gggs.h"
#include "marine_interfaces/msg/sonar_visualization_tile.hpp"
#include "marine_interfaces/msg/visualization_band.hpp"

#include "ros/live_coverage/sonar_live_tile.h"

namespace mi = marine_interfaces::msg;
using camp::ros::live_coverage::SonarLiveTile;
using camp::ros::live_coverage::tileIndexFromGridIndex;

namespace
{

constexpr int kEdge = 8;
constexpr int kLevel = 10;

/// One band, INT16-quantized, whose own NoData sentinel differs per band — the
/// shape that broke the GeoTIFF round trip.
mi::VisualizationBand makeBand(const std::string& name, double nodata, double scale,
                               double offset, std::int16_t value, int covered_cells)
{
  mi::VisualizationBand band;
  band.name = name;
  band.dtype = mi::VisualizationBand::INT16;
  band.scale = scale;
  band.offset = offset;
  band.nodata = nodata;
  const auto nd = static_cast<std::int16_t>(nodata);
  for(int i = 0; i < kEdge * kEdge; ++i)
  {
    const std::int16_t raw = (i < covered_cells) ? value : nd;
    band.data.push_back(static_cast<std::uint8_t>(raw & 0xff));
    band.data.push_back(static_cast<std::uint8_t>((raw >> 8) & 0xff));
  }
  return band;
}

/// Count cells a renderer would DRAW: finite, and not the band's NoData.
std::size_t drawableCells(const SonarLiveTile& tile, const std::string& name)
{
  const auto* band = tile.band(name);
  if(!band)
    return 0;
  std::size_t n = 0;
  for(float v : band->data)
    if(std::isfinite(v) && !(band->has_nodata && v == band->nodata))
      ++n;
  return n;
}

mi::SonarVisualizationTile makeThreeBandPatch(const gggs::GridIndex& grid, int covered)
{
  mi::SonarVisualizationTile msg;
  msg.header.stamp.sec = 100;
  msg.header.frame_id = "gggs";
  msg.index = tileIndexFromGridIndex(grid);
  msg.width = kEdge;
  msg.height = kEdge;
  msg.window_col = 0;
  msg.window_row = 0;
  msg.window_width = kEdge;
  msg.window_height = kEdge;
  // Three DIFFERENT sentinels — one GeoTIFF nodata slot cannot hold them all.
  msg.bands.push_back(makeBand("backscatter", 200.0, 0.01, 0.0, 150, covered));
  msg.bands.push_back(makeBand("depth", -32768.0, 0.01, 0.0, -1500, covered));
  msg.bands.push_back(makeBand("uncertainty", 1275.0, 0.01, 0.0, 30, covered));
  return msg;
}

}  // namespace

TEST(SonarLiveTileNoData, EmptyCellsStayTransparentAcrossReload)
{
  const gggs::Level level(kLevel);
  const gggs::GridIndex grid = level.gridIndex(43.0, -70.5);
  constexpr int kCovered = 4;   // 4 of 64 cells carry real data

  SonarLiveTile tile(grid, kEdge, kEdge);
  tile.applyPatch(makeThreeBandPatch(grid, kCovered));

  // Before writing: every band reports only the covered cells as drawable.
  for(const char* name : {"backscatter", "depth", "uncertainty"})
    EXPECT_EQ(drawableCells(tile, name), std::size_t(kCovered)) << name << " (in memory)";

  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const std::string path = QDir(dir.path()).filePath("tile.tif").toStdString();
  ASSERT_TRUE(tile.writeToGeoTiff(path));

  const auto reloaded = SonarLiveTile::loadFromGeoTiff(path, level);
  ASSERT_TRUE(reloaded.has_value());

  // After reload: still only the covered cells. Pre-fix, the two bands whose
  // sentinel lost the single GeoTIFF nodata slot reported all 64 as drawable,
  // which is what painted an empty tile solid.
  for(const char* name : {"backscatter", "depth", "uncertainty"})
    EXPECT_EQ(drawableCells(*reloaded, name), std::size_t(kCovered))
      << name << " (after reload) — empty cells must not become drawable";
}

TEST(SonarLiveTileNoData, SentinelIsNaNSoOneGeoTiffSlotSuffices)
{
  const gggs::Level level(kLevel);
  const gggs::GridIndex grid = level.gridIndex(43.0, -70.5);

  SonarLiveTile tile(grid, kEdge, kEdge);
  tile.applyPatch(makeThreeBandPatch(grid, 4));

  // Every band normalises to the SAME sentinel, which is what makes the
  // one-value-per-dataset GeoTIFF limit harmless.
  for(const char* name : {"backscatter", "depth", "uncertainty"})
  {
    const auto* band = tile.band(name);
    ASSERT_NE(band, nullptr) << name;
    EXPECT_TRUE(std::isnan(band->nodata)) << name << " sentinel should be NaN";
  }
}
