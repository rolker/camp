#ifndef CAMP_ROS_LIVE_COVERAGE_SONAR_LIVE_TILE_H
#define CAMP_ROS_LIVE_COVERAGE_SONAR_LIVE_TILE_H

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "marine_autonomy/gggs.h"
#include "marine_tiled_raster_store/tile_catalog.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include "std_msgs/msg/header.hpp"
#include "marine_interfaces/msg/sonar_visualization_tile.hpp"
#include "marine_interfaces/msg/tile_catalog.hpp"
#include "marine_interfaces/msg/tile_index.hpp"
#include "marine_interfaces/msg/visualization_band.hpp"

namespace camp
{
namespace ros
{
namespace live_coverage
{

/// [camp#121] One dequantized band of a live coverage tile, held in memory as
/// Float32. The render path duplicates GggsTileLayer's, so the cell layout
/// matches GggsTile: row-major, **north-up** (row 0 = north), `width*height`
/// cells. NoData is the dequantized sentinel (`raw_nodata * scale + offset`);
/// the duplicated shader discards cells exactly equal to it, as GggsTile's does.
struct SonarLiveBand
{
  std::string name;          ///< "depth" | "uncertainty" | "backscatter" | ...
  std::vector<float> data;   ///< row-major north-up, width*height cells
  float nodata = 0.0f;       ///< dequantized NoData sentinel
  bool has_nodata = false;
  float data_min = 1.0f;     ///< auto-range over finite non-NoData cells
  float data_max = 0.0f;     ///< crossed (min > max) => no valid samples
};

/// [camp#121] An in-memory GGGS tile carrying dequantized Float32 bands received
/// over the live coverage transport (uma ADR-0008). Mirrors GggsTile where the
/// duplicated render path needs it, but never uses GDAL on the hot path — patches
/// arrive as `SonarVisualizationTile` messages and are dequantized generically
/// (`value = raw * scale + offset`). Persistence (write-through / warm-load) goes
/// through GeoTIFF for crash-safety only; see ADR-0006.
///
/// **Not thread-safe** (ADR-0001 / ADR-0006 D4): construct, applyPatch, and
/// read only on the GUI thread.
class SonarLiveTile
{
public:
  /// Empty tile for @p index sized @p width x @p height. Bands are created lazily
  /// by applyPatch / warm-load.
  SonarLiveTile(const gggs::GridIndex& index, int width, int height);

  /// [camp#121] Dequantize the message's dirty sub-window and patch it into each
  /// named band (creating the band, NoData-filled, on first sight). Bumps the
  /// tile version to `max(version, ns(header.stamp))` and re-folds each touched
  /// band's auto-range. Cells outside any received window stay at the band's
  /// NoData sentinel (so they discard in the shader). A band whose declared
  /// window exceeds the tile, or whose byte length doesn't match its dtype, is
  /// skipped rather than throwing.
  void applyPatch(const marine_interfaces::msg::SonarVisualizationTile& msg);

  /// [camp#121] Warm-load one cached Float32 GeoTIFF (all bands, band names from
  /// band descriptions, per-band NoData, GridIndex recovered from the
  /// geotransform at @p level — the marine_tiled_raster_store::loadTile
  /// convention). Returns std::nullopt if the file can't be opened, isn't a
  /// WGS84 geographic raster, or doesn't match a grid at @p level. version() is 0
  /// (warm-loaded: "have something", older than any real catalog version).
  static std::optional<SonarLiveTile> loadFromGeoTiff(const std::string& path,
                                                       const gggs::Level& level);

  /// [camp#121] Warm-load every `*.tif` under @p dir via loadFromGeoTiff(),
  /// skipping any file that fails to load (e.g. a half-written `.tif.tmp` left by
  /// a crash — those are named `.tif.tmp` and excluded by the glob anyway). A
  /// missing directory yields an empty vector. Used by the layer at activation
  /// and exercised directly by the headless test.
  static std::vector<SonarLiveTile> loadCacheDir(const std::string& dir,
                                                  const gggs::Level& level);

  /// [camp#121] Serialize all bands to a Float32 GeoTIFF at @p path (north-up
  /// WGS84, GGGS geotransform for index_, band description = band name, per-band
  /// NoData). The caller does the atomic temp+rename; this just writes @p path.
  /// Returns false on any GDAL failure (the cache write is best-effort).
  bool writeToGeoTiff(const std::string& path) const;

  const gggs::GridIndex& index() const { return index_; }
  int width() const { return width_; }
  int height() const { return height_; }

  /// Geographic extent in degrees, from the GGGS grid (north-up, pixel-edge).
  double minLon() const { return index_.westLongitude(); }
  double maxLon() const { return index_.eastLongitude(); }
  double minLat() const { return index_.southLatitude(); }
  double maxLat() const { return index_.northLatitude(); }

  /// Band by name, or nullptr if this tile has no such band yet.
  const SonarLiveBand* band(const std::string& name) const;
  std::vector<std::string> bandNames() const;
  int bandCount() const { return static_cast<int>(bands_.size()); }

  /// Per-tile version (ns since epoch; the latest applied/seeded version).
  marine_tiled_raster_store::TileVersion version() const { return version_; }

private:
  /// Re-fold @p band's data_min/data_max over its finite, non-NoData cells.
  static void refoldRange(SonarLiveBand& band);

  gggs::GridIndex index_;
  int width_ = 0;
  int height_ = 0;
  std::map<std::string, SonarLiveBand> bands_;
  marine_tiled_raster_store::TileVersion version_ = 0;
};

// ---- Node-boundary conversions (marine_interfaces wire <-> gggs/reconciler) ----
// [camp#121] CAMP is the node boundary that adapts the ROS wire messages to the
// ROS-free reconciler/gggs types (tile_catalog.hpp). Kept as free functions so
// the test can exercise the downtime-gap scenario without a layer/node.

/// Flatten a builtin_interfaces/Time to ns-since-epoch (the single TileVersion
/// representation; ADR-0006 D4).
marine_tiled_raster_store::TileVersion toNanoseconds(
  const builtin_interfaces::msg::Time& time);

/// Flatten a Header's stamp to ns-since-epoch.
marine_tiled_raster_store::TileVersion toNanoseconds(
  const std_msgs::msg::Header& header);

/// Wire TileIndex {level,row,col} -> gggs::GridIndex. The GridIndex ctor is
/// private; we reconstruct it the way loadTile does — map the cell center back
/// through gggs::Level — and verify the round-trip. Returns an invalid (sentinel)
/// GridIndex if level/row/col don't name a real grid (the reconciler ignores
/// invalid indices, matching its "invalid indices ignored at ingestion" rule).
gggs::GridIndex gridIndexFromTileIndex(const marine_interfaces::msg::TileIndex& index);

/// gggs::GridIndex -> wire TileIndex {level,row,col}.
marine_interfaces::msg::TileIndex tileIndexFromGridIndex(const gggs::GridIndex& index);

/// Adapt a wire TileCatalog to the reconciler's TileCatalog (indices flattened to
/// gggs::GridIndex, versions + generation_time flattened to ns). Invalid indices
/// are dropped.
marine_tiled_raster_store::TileCatalog toReconcilerCatalog(
  const marine_interfaces::msg::TileCatalog& catalog);

}  // namespace live_coverage
}  // namespace ros
}  // namespace camp

#endif
