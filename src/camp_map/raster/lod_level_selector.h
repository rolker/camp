#ifndef RASTER_LOD_LEVEL_SELECTOR_H
#define RASTER_LOD_LEVEL_SELECTOR_H

#include "marine_autonomy/gggs.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace camp
{
namespace raster
{

/// [camp#103 / ADR-0013] Pick the display LOD: the level of @p available_levels
/// closest to the ideal GGGS level for @p ground_metres_per_pixel, preferring
/// the coarser side.
///
/// The ideal is `gggs::Level::fromCellSize(mpp)` — the coarsest level whose
/// cells are at most one screen pixel (so a render at it is crisp). From the
/// available levels we take the finest one that is coarser-or-equal to the
/// ideal (numerically the largest level number <= ideal): cells stay within
/// [0.5, 1] px of ideal on a contiguous pyramid, and on a sparse ladder the
/// choice degrades toward coarser (fewer tiles) rather than loading finer data
/// than the screen can show. If every available level is finer than the ideal,
/// the coarsest available (smallest number) is the closest match and is
/// returned. Returns -1 for an empty @p available_levels — the caller's
/// "no selection / no filter" sentinel.
///
/// This is the generic multi-level wiring (operator decision 2026-07-31): a
/// GGGS store layer calls it with the levels found in its directory + its
/// `overviews/` sidecar (uma ADR-0011), and a future natively multi-level
/// layer (chart ENC scale ladder, uma ADR-0010) calls the same function with
/// its native ladder. `ground_metres_per_pixel` must be TRUE ground metres —
/// Web-Mercator scene metres are inflated by ~sec(latitude), so callers
/// convert first (web_mercator::metersPerUnit at the viewport centre).
///
/// `inline` (header-only, multiple TUs) — the gggs_tile_util.h pattern.
inline int selectLodLevel(double ground_metres_per_pixel,
                          const std::vector<int>& available_levels)
{
  if(available_levels.empty())
    return -1;
  // fromCellSize clamps into the valid GGGS level range, so any positive
  // metres-per-pixel is safe; guard the degenerate non-positive input — and a
  // NaN (which std::max would pass through into ceil() UB) from a future
  // caller with a broken viewport centre.
  const double mpp = std::isfinite(ground_metres_per_pixel)
    ? std::max(ground_metres_per_pixel, 1e-6) : 1e-6;
  const int ideal = gggs::Level::fromCellSize(static_cast<float>(mpp)).level();
  int best_coarser = -1;   // largest available level number <= ideal
  int coarsest = available_levels.front();
  for(const int level : available_levels)
  {
    coarsest = std::min(coarsest, level);
    if(level <= ideal && level > best_coarser)
      best_coarser = level;
  }
  return best_coarser >= 0 ? best_coarser : coarsest;
}

}  // namespace raster
}  // namespace camp

#endif
