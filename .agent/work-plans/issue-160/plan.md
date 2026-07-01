# Plan: Consumer-side overview pyramid + bounded eviction for live sonar coverage

## Issue

https://github.com/rolker/camp/issues/160

## Context

`SonarLiveCacheLayer::tiles_` is unbounded today — the only cap is reconciler
prune-on-absence. During a sustained live survey the map grows monotonically until
RAM/VRAM is exhausted (the salmon crash, #153). A naive fix (evict fine tiles) makes
zoomed-out views blank over evicted areas. The overview pyramid is the graceful-degradation
companion: before dropping a fine tile, decimate it into its coarse parent so coverage
degrades to lower resolution instead of disappearing.

ADR-0006 makes this safe: live tiles are display-grade previews, always rebuildable from
the boat's durable stores. Building downsampled overviews is display-LOD generation with
no provenance burden.

The work spans two repos:
- **unh_marine_autonomy** (GGGS `parent()`/`children()` helpers) — Phase A, prerequisite PR
- **camp** (eviction + overview pyramid) — Phase B, depends on Phase A

**Sub-decisions resolved** (operator checkpoint 2, 2026-07-01 → applied here):
- Eviction trigger: **view-based LOD** — a configurable byte budget for VRAM accounting;
  when over budget, evict the fine tiles **farthest from the current viewport centre**
  (classic slippy-map/LOD behaviour — keep what's near the view, discard the rest), with
  pure LRU as the fallback when no view is available (headless / tests). **No
  vessel-position / tf dependency** (operator dropped distance-from-vessel as needless
  complication). Self-account locally (camp knows every texture size); adopt
  `ResourceMonitor` VRAM feed when #155 lands (the plan leaves a named integration seam).
- Overview chain depth: full chain from received fine level all the way to level 0 (one
  whole-survey tile). Levels are 4× smaller per step; the full chain is a handful of
  tiles — cheap enough to always keep resident.
- Fold timing: fold-on-evict only (cheapest). The overview is updated only when a fine
  tile is actually dropped. The overview may lag the latest fine data until first eviction;
  this is acceptable per ADR-0006 D1 (display-grade, rebuildable).
- Reconciler on evict: call `reconciler_.drop(index)` on eviction. The evicted tile is
  on disk (persist-then-drop); the next catalog re-requests it if the boat still has it.
  This keeps the reconciler's in-memory set consistent with the in-memory tile map.

## Approach

### Phase A — GGGS index math helpers (unh_marine_autonomy, separate PR)

**Phase A lands as its own standalone `unh_marine_autonomy` issue + PR** (title ~"Add
GGGS `parent()`/`children()` index-math helpers"), tagged **`Part of rolker/camp#160`** in
the body, merged and the underlay rebuilt before camp Phase B can build against it.

1. **Add `marine_autonomy/gggs/index_math.h`** — two free functions:
   - `GridIndex parent(const GridIndex& child)` — computes the child cell centre from
     `child.northLatitude()/southLatitude()/eastLongitude()/westLongitude()` (GridIndex has
     **no** `center_lat`/`center_lon` accessor — verified `gggs/grid_index.h:87-102`), then
     `Level(child.level()-1).gridIndex(center_lat, center_lon)` to handle polar
     `latitudeScaleFactor` (1/3/9) column-scale changes at the 72°/80° latitude
     boundaries correctly, without manual column arithmetic. Returns an invalid
     `GridIndex` when `child.level() == 0` or the child is invalid.
   - `std::vector<GridIndex> children(const GridIndex& parent)` — returns the 2–4
     child GridIndex values at `parent.level()+1` whose geographic extent overlaps
     the parent. In the normal latitude bands this is always 4 (2 rows × 2 cols);
     at polar-scale-change boundaries the child rows may cross a scale-factor
     transition, so the child column range is computed from the parent's west/east
     longitudes rather than from a fixed ×2 multiplier. Returns empty when
     `parent.level() == 20` or the parent is invalid.

2. **Add `marine_autonomy/test/test_gggs_index_math.cpp`** — tests:
   - `parent()` at equatorial, sub-polar (72°), and polar (80°) band boundaries.
   - Round-trip: `parent(c)` equals `parent(sibling)` for two children of the same
     parent.
   - `parent()` at level 0 returns invalid.
   - `children()` of a given parent contains exactly the tiles whose `parent()` maps
     back to that parent.
   - `children()` at level 20 returns empty.

3. **Update `marine_autonomy/gggs.h`** — add `#include "gggs/index_math.h"` so
   callers get both helpers with the existing single include.

4. **Update `marine_autonomy/CMakeLists.txt`** — add `ament_add_gtest` for
   `test_gggs_index_math` linked to `${PROJECT_NAME}`.

### Phase B — Eviction + overview pyramid (camp, PR 2, depends on Phase A)

5. **Add camp ADR-0010** (`docs/decisions/0010-bounded-eviction-overview-pyramid.md`) —
   records the eviction + overview design decisions: trigger (distance + byte budget),
   overview chain depth (full to level 0), fold timing (on-evict), reconciler interaction
   (drop on evict), overview isolation from reconciler. Supersedes the TODO in ADR-0006
   consequences. Add an ADR-0006 addendum cross-reference per ADR-0012.

6. **Add `OverviewEntry` struct and `overview_tiles_` map** to `SonarLiveCacheLayer`:
   ```cpp
   struct OverviewEntry {
     SonarLiveTile tile;
     std::unique_ptr<QOpenGLTexture> texture;
     bool texture_dirty = true;
   };
   std::map<gggs::GridIndex, OverviewEntry> overview_tiles_;
   ```
   Overview tiles are indexed by their coarse `GridIndex` and are **never** entered in
   `tiles_` or touched by `reconciler_`. The dtor tears down `overview_tiles_` textures
   under the renderer context the same way it handles `tiles_`.

7. **Add eviction budget state** to `SonarLiveCacheLayer`:
   - `size_t vram_budget_bytes_` — loaded from
     `QSettings("LiveTileCache/max_vram_bytes")` in the constructor; default 512 MiB.
   - `uint64_t access_seq_` — monotonic counter; each `Entry` carries a
     `uint64_t last_access_seq` bumped in `handleTile()` and `textureFor()` (the LRU
     fallback ordering when no view is available).
   - **No vessel-position state** — eviction ordering uses the current viewport centre
     (below), obtained on demand from `scene()->views()`; no ROS position subscription.
   - `size_t accountedBytes() const` — sum of `width * height * 4` (R32F) across all
     `Entry` textures in `tiles_` that have a non-null texture.

8. **Implement `evictIfOverBudget()`** (GUI thread):
   - Compute `accountedBytes()`. If under `vram_budget_bytes_`, return immediately.
   - Build a sorted eviction candidate list: entries in `tiles_` ordered by distance of
     the tile's scene-space centre from the **current viewport centre** (farthest first).
     Obtain the viewport centre in Web-Mercator scene coords via
     `scene()->views().first()->mapToScene(viewport rect).boundingRect().center()`. When
     no view is attached (headless / tests), fall back to LRU order by `last_access_seq`
     (oldest first).
   - For each candidate until under budget:
     - Call `foldIntoParent(entry.tile)` (step 9).
     - Call `scheduleWriteThrough(entry.tile)` (persist before drop, reusing the
       existing coalesced write path).
     - Free the GL texture under the renderer context (reuse the `hasContext()` /
       `makeCurrent()` guard from `handleCatalog()`).
     - Erase from `tiles_`; call `reconciler_.drop(index)`.
   - Call `evictIfOverBudget()` at the end of `handleTile()` and after `warmLoad()`.
   - Log (qWarning) when entering the shed-load path so the operator can see it in
     the console.

9. **Implement `foldIntoParent(const SonarLiveTile& fine)`** (GUI thread):
   - Compute `parent_idx = gggs::parent(fine.index())`. If invalid (level 0), return.
   - Look up or create an `OverviewEntry` in `overview_tiles_[parent_idx]`:
     - On first creation: `SonarLiveTile(parent_idx, fine.width(), fine.height())` — the
       parent tile **matches the fine tile's `width`/`height`** (operator decision:
       standard pyramid where every tile is the same pixel size, so a parent W×H tile
       covers the area of its 4 children at half linear resolution).
   - Determine the child's **quadrant** within the parent (NW/NE/SW/SE) from the child's
     row/col parity relative to `parent(child)`'s children (or from the child vs parent
     geographic bounds). 2×2 mean-decimate the fine `W×H` band into the corresponding
     `W/2 × H/2` quadrant of the parent band, skipping NoData cells (a 2×2 block that is
     all NoData stays NoData). Mark the overview `texture_dirty = true`.
   - Schedule write-through for the updated overview tile, writing to a sub-directory
     `overviews/` within `cache_dir_` so warm-load of fine tiles cannot accidentally pick
     up overview files. **Small API change (review-plan finding):** `scheduleWriteThrough`
     / `startWriteThrough` currently hard-target `cache_dir_` root
     (`sonar_live_cache_layer.h:151`); add an optional destination-subdir parameter (default
     "" = root for fine tiles, "overviews" for parents) threaded through the write-state
     bookkeeping so overview and fine writes stay serialized on distinct paths.
   - Recurse: `foldIntoParent(overview_tiles_[parent_idx].tile)` to build the full
     chain to level 0.

10. **Bound `warmLoad()`**:
    - Load overview tiles from `<cache_dir_>/overviews/<level>_<row>_<col>.tif` into
      `overview_tiles_` before loading fine tiles (overviews are kept resident, never
      evicted).
    - After loading fine tiles into `tiles_`, call `evictIfOverBudget()` to trim to the
      budget before activating the subscription.

11. **LOD fallback in `items()`**:
    - After collecting the normal `tiles_` entries, for each `OverviewEntry` in
      `overview_tiles_`: emit a `RasterFieldItem` only if no finer `tiles_` entry
      covers the same area (check by level comparison — if any `tiles_` entry with a
      higher level number overlaps this overview's bounds, skip the overview). In
      practice this means: include an overview only when at least one of its children
      is absent from `tiles_`.
    - `recomputeBounds()` must union extents from both `tiles_` and `overview_tiles_`
      so the bounding rect stays correct after fine tiles are evicted.
    - `foldAutoRange()` must also fold overview bands into `data_min_`/`data_max_` so
      the colormap range tracks the full visible extent.

12. **Update `updateDisplay()`**:
    - Emit `"(live: %1 fine + %2 overview tiles)"` so the operator can see both counts.

13. **Extend `test_sonar_live_cache.cpp`**:
    - Eviction bounding: insert tiles up to 2× the byte budget; verify that
      `tiles_.size()` stays within the budget and evicted tiles appear in
      `overview_tiles_`.
    - Fold-into-parent decimation: verify 2×2 mean of known values; verify NoData
      propagation.
    - Render fallback: after evicting a fine tile, verify that `items()` returns an
      overview item covering its area.
    - `warmLoad()` bounding: pre-populate a cache dir with more tiles than the budget;
      verify warm-load trims to budget.
    - Reconciler isolation: verify that `overview_tiles_` keys are never present in
      `reconciler_`'s held set after fold.

## Files to Change

| File | Change |
|------|--------|
| `core_ws/.../marine_autonomy/include/marine_autonomy/gggs/index_math.h` | New: `parent()` and `children()` free functions |
| `core_ws/.../marine_autonomy/include/marine_autonomy/gggs.h` | Add `#include "gggs/index_math.h"` |
| `core_ws/.../marine_autonomy/test/test_gggs_index_math.cpp` | New: unit tests for both helpers |
| `core_ws/.../marine_autonomy/CMakeLists.txt` | Register `test_gggs_index_math` target |
| `ui_ws/src/camp/docs/decisions/0010-bounded-eviction-overview-pyramid.md` | New camp ADR-0010 |
| `ui_ws/src/camp/docs/decisions/0006-live-tile-cache-persistence.md` | Addendum cross-ref to ADR-0010 |
| `ui_ws/src/camp/src/camp_map/ros/live_coverage/sonar_live_cache_layer.h` | `OverviewEntry`, `overview_tiles_`, budget/position state |
| `ui_ws/src/camp/src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp` | `evictIfOverBudget()`, `foldIntoParent()`, bounded `warmLoad()`, LOD fallback in `items()` |
| `ui_ws/src/camp/test/test_sonar_live_cache.cpp` | Extended tests per step 13 |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Budgets via QSettings (ADR-0006 D2 / camp#117); status line shows fine + overview tile counts; qWarning logged on force-evict. |
| Capture decisions, not just implementations | ADR-0010 records the four open sub-decisions before implementation. ADR-0006 addendum per ADR-0012. |
| A change includes its consequences | `recomputeBounds()` and `foldAutoRange()` extended to cover overview tiles; existing test suite stays green; reconciler isolation tested explicitly. |
| Only what's needed | Fold-on-evict only (not fold-on-arrival). `children()` added to GGGS for completeness and future boat-side reuse, but camp only calls `parent()`. |
| Improve incrementally | Two-PR structure: GGGS helper is independently testable before the camp consumer lands. |
| Test what breaks | Tests target the crash scenario (unbounded growth) and the blank-zoom-out regression, not framework glue. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| camp ADR-0006 (live tile cache persistence) | Yes | Overview tiles use same `<level>_<row>_<col>.tif` format in `overviews/` sub-dir. Overview tiles stay out of reconciler (correctness constraint). Eviction reuses `scheduleWriteThrough` persist-then-drop path. |
| camp ADR-0006 D2 (no hardcoded defaults) | Yes | `max_vram_bytes` defaults to 512 MiB but overridable via QSettings. |
| camp ADR-0006 D4 (GUI-thread invariant) | Yes | Eviction, folding, and GL texture release all run on GUI thread. ROS position callback marshals to GUI thread. |
| camp ADR-0001 (threading) | Yes | No new thread-safety concerns; eviction callbacks are queued to GUI thread same as tile/catalog callbacks. |
| workspace ADR-0012 (cross-ref addendum) | Yes | ADR-0006 addendum is navigational (points to ADR-0010); substantive decisions go in ADR-0010. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `tiles_` eviction | `reconciler_.drop()` call so anti-entropy re-requests if still current | Yes (step 8) |
| Overview tile write (overviews/ sub-dir) | `warmLoad()` must load overviews from that sub-dir on re-enable | Yes (step 10) |
| `recomputeBounds()` now unions overview tiles | `foldAutoRange()` must also fold overview tile ranges | Yes (step 11) |
| Camp depends on `gggs::parent()` | unh_marine_autonomy Phase A must land and underlay be rebuilt before camp Phase B can build | Yes — two-PR structure; Phase A merges first |
| `#155 ResourceMonitor` lands | Replace `accountedBytes()` self-accounting with `ResourceMonitor` VRAM feed; the `vram_budget_bytes_` knob stays the same | Not in this PR — named integration seam (leave a `// TODO(#155)` comment) |

## Open Questions (resolved at operator checkpoint 2, 2026-07-01)

- [x] **Eviction ordering / vessel source** → **view-based LOD, no vessel source.** Evict
  fine tiles farthest from the current viewport centre (via `scene()->views()`), pure LRU
  fallback when headless. No ROS position subscription / tf dependency. (Operator: distance-
  from-vessel needlessly complicates; use traditional LOD — discard those farthest from the
  viewpoint.)
- [x] **Overview resolution** → **match the fine tile's `width`/`height`.** Standard pyramid:
  every tile is the same pixel size; a parent W×H tile covers 4 children at half linear
  resolution, each child decimated into its W/2×H/2 quadrant.

## Estimated Scope

Two PRs:
- **Phase A (unh_marine_autonomy)**: ~80 lines (header + tests + CMake) — small, independent.
- **Phase B (camp)**: ~500–700 lines across 4 files — medium; the bulk is
  `evictIfOverBudget()`, `foldIntoParent()`, and the test extensions.
