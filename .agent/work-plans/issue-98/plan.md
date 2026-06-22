# Plan: CAMP silently crashes on map zoom/pan during live ops (recurring) — likely OOM

## Issue

https://github.com/rolker/camp/issues/98

## Context

`MapTiles::paint()` (`src/camp2/map_tiles/map_tiles.cpp`) creates a new `Tile*` for
each newly-visible tile address and inserts it into `tiles_` (a
`std::map<TileAddress, Tile*>`). Off-screen tiles are only hidden via
`setVisible(false)`. The **only** deletion of accumulated tiles is in `setLayout()`,
which is called at construction and on each `onRefreshTimer` fire.

The radar overlay calls `setRefreshInterval()`, so `setLayout()` runs every ~5 min and
self-bounds. The OSM/WMTS basemap never calls `setRefreshInterval()`, so `setLayout()`
is never called after initial construction. Every new tile area visited in a pan/zoom
session adds a permanent entry to `tiles_` → RSS grows monotonically → OOM-kill (silent
crash with no stacktrace). This matches the field evidence (crash while zooming/panning,
auto-relaunch healthy).

The `test_map_tiles_refresh.cpp` comment (lines 149–156) explicitly notes this gap:
"this guards ONLY the refresh-boundary reset … does NOT cover the within-cycle growth
that the #98 OOM is about."

## Approach

1. **Add LRU eviction state to `MapTiles`** — a paint-call generation counter
   (`paint_generation_`) and a per-tile last-visible generation map
   (`tile_last_visible_gen_`). These track the most recent paint call in which each
   tile was in the visible set. Both live in `MapTiles` (no change to `Tile`).

2. **Evict in `paint()`** — after the existing visibility loop:
   - Record `tile_last_visible_gen_[addr] = paint_generation_` for each visible tile.
   - Increment `paint_generation_`.
   - Compute eviction cap: `max(kTileEvictionMinCap, kTileEvictionMultiplier * int(visible_tiles.size()))`.
   - If `tiles_.size() > cap`: collect all non-visible tiles, sort ascending by
     `tile_last_visible_gen_` (missing entry → 0, treated as oldest), delete the oldest
     until `tiles_.size() <= cap`. Erase corresponding `tile_last_visible_gen_` entries.
   - Currently-visible tiles are never evicted (guarded by the visible set check).

3. **Clear eviction state in `setLayout()`** — add `tile_last_visible_gen_.clear()`
   alongside the existing `tiles_.clear()`. `paint_generation_` is not reset
   (monotonic counter; cleared entries for the new tile set start fresh).

4. **Update `onRefreshTimer` comment** — lines 150–155 in `map_tiles.cpp` say
   "paint() still only hides (not deletes) tiles … between refreshes." This will no
   longer be accurate once step 2 lands; update the comment to reflect that eviction
   now also bounds within-cycle accumulation.

5. **Add `test_map_tiles_eviction.cpp`** — simulate pan over a large layout and
   assert `tileChildCount()` stays at or below the eviction cap. Test drives `paint()`
   via `QGraphicsScene::render()` with the target MapTiles-containing scene, using a
   small source rect stepped across all tile positions of a large layout (e.g. 20×20 =
   400 tiles). After visiting all 400 positions, `tileChildCount()` must be ≤ cap.
   Follows `test_map_tiles_refresh.cpp` patterns (offscreen QApplication, `makeLayout`,
   `tileChildCount` helper). If `QGraphicsScene::render()` source-rect positioning
   proves unreliable under offscreen QPlatform, fall back to a protected slot
   `evictIfNeeded(const std::set<TileAddress>&)` invoked via `QMetaObject::invokeMethod`
   with a serialised argument.

6. **Register test in `CMakeLists.txt`** — mirror the `test_map_tiles_refresh` block:
   `ament_add_gtest`, include `${CMAKE_CURRENT_SOURCE_DIR}/src/camp2`, link
   `camp_map Qt5::Widgets Qt5::Test`.

## Files to Change

| File | Change |
|------|--------|
| `src/camp2/map_tiles/map_tiles.h` | Add `paint_generation_`, `tile_last_visible_gen_`, and eviction constants |
| `src/camp2/map_tiles/map_tiles.cpp` | Eviction logic in `paint()`; clear in `setLayout()`; update `onRefreshTimer` comment |
| `test/test_map_tiles_eviction.cpp` | New test asserting `tileChildCount()` bounded after pan |
| `CMakeLists.txt` | Register `test_map_tiles_eviction` target |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Safety first | Eviction guard ensures currently-visible tiles are never evicted — the operator display never shows a blank tile due to eviction |
| Only what's needed | Change is confined to `MapTiles`; no new classes, no change to `Tile` or `TileAddress` |
| Test what breaks | New test asserts the exact invariant that caused the OOM — `tiles_.size()` bounded during pan |
| A change includes its consequences | `onRefreshTimer` comment update included; `test_map_tiles_refresh.cpp` scope comment updated to reflect that the within-cycle gap is now closed |
| Capture decisions, not just implementations | Eviction constants documented with a comment at the definition site explaining the N× rationale |
| Improve incrementally | Single PR, single mechanism, no layout or API changes |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0002 (worktree isolation) | Yes | Work is in `issue-camp-98` worktree, camp repo on `feature/issue-98` |
| ADR-0008 (ROS 2 conventions) | Partial | C++ Qt code in a ROS 2 package; follows C++ naming conventions; no new package interfaces |
| ADR-0013 (progress.md vocabulary) | Yes | `## Plan Authored` entry written per vocabulary |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Eviction in `paint()` | `onRefreshTimer` comment (lines 150–155) that says "paint() still only hides" | Yes — step 4 |
| `tiles_` map now has eviction | `tile_last_visible_gen_` must be cleared in `setLayout()` to avoid stale entries after layout change | Yes — step 3 |
| Within-cycle gap closed | `test_map_tiles_refresh.cpp` SCOPE comment (lines 149–156) acknowledging the gap — update to note the gap is fixed in `test_map_tiles_eviction.cpp` | Yes — step 4 scope |

## Open Questions

- What values for `kTileEvictionMinCap` and `kTileEvictionMultiplier`? The issue
  says "N× visible-tile count"; a typical viewport at zoom 12 shows ~9–16 tiles. A
  cap of `max(256, 4 × visible)` keeps ~256+ tiles as a pan buffer before eviction
  kicks in. Confirm these constants are appropriate for deployment hardware (salmon)
  memory constraints — if RSS budget is tight, a lower `kTileEvictionMinCap` (e.g.
  128) may be needed.
- Test approach: confirm that `QGraphicsScene::render()` with a varying source rect
  produces distinct `painter->worldTransform()` values inside `MapTiles::paint()` in
  the offscreen test environment (no QPA rendering). If not, a fallback invocation
  path for the eviction logic will be needed.

## Estimated Scope

Single PR.
