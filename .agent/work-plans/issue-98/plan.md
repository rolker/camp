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

2. **Schedule eviction from `paint()`, run it DEFERRED (as built)** — after the
   existing visibility loop, `paint()` does NOT delete any tile (deleting a `Tile`,
   a `QGraphicsObject` scene child, mid-paint is a use-after-free risk per the Plan
   Review must-fix). Instead it:
   - Records `tile_last_visible_gen_[addr] = paint_generation_` for each visible tile.
   - Increments `paint_generation_`.
   - Computes the cap `max(kTileEvictionMinCap, kTileEvictionMultiplier * int(visible_tiles.size()))`.
   - If `tiles_.size() > cap` AND `!eviction_pending_`, sets `eviction_pending_ = true`
     and queues `evictIfNeeded()` onto the event loop via
     `QMetaObject::invokeMethod(this, "evictIfNeeded", Qt::QueuedConnection)`. The
     `eviction_pending_` flag debounces this so paint() never queues more than one
     pending eviction.

   **`evictIfNeeded()` (private slot, the primary deletion path)** runs in the event
   loop where deleting scene items is safe:
   - Clears `eviction_pending_`.
   - Recomputes the cap from the CURRENT visible count (counts `tile->isVisible()`).
   - Collects candidates = tiles NOT visible right now (live `isVisible()` guard, so a
     tile re-shown between scheduling and now is never evicted).
   - Sorts candidates ascending by `tile_last_visible_gen_` (missing entry → 0 =
     oldest) and deletes oldest-first (`delete` + `tiles_.erase` + erase its
     `tile_last_visible_gen_` entry) until `tiles_.size() <= cap`. Because
     `cap >= visible_count`, there are always enough non-visible candidates to reach
     the cap without touching a visible tile.

3. **Clear eviction state in `setLayout()`** — add `tile_last_visible_gen_.clear()`
   and `eviction_pending_ = false` alongside the existing `tiles_.clear()`.
   `paint_generation_` is not reset (monotonic counter; cleared entries for the new
   tile set start fresh).

4. **Update `onRefreshTimer` comment** — lines 150–155 in `map_tiles.cpp` say
   "paint() still only hides (not deletes) tiles … between refreshes." This will no
   longer be accurate once step 2 lands; update the comment to reflect that eviction
   now also bounds within-cycle accumulation.

5. **Add `test_map_tiles_eviction.cpp` (as built)** — a MULTI-zoom-level layout is
   required: `setLayout()` pre-seeds the whole `zoom_levels.front()` grid at
   construction, so a single-zoom layout mints nothing in `paint()` and would not
   reproduce the bug (Plan Review suggestion). The test builds a 1×1 front level plus
   a 20×20 deeper level; `paint()` renders and mints the deeper level on demand.
   `paint()` is driven directly with a hand-built `QPainter` world transform
   (`QTransform(8,0,0,-8,tx,ty)` — uniform scale 8 with a web-mercator y-flip), chosen
   so all index arithmetic in `MapTiles::paint()` lands on exact integers; this is
   deterministic and avoids `QGraphicsScene::render()` QPA fragility entirely (the
   plan's fallback proved unnecessary). A small (one-tile) viewport is panned across
   every position of the 20×20 grid; because eviction is DEFERRED,
   `QApplication::processEvents()` is called after each `paint()` so the queued
   `evictIfNeeded()` runs. The test asserts `tileChildCount()` stays ≤ the 256 floor
   throughout and after the full sweep. A second test (`DeepLevelTilesAreMintedByPaint`)
   asserts the deeper tiles really are minted by `paint()` (not pre-seeded), so the
   pan genuinely exercises the accumulation path. Non-vacuity was verified empirically:
   with `evictIfNeeded()` stubbed to a no-op, `tiles_` reaches 401 (1 front + 400 deep)
   and the test fails at the 256 cap. Follows `test_map_tiles_refresh.cpp` patterns
   (offscreen QApplication, `tileChildCount` helper).

6. **Register test in `CMakeLists.txt`** — mirror the `test_map_tiles_refresh` block:
   `ament_add_gtest`, include `${CMAKE_CURRENT_SOURCE_DIR}/src/camp2`, link
   `camp_map Qt5::Widgets Qt5::Test`.

## Files to Change

| File | Change |
|------|--------|
| `src/camp2/map_tiles/map_tiles.h` | Add `paint_generation_`, `tile_last_visible_gen_`, `eviction_pending_`, and the `evictIfNeeded()` private slot |
| `src/camp2/map_tiles/map_tiles.cpp` | Eviction constants (file-private); schedule deferred eviction in `paint()`; `evictIfNeeded()` slot (primary deletion path); clear eviction state in `setLayout()`; update `onRefreshTimer` comment |
| `test/test_map_tiles_refresh.cpp` | Update SCOPE comment to point at the new eviction test |
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

## Open Questions (resolved)

- **Constant values — RESOLVED (operator-confirmed).** `kTileEvictionMinCap = 256`,
  `kTileEvictionMultiplier = 4`, giving `cap = max(256, 4 × visible)`. OSM tiles decode
  to ~256×256 ARGB (~256 KB each), so the 256-tile floor caps the basemap tile buffer
  at ~64 MB — a generous pan buffer sized for the salmon operator workstation, bounded
  so an all-afternoon survey can't OOM-kill CAMP. The multiplier keeps the cap above the
  working set at high zoom. Rationale is documented at the constants' definition site in
  `map_tiles.cpp`.
- **Test driving mechanism — RESOLVED.** `QGraphicsScene::render()` was not needed.
  `paint()` is called directly with a hand-built `QPainter` world transform, which is
  fully deterministic under offscreen QPA and lets the test step the viewport exactly
  tile-by-tile. The planned `evictIfNeeded`-via-`invokeMethod` fallback became the
  PRIMARY production mechanism (deferred eviction, per the Plan Review must-fix) rather
  than a test-only path.

## Estimated Scope

Single PR.
