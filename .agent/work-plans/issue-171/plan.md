# Plan: Live-coverage display collapses to low resolution during eviction (#171 + #172)

## Issue

https://github.com/rolker/camp/issues/171 (closes #171, closes #172)

## Context

During the 2026-07-23 BizzyBoat M3 survey, CAMP's live coverage collapsed to low resolution
mid-session. Root cause (verified in source):

**#171**: `foldIntoParent()` (sonar_live_cache_layer.cpp:603) allocates each parent overview
tile at `fine.width() × fine.height()` — same pixel count (960×960 ≈ 11 MB) at every level.
On 2026-07-23, 45 accumulated overview tiles × 11 MB ≈ 500 MB consumed the 512 MiB budget
before phase-1 eviction could shed any fine tiles, so all 38 fine tiles were dropped → full
resolution collapse.

**#172**: Evicted fine tiles remain in reconciler possession (`markHave`) so the anti-entropy
loop never re-requests them. `warmLoad()` is the only disk-reload path (startup only). Once
evicted, a fine tile stays coarse for the rest of the session.

**Fix directions (operator confirmed 2026-08-20)**:
- #171: decimate overview pixel size via the uma shared fold engine; budget accounting unchanged.
- #172: implement the `hasUnloadedVisibleTiles` + snapshot-filtered reload worker hook that
  ADR-0013 §"camp#172 hook" deliberately left enabled-but-unimplemented.
- Nightly-regen anti-clobber / catalog-prune propagation → tracked follow-up issue (not this PR).

Steps 1–3 of world-store LOD are already landed: ADR-0011 visible-region rendering (PR#173),
uma shared fold engine `overview_builder.hpp` (uma ADR-0011 / PR#287), LOD renderer with the
reload seam (camp ADR-0013 / PR#183).

## Approach

### Step 1 — Amend camp ADR-0010 (governance first)

Edit `docs/decisions/0010-bounded-eviction-overview-pyramid.md`:
- D3: change from "match-resolution" to "decimated — parent tile is `⌊W/2⌋ × ⌊H/2⌋` of its
  child, minimum 1×1; a chain of 8 levels = ~1.33× one tile's footprint instead of 8×".
- D2 limitation: remove "no on-demand reload" — to be resolved by #172 in this PR.
- Add a D6 for the reload-hysteresis rule (reload only when `accountedBytes() <
  vram_budget_bytes_ × kReloadHysteresisFactor`; no ping-pong with eviction).
- Consequences: update memory math (45 overviews was ~500 MB; after fix chain ≈ 14.7 MB).

### Step 2 — Decimate overview tile sizes in `foldIntoParent()` (#171 fix)

`src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp`, `foldIntoParent()`:

Change the parent-tile construction from:
```cpp
Entry{SonarLiveTile(parent_index, fine.width(), fine.height()), …}
```
to:
```cpp
Entry{SonarLiveTile(parent_index,
                    std::max(fine.width() / 2, 1),
                    std::max(fine.height() / 2, 1)), …}
```

`foldChild()` (sonar_live_tile.cpp:130) already uses geographic center mapping — it maps each
child cell's center into the parent grid by lat/lon, accumulates finite/non-NoData values, and
averages (MEAN). This is the same algorithm as `overview_builder.hpp`'s `buildParentTile<float>`
(imagery = mean policy). The decimated parent size means ~4 child cells land in each parent
cell (2×2 in temperate bands): correct and consistent with uma ADR-0011. No change to
`foldChild()` needed.

Because `foldIntoParent()` recurses via `foldIntoParent(overview.tile)`, the next level
automatically gets half of 480→240→120→… The recursion terminates at `!parent_index.valid()`
(level 0). Warm-load reads width/height from the GeoTIFF via `loadFromGeoTiff()`, so
decimated overview files reload at their correct size.

Update the comment in `foldIntoParent()` to reference ADR-0010 D3 (amended).

### Step 3 — Implement the #172 on-demand reload hook

**3a. Track evicted indices** (`sonar_live_cache_layer.h` + `.cpp`):

Add `std::set<gggs::GridIndex> evicted_fine_indices_` (populated when a fine tile is erased
in `evictIfOverBudget()`, cleared when the tile is reloaded or a catalog prune removes it).

Add `bool hasEvictedVisibleTiles(const QRectF& viewport_scene) const`:
iterate `evicted_fine_indices_`, for each check `tileSceneRect(tile_index).intersects(viewport_scene)`.

**3b. Add reload worker** (`sonar_live_cache_layer.h`):

Add `QFutureWatcher<std::vector<SonarLiveTile>> reload_watcher_` and `QRectF reload_kicked_viewport_`.

Add `void kickReload(const QRectF& viewport)`: snapshots the set of visible evicted indices +
`cache_dir_`, launches a QtConcurrent worker that loads each from the `cache_dir_` fine-tile GeoTIFFs
(using `SonarLiveTile::loadFromGeoTiff()`), and connects the watcher's `finished` signal to
`onReloadFinished()`.

Add `void onReloadFinished()` (GUI thread slot): inserts loaded tiles back into `tiles_`, bumps
`access_seq_`, runs `evictIfOverBudget()` (maintains the budget; hysteresis prevents immediate
re-eviction), clears those indices from `evicted_fine_indices_`, updates display.

**3c. Trigger in `paint()`**:

After deriving the viewport clip (already done for `itemsIntersecting`), add:
```cpp
if(!reload_watcher_.isRunning() &&
   vram_budget_bytes_ > 0 &&
   accountedBytes() < vram_budget_bytes_ * kReloadHysteresisFactor &&
   hasEvictedVisibleTiles(clip.scene))
{
    kickReload(clip.scene);
}
```
`kReloadHysteresisFactor = 0.75`: reload only when resident < 75 % of budget, so the inserted
tiles don't immediately re-trigger eviction (ADR-0010 D6).

**3d. Lifecycle**: destructor joins the reload watcher (same pattern as `write_watchers_`).
`disableLiveCoverage()` cancels/joins it and clears `evicted_fine_indices_`.

### Step 4 — Update camp ADR-0013 §"camp#172 hook"

Change the status clause from "enabled, not implemented" to "implemented in #171/#172 PR" with
a one-line note on the hysteresis guard (ADR-0010 D6).

### Step 5 — Regression tests

**5a. `test/test_sonar_live_eviction.cpp` (extend existing)**:

Extend `WarmLoadTrimsToBudgetAndBuildsOverviews` to assert that after eviction, the total
overview tile footprint is substantially below the fine-tile footprint (e.g., `< budget / 4`).
This guards against regression to the same-size-tile behaviour.

Add a second test `EvictionHeadroomAtSurveyScale`: seed 38 fine tiles (matching 2026-07-23),
set budget to 512 MiB / 38 ≈ 13.5 MiB (one fine tile). Verify eviction leaves headroom:
`accountedBytes() <= budget`, and fine tiles remain resident (overview chain < budget).

**5b. `test/test_sonar_live_reload.cpp` (new)**:

`ReloadEvictedTilesOnViewportChange`: enable the layer with N fine tiles; trigger eviction; set
a fake viewport covering an evicted tile; drive the reload (call a `waitForReload()` seam or
trigger `paint()` with a forced viewport); verify the tile re-appears in `residentTileCount()`
and `overviewTileCount()` does not increase (reload replaces the overview for that region).

`ReloadHysteresisPreventsPingPong`: verify that after reload completes, a second eviction does
not immediately re-evict the just-reloaded tile (budget must remain below the hysteresis
threshold after the reload insert + evict cycle).

## Files to Change

| File | Change |
|------|--------|
| `docs/decisions/0010-bounded-eviction-overview-pyramid.md` | Amend D3 (decimated sizes), D2 (reload now implemented), add D6 (hysteresis rule), update Consequences memory math |
| `docs/decisions/0013-lod-level-selection-demand-driven-load.md` | Update §"camp#172 hook" status |
| `src/camp_map/ros/live_coverage/sonar_live_cache_layer.h` | Add `evicted_fine_indices_`, `reload_watcher_`, `reload_kicked_viewport_`; declare `hasEvictedVisibleTiles`, `kickReload`, `onReloadFinished` |
| `src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp` | `foldIntoParent()` decimated size; `evictIfOverBudget()` populate `evicted_fine_indices_`; `paint()` reload trigger; `kickReload`, `onReloadFinished`, `hasEvictedVisibleTiles`, destructor join |
| `test/test_sonar_live_eviction.cpp` | Extend headroom assertion; add survey-scale test |
| `test/test_sonar_live_reload.cpp` (new) | On-demand reload + hysteresis tests |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Fix it completely | Both halves (#171 size, #172 reload) land in one PR; tests cover both field-scale regression and hysteresis |
| Capture decisions | ADR-0010 D3 and D6 amended before implementation |
| Only what's needed | Scope is tight; nightly-regen anti-clobber is a tracked follow-up per operator directive |
| Test what breaks | Eviction-headroom and survey-scale tests guard against recurrence; hysteresis test guards against ping-pong |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| Camp ADR-0010 | Yes — D3 (overview size), D2 (reload), D1 (budget accounting unchanged) | Amended before implementation (Step 1) |
| Camp ADR-0013 | Yes — §"camp#172 hook" | Updated to "implemented" (Step 4) |
| Uma ADR-0011 | Yes — shared fold engine adoption | `foldChild()` confirmed equivalent to `buildParentTile<float>` (geographic center + MEAN); noted in code comment |
| Camp ADR-0006 | Watch — D4 GUI-thread-only | Reload worker snapshots before launch; insertion into `tiles_` on GUI thread via `onReloadFinished`; same invariant as `write_watchers_` |

## Consequences

| If we change… | Also update… | Included? |
|---|---|---|
| Overview tile sizes | `writeToGeoTiff` / `loadFromGeoTiff` (reads/writes GDAL raster of actual size — no change needed; dimensions come from tile) | N/A — no code change |
| `evictIfOverBudget()` eviction loop | `evicted_fine_indices_` must track erased tiles (Step 3a) | Yes |
| Reload path inserts into `tiles_` | `evictIfOverBudget()` must run after insert to maintain budget | Yes — in `onReloadFinished()` |
| `disableLiveCoverage()` | Must cancel/join reload watcher and clear evicted set | Yes |

## Documentation & Instruction Impact

- **Stale docs**: ADR-0010 D3 / D2 / Consequences, ADR-0013 §"camp#172 hook" — both in this PR.
- **Agent-instruction candidates**: None — the decimation formula and hysteresis factor go in the ADR and code; no new .agent/knowledge entry warranted.

## Open Questions

- None — fix direction, scope boundary (catalog-prune = follow-up), and hysteresis value (0.75) are operator-confirmed.

## Estimated Scope

Single PR closing #171 and #172.
