# Plan: Live-coverage display collapses to low resolution during eviction (#171 + #172)

## Issue

https://github.com/rolker/camp/issues/171 (closes #171, closes #172)

## Context

During the 2026-07-23 BizzyBoat M3 survey, CAMP's live coverage collapsed to low resolution
mid-session and stayed there for the rest of the session. Root cause (verified in source):

**#171 (folding memory)**: `foldIntoParent()` (`sonar_live_cache_layer.cpp:589`) already builds
each overview parent at the fine tile's own `width × height` — i.e. the **fixed uniform
`TiledRasterTile::edge` (960²)** size, matching the uma shared fold engine
(`overview_builder.hpp::buildParentTile`, which builds every parent at the fixed
`TiledRasterTile<T>::edge`). `SonarLiveTile::foldChild()` area-maps a child into a **¼
sub-window** of that same-size parent (2×2 children per parent) and averages (MEAN) — the
standard half-resolution-per-level pyramid. This geometry is a **hard invariant** of
`foldChild()`: it asserts/requires `parent.width == child.width` (sonar_live_tile.h:66-76,
sonar_live_tile.cpp:130-195). The original diagnosis ("folding frees almost no memory") was
the pessimistic view: for a *linear* lawnmower survey few fine tiles complete a 2×2 block, so
near the fine level the collapse ratio is closer to 2→1 than 4→1, and the chain recurses to
level 0, so the overview pyramid grew large (field: ~45 overview tiles ≈ 500 MB against the
512 MiB budget). Phase-1 eviction then shed fine tiles until the budget was met, and with no
reload path the display stayed coarse.

**#172 (no reload)**: Evicted fine tiles remain in reconciler possession (`markHave`) so the
anti-entropy loop never re-requests them, and `warmLoad()` is the only disk-reload path
(startup only). Once evicted, a fine tile stays coarse for the rest of the session — this is
what made the field collapse *permanent*. ADR-0013 §"The camp#172 hook" deliberately left the
`hasUnloadedVisibleTiles()` + snapshot-filtered worker seam **enabled-but-unimplemented**.

**Operator checkpoint decisions (2026-08-20, authoritative — not re-opened):**

1. **Pyramid geometry = the uma fold engine AS-IS.** Fixed uniform `TiledRasterTile::edge`
   parents, standard half-resolution-per-level pyramid, MEAN fold policy — **true convergence**
   with the merged store pyramids (identical fidelity at the same zoom). This is what the
   shipped `foldIntoParent()` already does; the earlier plan's camp-specific `⌊W/2⌋`
   quarter-resolution decimation is **DROPPED** (it would also have violated `foldChild()`'s
   same-size invariant). So #171 carries **no fold-geometry code change** — the ADR-0010
   amendment records the shipped geometry as *convergence* (not a new camp divergence), and the
   session-long collapse is remedied by #172's on-demand reload.
2. **Catalog-prune propagation / nightly-regen anti-clobber → tracked follow-up issue**, NOT
   this PR. The host files it at the publish checkpoint. This PR's `handleCatalog()` prune only
   needs to keep the new evicted-index set consistent (erase a pruned index — Step 3a).

Steps 1–3 of world-store LOD are already landed: ADR-0011 visible-region rendering (PR#173),
uma shared fold engine `overview_builder.hpp` (uma ADR-0011 / PR#287), LOD renderer with the
reload seam (camp ADR-0013 / PR#183). This PR is step 4: the live coverage cache adopts the
shared pyramid machinery (convergence) and lights up the reload seam.

## Approach

### Step 1 — Amend camp ADR-0010 (governance first)

Edit `docs/decisions/0010-bounded-eviction-overview-pyramid.md`:

- **D3 — reframe as convergence, not a geometry change.** Retitle from "match-resolution …"
  to record that the shipped overview geometry (parent at the fine tile's `width×height` = the
  fixed uniform `TiledRasterTile::edge`, ¼-sub-window MEAN fold, half-resolution per level) **is
  the uma shared fold engine's geometry** — identical fidelity to the merged store pyramid at
  the same zoom (`overview_builder.hpp::buildParentTile` folds every parent at the fixed
  `TiledRasterTile<T>::edge` with the MEAN cell policy for imagery). Note the same-size
  parent/child requirement is a `foldChild()` invariant. No pixel-size change; this is a
  documentation convergence, closing the "confirm aligned or note divergence" action from the
  Issue Review.
- **"Alternatives considered" reconciliation.** The existing "rejected fixed-64×64 in favour of
  matching fine dimensions (standard pyramid, uniform tile size)" entry is **retained and now
  reinforced** — uniform tile size is exactly the convergence choice, so it is **not** a
  reversal. Add a sentence tying it to the uma fold engine so no stale contradiction remains.
- **D2 limitation removed.** Delete "no on-demand reload / deferred follow-up" — #172 implements
  it in this PR. Update the Consequences "Deferred follow-ups" bullet accordingly (reload is
  done; the catalog-prune overview-lifecycle bullet stays, now cross-referenced to the tracked
  follow-up issue per operator decision 2).
- **Add D6 — reload hysteresis.** Reload fires only when `accountedBytes() < vram_budget_bytes_
  × kReloadHysteresisFactor` (0.75) so a reload insert cannot immediately re-trigger eviction
  (no ping-pong with D1). State it as a rule, not just a code comment.
- **Consequences — memory math.** State the fixed-edge series: total pyramid memory converges to
  the **1.33× series** (1 + ¼ + 1/16 + … = 4/3 of one level) because 4 fine tiles collapse into
  1 uniform parent; eviction therefore frees **real** memory (4 same-size fine → 1 same-size
  parent). Note the linear-survey caveat (closer to a 2× series where 2→1 dominates) and that
  the protected apex + evictable near-fine overviews keep total residency bounded.
- **On-disk migration.** Add a one-line note: because the geometry is **unchanged** from the
  shipped D3 (uniform edge, `overviews/<level>_<row>_<col>.tif` sidecar layout — already the uma
  layout), **no on-disk migration is required**; existing overview files remain valid and
  warm-load unchanged. (Overviews are display-grade and self-heal on the next eviction/regen
  regardless.)

### Step 2 — #171: confirm convergence in code (no geometry change)

`src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp`, `foldIntoParent()`:

- **No change to the parent-tile construction** — `SonarLiveTile(parent_index, fine.width(),
  fine.height())` stays (uniform `TiledRasterTile::edge`; required by `foldChild()`).
- Update the `foldIntoParent()` comment to cite ADR-0010 D3 (amended) and state the convergence
  explicitly: same geometry + MEAN policy as `overview_builder.hpp::buildParentTile`.
- Update the `foldChild()` comment in `sonar_live_tile.cpp` to name the uma equivalence
  (geographic-centre area-map + MEAN = `buildParentTile<float>` imagery policy).

That is the whole of the #171 code delta. The memory relief comes from the (already-correct)
4→1 uniform-parent collapse; the *display* relief for the field collapse comes from Step 3.

### Step 3 — Implement the #172 on-demand reload hook

Template: the sibling demand-driven loader in `GggsTileLayer`
(`gggs_tile_layer.cpp:296-382,428-449,469-490,703-715`) — it already carries the
moved-since-last-kick guard (`last_kick_viewport_`), the snapshot-filtered async worker, and the
`waitForLoad()` headless seam. The reload path mirrors it (not merely `write_watchers_`).

**3a. Track evicted indices** (`sonar_live_cache_layer.h` + `.cpp`):

- Add `std::set<gggs::GridIndex> evicted_fine_indices_`.
- Populate it in `evictIfOverBudget()` **phase 1** for every fine index erased.
- Erase from it in `handleCatalog()`'s prune loop (`sonar_live_cache_layer.cpp:460-476`): a
  pruned index whose GeoTIFF is `fs::remove`d must NOT linger in the set, or `kickReload` would
  snapshot it → `loadFromGeoTiff` fails → index never cleared (review must-fix #3).
- Erase from it in `handleTile()` when the boat re-sends a tile for that index (it is resident
  again — no longer evicted).

**3b. `hasUnloadedVisibleTiles()` seam** (ADR-0013 name; review suggestion #6):

`bool hasUnloadedVisibleTiles(const QRectF& viewport_scene) const` — iterate
`evicted_fine_indices_`; for each compute its scene rect from the GGGS geographic extent of the
index (`web_mercator::geoToMap` of `index.westLongitude()/southLatitude()/…`, no live tile
needed) and return true on the first intersect with `viewport_scene`.

**3c. Reload worker** (`sonar_live_cache_layer.h`):

- Add `QFutureWatcher<std::vector<SonarLiveTile>> reload_watcher_` and `QRectF
  last_reload_viewport_` (the moved-since-kick guard, mirroring `last_kick_viewport_`).
- `void kickReload(const QRectF& viewport_scene)`: snapshot the set of **visible** evicted
  indices + `cache_dir_` + `*level_` into value copies; record `last_reload_viewport_ =
  viewport_scene`; launch a self-contained `QtConcurrent::run` worker that loads each index's
  fine GeoTIFF from `cache_dir_` via `SonarLiveTile::loadFromGeoTiff()` and returns the loaded
  vector; connect `reload_watcher_.finished` → `onReloadFinished()`. Worker never touches
  `this` (same contract as `writeTileToCache`).
- `void onReloadFinished()` (GUI-thread slot): read `reload_watcher_.future().result()`; for
  each **attempted** snapshot index, erase it from `evicted_fine_indices_` **whether or not the
  load succeeded** (a permanently-unloadable index must not keep `hasUnloadedVisibleTiles` true
  forever — review must-fix #4); insert each successfully-loaded tile into `tiles_` (bump
  `access_seq_`, `texture_dirty=true`); run `recomputeBounds()/resetAutoRange()/foldAutoRange()`;
  run `evictIfOverBudget()` (D6 hysteresis keeps it from immediately re-evicting the inserts);
  invalidate `cached_image_` and repaint. Snapshot the attempted set on the watcher (member
  `reload_attempted_` or capture via the future payload carrying its own index) so
  onReloadFinished knows which to clear.

**3d. Trigger in `paint()`** (`sonar_live_cache_layer.cpp:963`):

After deriving `clip` (already computed for the render), add the guarded kick — mirroring
`GggsTileLayer::paint()` (gggs_tile_layer.cpp:711-715):
```cpp
if(!reload_watcher_.isRunning() &&
   vram_budget_bytes_ > 0 &&
   accountedBytes() < vram_budget_bytes_ * kReloadHysteresisFactor &&
   clip.scene != last_reload_viewport_ &&          // moved-since-last-kick guard
   hasUnloadedVisibleTiles(clip.scene))
{
    kickReload(clip.scene);
}
```
`kReloadHysteresisFactor = 0.75` (ADR-0010 D6). The `clip.scene != last_reload_viewport_` guard
means a permanently-unloadable index can re-kick at most once per distinct viewport, not every
frame.

**3e. Headless seam** `void waitForReload(const QRectF& viewport_scene)` mirroring
`GggsTileLayer::waitForLoad()`: if idle and `viewport_scene != last_reload_viewport_` and
`hasUnloadedVisibleTiles(viewport_scene)`, `kickReload(viewport_scene)`; then
`reload_watcher_.waitForFinished(); onReloadFinished();`. Lets the reload test drive the load
deterministically without a scene/view (paint is never called headlessly).

**3f. Lifecycle**: the destructor and `disableLiveCoverage()` must cancel/join `reload_watcher_`
(`reload_watcher_.waitForFinished()` after setting `shutting_down_`), and `disableLiveCoverage()`
clears `evicted_fine_indices_` and `last_reload_viewport_`. Guard `onReloadFinished()` early-out
on `shutting_down_`.

### Step 4 — Update camp ADR-0013 §"The camp#172 hook"

Change the status from "enabled, not implemented" to "implemented in the #171/#172 PR": the
`SonarLiveCacheLayer` reload seam reuses the `hasUnloadedVisibleTiles()` + snapshot-filtered
worker pattern from `GggsTileLayer`, with the D6 hysteresis guard (reload only under 0.75×
budget) to avoid ping-pong with eviction. Note #171 rides the same structure and confirms the
shared-fold convergence.

### Step 5 — Regression tests

**5a. Per-pool byte-accounting seam** (`sonar_live_cache_layer.h`; review suggestion #8):
factor the `tileBytes` lambda in `accountedBytes()` into a helper and expose
`std::size_t fineResidentBytes() const` and `std::size_t overviewResidentBytes() const`
(introspection, like `residentTileCount()`/`overviewTileCount()`). The eviction-headroom test
needs the overview-pool byte total; `accountedBytes()` (total) alone can't express "the pyramid
stays bounded well under budget."

**5b. `test/test_sonar_live_eviction.cpp` (extend)**:

- Extend `WarmLoadTrimsToBudgetAndBuildsOverviews` to assert `overviewResidentBytes()` is a
  bounded fraction of the budget (the pyramid must not itself consume the whole budget —
  guards the field failure mode).
- Add `EvictionHeadroomAtSurveyScale`: seed a realistic cluster of fine tiles, set a budget
  that comfortably holds the pyramid plus several fine tiles, warm-load+evict, and assert:
  `accountedBytes() <= budget`, `residentTileCount() > 0` (eviction does **not** shed *all*
  fine tiles), and `overviewResidentBytes() < budget` (pyramid bounded). This is the
  realistic-scale eviction-headroom regression from the Issue Review.

**5c. `test/test_sonar_live_reload.cpp` (new, registered in CMakeLists — 5d)**:

- `ReloadEvictedTilesOnViewportChange`: enable the layer with N fine tiles on disk; force
  eviction (small budget) so some fine tiles are evicted (assert `evicted_fine_indices_`
  non-empty via `residentTileCount()` drop + `overviewTileCount() > 0`); RAISE the budget above
  the resident footprint (so D6 hysteresis permits reload); call
  `waitForReload(<scene rect covering an evicted tile>)`; assert `residentTileCount()`
  increases (the evicted fine tile is back) and `overviewTileCount()` does not increase (reload
  adds fine, not overviews).
- `ReloadHysteresisPreventsPingPong`: after a reload completes, assert a subsequent
  `evictIfOverBudget()`/paint cycle does not immediately re-evict the just-reloaded tile while
  resident < 0.75× budget (residency stays below the hysteresis threshold). Drive a second
  `waitForReload` on the same viewport and assert it does **not** re-kick (guarded by
  `last_reload_viewport_`), proving no per-frame churn.

**5d. Register the new gtest in `CMakeLists.txt`** via `ament_add_gtest(test_sonar_live_reload
…)` with the same `target_include_directories` / `ament_target_dependencies` /
`target_link_libraries` block as `test_sonar_live_eviction` (review must-fix #5 — an
unregistered gtest never runs).

## Files to Change

| File | Change |
|------|--------|
| `docs/decisions/0010-bounded-eviction-overview-pyramid.md` | Reframe D3 as uma convergence (no geometry change); reconcile "Alternatives considered"; remove D2 reload limitation; add D6 hysteresis; Consequences 1.33× memory math + "no on-disk migration" note |
| `docs/decisions/0013-lod-level-selection-demand-driven-load.md` | §"The camp#172 hook" → "implemented" (hysteresis note) |
| `src/camp_map/ros/live_coverage/sonar_live_tile.cpp` | `foldChild()` comment: name the uma `buildParentTile` equivalence (no code change) |
| `src/camp_map/ros/live_coverage/sonar_live_cache_layer.h` | Add `evicted_fine_indices_`, `reload_watcher_`, `last_reload_viewport_`, attempted-snapshot member; declare `hasUnloadedVisibleTiles`, `kickReload`, `onReloadFinished`, `waitForReload`, `fineResidentBytes`, `overviewResidentBytes` |
| `src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp` | `foldIntoParent()` comment (convergence); `evictIfOverBudget()` populate `evicted_fine_indices_`; `handleCatalog()` prune erases evicted index; `handleTile()` clears re-sent index; `paint()` reload trigger (guarded); `kickReload`/`onReloadFinished`/`hasUnloadedVisibleTiles`/`waitForReload`; `accountedBytes()` refactor + per-pool seams; destructor + `disableLiveCoverage()` join/cancel |
| `test/test_sonar_live_eviction.cpp` | Extend overview-bytes assertion; add `EvictionHeadroomAtSurveyScale` |
| `test/test_sonar_live_reload.cpp` (new) | On-demand reload + hysteresis tests |
| `CMakeLists.txt` | `ament_add_gtest(test_sonar_live_reload …)` |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Fix it completely | #171 (convergent bounded pyramid — already correct geometry, documented) + #172 (reload restores detail) land together; tests cover survey-scale headroom and hysteresis |
| Capture decisions | ADR-0010 D3 reframed as convergence + D6 hysteresis before implementation; ADR-0013 hook marked implemented |
| Only what's needed | No geometry code change (the shipped uniform-edge fold is already the convergent scheme); catalog-prune propagation is a tracked follow-up per operator decision 2 |
| Test what breaks | Eviction-headroom + reload/hysteresis tests guard the field failure and its remedy |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| Camp ADR-0010 | Yes — D3 (convergence framing), D2 (reload), D6 (new hysteresis), D1 (budget accounting unchanged) | Amended before implementation (Step 1) |
| Camp ADR-0013 | Yes — §"camp#172 hook" | Updated to "implemented" (Step 4) |
| Uma ADR-0011 | Yes — shared fold engine convergence | `foldChild()` confirmed equivalent to `buildParentTile<float>` (geographic-centre area-map + MEAN, fixed uniform edge); documented, no divergence |
| Camp ADR-0006 | Watch — D4 GUI-thread-only | Reload worker snapshots before launch; insertion into `tiles_` on the GUI thread via `onReloadFinished`; same invariant as `write_watchers_` and the `GggsTileLayer` loader |

## Consequences

| If we change… | Also update… | Included? |
|---|---|---|
| Overview geometry | Nothing — geometry is unchanged (convergence documentation only); existing `overviews/*.tif` remain valid | N/A — no migration |
| `evictIfOverBudget()` fine-tile eviction | `evicted_fine_indices_` must record every erased fine index (Step 3a) | Yes |
| `handleCatalog()` prune | Must erase the pruned index from `evicted_fine_indices_` (Step 3a) | Yes |
| Reload path inserts into `tiles_` | `evictIfOverBudget()` must run after insert; D6 hysteresis prevents immediate re-eviction | Yes — in `onReloadFinished()` |
| `disableLiveCoverage()` / destructor | Must cancel/join `reload_watcher_` and clear `evicted_fine_indices_` | Yes |

## Documentation & Instruction Impact

- **Stale docs**: ADR-0010 D3 / D2 / Consequences (convergence + hysteresis + migration note),
  ADR-0013 §"camp#172 hook" — both in this PR.
- **Agent-instruction candidates**: None — the convergence rationale and the hysteresis factor
  live in the ADR and code; no new `.agent/knowledge` entry warranted.

## Open Questions

- None — geometry decision (convergence, no `⌊W/2⌋`), scope boundary (catalog-prune propagation
  = tracked follow-up), and the hysteresis value (0.75) are operator-confirmed.

## Implementation Divergences (recorded at implementation time)

Minor deltas from the plan above, all verified against a green build + test run:

- **Eviction-headroom test scale.** The Issue-Review sketch (38 fine tiles, budget =
  512 MiB / 38 ≈ one fine tile) cannot demonstrate *headroom*: at that scale the overview
  pyramid (with its protected apex + the full chain to level 0) is comparable to the
  budget, so eviction correctly collapses to overviews — which is the LOD design, remedied
  by #172 reload, not a bug. The headroom regression instead seeds a **contiguous 10×10
  GGGS block** (so parents actually share 4→1) against a budget **above** the bounded
  pyramid, proving eviction leaves real fine-tile headroom (measured: pyramid 43 tiles vs
  70-tile budget → 27 fine tiles resident). A sparse lat/lon scatter (the first attempt)
  does *not* share coarse parents and inflates the pyramid past the budget — contiguity is
  essential to the test and to the convergent-pyramid memory claim.
- **Per-pool byte assertion.** Plan Step 5a's "overview footprint < budget/4" only holds
  when the budget dwarfs the apex floor; at the existing test's deliberately tiny 4-tile
  budget the protected apex alone can exceed it. So the tiny-budget test asserts per-pool
  seam *consistency* (`fine + overview == accounted`, both > 0), and the bounded-pyramid
  assertion lives in the survey-scale headroom test where the budget > pyramid.
- **`waitForReload()` mirrors the full `paint()` gate** (including the D6 hysteresis), so
  the `ReloadHysteresisPreventsPingPong` test can exercise the guard headlessly. Added a
  test-only `setResidentBudgetForTest()` seam and an `evictedFineCount()` introspection for
  the reload tests; `accountedBytes()` was made public (it is the sum of the two public
  per-pool seams).
- **No `sonar_live_tile.cpp` code change** — only the `foldChild()` comment gained the uma
  `buildParentTile` convergence note, as planned.

## Estimated Scope

Single PR closing #171 and #172.
