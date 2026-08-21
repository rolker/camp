# Plan: GggsTileLayer — viewport-scoped retention and residency budget

## Issue

https://github.com/rolker/camp/issues/195

## Context

Verified against the code: `GggsTileLayer` has exactly one release path —
`tilesReady()`'s loop that drops tiles at levels **finer** than
`selected_level_` once the selection's visible set has settled
(`gggs_tile_layer.cpp:641-716`). Nothing frees a tile because the view panned
away. `loadTilesWorker()` is viewport-filtered on the way *in*
(`:527-531`), so residency equals the union of every viewport ever visited.

Per-tile cost is high because `GggsTile::texture()` deliberately **retains** the
CPU buffer past the GPU upload (camp#180 cursor readout, `gggs_tile.cpp`): a
painted 960×960 tile holds 3.52 MiB of `std::vector<float>` **plus** a 3.52 MiB
R32F texture ≈ **7.03 MiB**. A hundred panned-over tiles is ~700 MiB — the same
shape that OOM'd salmon in camp#153 and produced `camp-ADR-0010`.

Two in-repo precedents bracket the fix:
`SonarLiveCacheLayer::evictIfOverBudget()` (byte budget, farthest-from-centre
with `last_access_seq` LRU fallback, `foldIntoParent()`, `kApexProtectLevel`,
`kReloadHysteresisFactor`), and `MapTiles::evictIfNeeded()` (camp#98 — a
**count** cap of `max(256, 4 × visible_count)`, deferred + debounced off
`paint()`, protecting the live visible set).

## Approach

Governed by `uma-ADR-0013` D4. Design decisions, with the reasoning the issue
asked to be checked rather than assumed:

1. **No fold-to-parent — plain drop-and-re-read.** `SonarLiveCacheLayer` folds
   because its tiles arrive once over ROS and must survive eviction; a GGGS
   tile's source of truth is the file on disk. `resetPixels()` + `releaseGL()`
   already is a complete release, and the existing demand-driven loader already
   reloads on pan-back (`hasUnloadedVisibleTiles()` + the
   moved-since-last-kick re-kick in `paint():996-1000`) — **the reload path is
   free**. Building a CAMP-side pyramid would also manufacture a derived
   product for `chart`/`reference`, which `uma-ADR-0010` D9 deliberately gives
   no overviews.
   *Cost:* an evicted area **blanks** until the reload lands, rather than
   degrading. Bought back cheaply by the apex-protect analogue in step 3.

2. **Structural current-frame protection** (`raster/tile_residency.h`, new,
   header-only + GL-free). A `TileResidency` type owns two `std::list<size_t>`
   partitions and an index of iterators: `beginFrame()` splices the whole
   protected list into the evictable list (O(1)); `protect(i)` splices one
   entry back (O(1)); `candidates()` returns **only** the evictable partition.
   The eviction routine has no access to the protected partition, so "cannot
   evict what this frame selected" is a property of the type, not an `if` a
   future edit can drop — D4's requirement, via the sentinel-splice pattern,
   without an intrusive list inside `tiles_`. Standalone-testable.

3. **Count budget, adaptive cap.** Tiles are a fixed 960×960, so count is a
   faithful RAM proxy (D4 blesses it; `cube_bathymetry`'s
   `test_tile_eviction_rss.cpp` uses the same proxy). Cap =
   `clamp(kMultiplier × protected_count, kMinCap, kMaxCap)` — MapTiles' form,
   which makes "budget ≥ current-frame working set" true *by construction*
   (D4). Proposed defaults `kMultiplier = 3`, `kMinCap = 32`,
   `kMaxCap = 64` (≈450 MiB worst case at 7.03 MiB/tile), overridable via
   `QSettings GggsTileLayers/max_resident_tiles` (`0` disables → pre-#195
   behaviour, mirroring `LiveTileCache/max_vram_bytes`). Tiles at the
   **coarsest available level** are never evicted (the `kApexProtectLevel`
   analogue): on a nested `overviews/` ladder that guarantees a zoom-out floor;
   on a single-level store it is a no-op and blanking stands (documented).

4. **Hybrid ordering, camp#150-safe.** Order candidates farthest-first by
   distance to the **nearest** recorded view centre, `last_visible_gen_` LRU
   as the fallback (headless / no view). Do **not** inherit
   `views().first()`: `paint()` receives the painting viewport as its `QWidget*
   widget` argument, so `deriveViewportClip()` gains an optional `widget`
   parameter and recovers the painting view via
   `qobject_cast<QGraphicsView*>(widget->parentWidget())`, falling back to
   `views().first()` exactly as today. Protection and view centres accumulate
   per `paint()` call; eviction is deferred through a queued, debounced
   `evictIfOverBudget()` (MapTiles pattern) so one pass sees the **union** of
   every view that painted. Single-view behaviour is unchanged; camp#150 does
   not invalidate the result.

5. **Hysteresis and volume caps.** Once triggered, evict down to
   `kEvictHysteresisFactor = 0.75 × cap` so the next frame cannot re-trigger
   (the `kReloadHysteresisFactor` analogue). The loader's existing
   moved-since-last-kick guard covers re-kick storms; add a per-kick bound so
   `loadTilesWorker()` stops after `cap` tiles — its pass is already
   ascending-by-level, so a truncated kick yields coarse coverage first.

6. **Degrade, don't thrash; never silently.** When the protected set alone
   exceeds `kMaxCap`, eviction cannot help — so relax the quality target:
   `pressure_bias_` steps the `selectLodLevel()` pick `bias` entries coarser in
   `available_levels_`, one-way ratchet with an N-frame cool-down before
   relaxing, and `setStatus("(reduced detail — tile budget)")`. Where no
   coarser level exists the bias is a no-op and the status says the layer is
   over budget instead. This closes D4's loop rather than reporting a number
   nobody acts on.

7. **Preserve camp#194's hole-coverage rule.** `tilesReady()` retains finer
   tiles that intersect a `loadFailed()` tile at a level ≤ the selection —
   the only usable coverage over that hole. Those tiles must be `protect()`ed
   in the new pass too, or the budget path silently undoes that guarantee.

8. **Extract `releaseTile()`.** The `releaseGL()`/`resetPixels()` pairing plus
   the `makeCurrent()`-or-skip dance in `tilesReady()` becomes one helper both
   release paths call. Both keep the GUI-thread assert and the
   `!future_watcher_.isRunning()` gate (they mutate tiles the worker iterates).

## Test Strategy

| Test | Asserts |
|---|---|
| `test/test_tile_residency.cpp` (new, pure) | `beginFrame()`/`protect()`/`candidates()` invariants: a protected index never appears in `candidates()`; splice is O(1)-shaped; re-protect is idempotent |
| `test/test_gggs_eviction.cpp` (new, headless, GL-free) | Synthetic tile dir + `setLodForTest()` walked across a strip far wider than the cap: resident count stays ≤ cap while visited count grows (the `test_tile_eviction_rss.cpp` bounded-count shape); no tile in the current `load_viewport_` is ever evicted; coarsest level survives; pan-back reloads the dropped tile (`pixelsLoadedCount()`); evict↔reload ping-pong does not occur across repeated A→B→A pans; a `loadFailed()`-covering finer tile is retained; `max_resident_tiles = 0` reproduces pre-#195 residency |
| `test/test_gggs_render.cpp` (extend) | Degrade path: with a cap below the visible set, the selection steps coarser and `status()` is non-empty |

Precedents: `test_sonar_live_eviction.cpp`, `test_sonar_live_reload.cpp`.
Register both new suites in `CMakeLists.txt` beside `test_gggs_rescan`.

## Files to Change

| File | Change |
|------|--------|
| `src/camp_map/raster/tile_residency.h` | **New.** Splice-partition residency helper (protected / evictable), header-only, GL- and Qt-light |
| `src/camp_map/raster/gggs_tile_layer.h` | Budget + residency + pressure-bias members; `evictIfOverBudget()` slot, `releaseTile()`, test seams (`residentTileCount()`, `setResidentBudgetForTest()`) |
| `src/camp_map/raster/gggs_tile_layer.cpp` | `paint()`: `beginFrame()`/`protect()`/record view centre/schedule eviction; pressure-bias LOD step; `evictIfOverBudget()`; `releaseTile()` extraction; loader per-kick cap; status reporting; ctor reads `QSettings` |
| `src/camp_map/raster/viewport_clip.h` | Optional `QWidget* widget` → prefer the painting view, `views().first()` fallback |
| `src/camp_map/raster/raster_layer.cpp`, `src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp` | Pass `widget` to `deriveViewportClip()` (call sites only) |
| `test/test_tile_residency.cpp`, `test/test_gggs_eviction.cpp` | **New** (see Test Strategy) |
| `test/test_gggs_render.cpp` | Degrade-path case |
| `CMakeLists.txt` | Register the two new gtest suites |
| `docs/decisions/0014-gggs-viewport-scoped-residency.md` | **New camp ADR** — the decision, cross-referencing `uma-ADR-0013` D4 and `camp-ADR-0010` |
| `docs/decisions/0013-lod-level-selection-demand-driven-load.md` | Amend: "levels ≤ the selection are never released" is no longer true; point at ADR-0014 |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Test what breaks | The failure is an OOM after prolonged panning — asserted as a bounded resident count under a synthetic pan walk, the portable proxy `cube_bathymetry` already uses |
| A change includes its consequences | ADR-0013's residency paragraph, `getElevation()` semantics, and the shared `deriveViewportClip()` signature are all in-plan, not follow-ups |
| Capture decisions, not just implementations | New camp ADR-0014 records the fold-vs-drop and count-vs-bytes reasoning; without it the next agent re-derives it |
| Only what's needed | No fold-to-parent, no ResourceMonitor dependency, no prefetch (`uma-ADR-0013` D6 explicitly wants measurement first) |
| Improve incrementally | A fixed sane default now; the measured budget arrives via #155/#156 through the same `QSettings` seam |
| Enforcement over documentation | Current-frame protection is enforced by `TileResidency`'s type boundary, not by a comment |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| `uma-ADR-0013` D4 | Yes | Budget + structural current-frame protection + hybrid ordering + degrade-not-thrash, all implemented; count units explicitly blessed for fixed 960×960 tiles |
| `uma-ADR-0013` D5 | Yes | Ascending (zoom-out) backdrop retention is untouched; eviction never targets the current frame, so "the picture never gets worse" holds except across a true pan-away |
| `uma-ADR-0013` D6 | No | No prefetch — the ADR requires latency measurement first |
| `uma-ADR-0013` D8 | No | `getElevation()` is a display readout, not a safety query; `shallowestReliable()` is not in this layer |
| `uma-ADR-0010` D9 | Yes (respected) | No CAMP-side pyramid is manufactured for `chart`/`reference`; the coarse-zoom blank stays a store-side question |
| `camp-ADR-0010` | Yes | Same forces, different layer; divergences (drop vs fold, count vs bytes) are recorded in the new ADR-0014 |
| `camp-ADR-0011` | Yes | `deriveViewportClip()` gains a parameter; the render convention itself is unchanged |
| `camp-ADR-0013` | Yes | Amended — the "levels ≤ selection are never released" residency rule is superseded |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| The residency rule | `docs/decisions/0013-...md` + new ADR-0014 | Yes |
| `deriveViewportClip()` signature | `RasterLayer`, `SonarLiveCacheLayer` call sites | Yes |
| Tile residency | `getElevation()` returns NaN over evicted (off-screen) area | Yes — noted in ADR-0014; the cursor is always in-viewport, so protected |
| Tile residency | Auto-range: deliberately **not** recomputed on evict (the fold is widening-only), so the colormap does not flicker as tiles come and go | Yes — stated in ADR-0014 |
| Per-layer budget | N GGGS layers multiply the process footprint | No — process-wide accounting is #155/#156; ADR-0014 records the gap |
| Per-frame overdraw (issue comment) | Composite-depth capping / occlusion skip | No — separate axis; the budget bounds it only indirectly. Follow-up issue |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `docs/decisions/0013-lod-level-selection-demand-driven-load.md` (residency rule superseded); new `docs/decisions/0014-gggs-viewport-scoped-residency.md`; `.agents/README.md` — add the residency budget to the GGGS-layer notes if that section names the load model.
- **Agent-instruction candidates** (proposals only): none new — the reusable
  lesson ("a viewport-filtered *loader* is not a residency bound") belongs in
  ADR-0014, which is repo-tracked and discoverable.

## Open Questions

- Confirm the cap defaults (`kMultiplier = 3`, `kMinCap = 32`, `kMaxCap = 64`
  ≈ 450 MiB worst case) and the settings key `GggsTileLayers/max_resident_tiles`.
- Confirm the degrade lever (step 6) belongs in this PR rather than as a
  follow-up — it is the part most likely to oscillate in the field.
- Confirm the `deriveViewportClip()` `widget` parameter (touching two other
  layers' call sites) is acceptable scope here, versus a separate camp#150 PR.

## Estimated Scope

Single PR, ~5 atomic commits: (1) `TileResidency` + its test, (2)
`deriveViewportClip` painting-view recovery + call sites, (3) `releaseTile()`
extraction, (4) budget/eviction/hysteresis + test, (5) degrade lever + ADRs.
