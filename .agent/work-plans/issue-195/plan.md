# Plan: GggsTileLayer — viewport-scoped retention and residency budget

## Issue

https://github.com/rolker/camp/issues/195

**Revised 2026-08-21** after the plan review (`progress.md` § Plan Review,
`66c5f76`, 14 findings) and the operator decisions taken on it. The revision is
recorded inline throughout; the four decisions are summarised in
"Decisions applied to this revision" below so the delta from `b70311f` stays
readable.

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
`paint()`, protecting the live visible set, with the eviction pass recomputing
the protected set **live** rather than trusting the one captured at schedule
time). `test/test_map_tiles_eviction.cpp` is the in-repo count-budget test
precedent; `cube_bathymetry`'s `test_tile_eviction_rss.cpp` is the out-of-repo
one.

## Decisions applied to this revision

1. **Budget is a `QSettings` byte target, default 512 MiB**, converted to a tile
   count from the *observed* tile size. `width_`/`height_` come from GDAL
   (`gggs_tile.cpp:70-71,96-97`), so 960×960 is a store convention, not a
   guarantee, and a hardcoded count would silently mis-size on any other store.
   The key mirrors `LiveTileCache/max_vram_bytes` (`sonar_live_cache_layer.cpp:150-163`),
   including `0 = disabled`.
   *Rationale for 512 MiB beside the live cache's own 512 MiB*: the camp#153 OOM
   precedent is **salmon**'s hardware, and **the Shoals operator station is
   pandy, not salmon**. A 512 MiB display working set alongside the live cache's
   512 MiB is acceptable on pandy; the settings key exists precisely so the
   number can be tuned on the host during the rebuild day rather than requiring
   a rebuild.
2. **The cap is floored at the protected (current-frame) set.** A ceiling that
   can fall below the working set is D4's "thrashes by construction" (review
   finding 6). The floor is paired with a **non-silent over-budget status**, so
   D4's "never fail silently" holds without the degrade lever.
3. **The `pressure_bias_` / degrade-under-pressure lever is deferred entirely**
   to **camp#197** (review finding 9: as specified it oscillates with a period
   equal to its cool-down). camp#155/#156 are named there as the measured-pressure
   input.
4. **The `deriveViewportClip()` signature change is dropped** (review finding 8).
   `clip.scene.center()` is already computed every `paint()`, and with CAMP's
   single MapView it *is* the painting view's centre. Painting-view recovery
   stays camp#150's.

## Approach

Governed by `uma-ADR-0013` D4.

1. **No fold-to-parent — plain drop-and-re-read.** `SonarLiveCacheLayer` folds
   because its tiles arrive once over ROS and must survive eviction; a GGGS
   tile's source of truth is the file on disk. `resetPixels()` + `releaseGL()`
   already is a complete release, and the existing demand-driven loader reloads
   on pan-back (`hasUnloadedVisibleTiles()` + the moved-since-last-kick re-kick
   in `paint():996-1000`) — **the reload path is free for every tile at a level
   ≤ the selection**. Building a CAMP-side pyramid would also manufacture a
   derived product for `chart`/`reference`, which `uma-ADR-0010` D9 deliberately
   gives no overviews.
   *Cost:* an evicted area **blanks** until the reload lands, rather than
   degrading — bought back by the coarsest-level exemption in step 4, and, in
   the ladder case, by the fact that every level ≤ the selection is resident, so
   a panned-away area retains coarse coverage.
   *Exception (review finding 4):* a **hole-covering** tile at a level **finer**
   than the selection is NOT reloadable — `loadTilesWorker()` skips
   `tile->level() > level` (`:527`). See step 5.

2. **Structural current-frame protection** (`raster/tile_residency.h`, new,
   header-only, Qt- and GL-free). `TileResidency` owns two `std::list<size_t>`
   partitions plus an index of iterators and a per-index frame epoch:
   `beginFrame()` splices the whole protected list into the evictable list and
   bumps the epoch (O(1)); `protect(i)` splices one entry back (O(1), idempotent
   within a frame); `candidates()` returns **only** the evictable partition. The
   eviction routine has no access to the protected partition, so "cannot evict
   what this frame selected" is a property of the type, not an `if` a future
   edit can drop — D4's sentinel-node requirement. Standalone-testable.

3. **Protection comes from the loader predicate, not the draw list**
   (review finding 1 — must-fix). `itemsIntersecting()` is short-circuited by
   `cached_image_` on a static frame (`paint():1009-1015`), so a draw-list-sourced
   `protect()` would protect **nothing** on exactly the frames eviction runs on,
   and the live visible set would become evictable. The predicate is the
   loader's own: *tile intersects `clip.scene`* **and** *level ≤ `selected_level_`*
   (plus the hole-coverage extension of step 5), evaluated over `tiles_`
   irrespective of load state — a not-yet-loaded selected tile is part of the
   working set the cap must accommodate. `loadFailed()` tiles are excluded (they
   never become resident, so they must not inflate the floor).
   **Placement** (review finding 2 — must-fix): `beginFrame()`/`protect()` run
   in `paint()` immediately after the LOD selection and **before** the
   `data_min_ > data_max_` early return at `:1002`. The `tiles_.empty()` return
   at `:942` is vacuous (nothing to protect or evict). The eviction pass
   **re-runs** the same predicate live before choosing candidates, MapTiles-style,
   so a tile that re-entered the view between scheduling and running is never
   evicted — and so protection cannot go stale across the debounce.

4. **Byte budget → count cap, floored at the working set.**
   `GggsTileLayers/max_resident_bytes`, default 512 MiB, `0` disables (pre-#195
   behaviour). Per-tile resident bytes are observed from the tile-set
   (`width × height × 4`, doubled when a GL texture may also be held), so the
   count adapts to whatever GDAL reports. Then
   `cap = max(budget_bytes / per_tile_bytes, protected_count)` — the floor is
   what makes "budget ≥ current-frame working set" true by construction (D4);
   `protected_count > budget_tiles` sets the **over-budget status** instead of
   thrashing (decision 2). Eviction runs down to
   `max(protected_count, 0.75 × cap)` (the `kReloadHysteresisFactor` analogue)
   so the next frame cannot immediately re-trigger.
   **Coarsest-level exemption, bounded** (review finding 11): tiles at
   `available_levels_.front()` are exempt as the zoom-out floor — but only when
   the ladder has more than one level (on a single-level store every tile would
   otherwise be exempt and the budget would be a no-op), and only for the
   `kCoarsestExemptCap` nearest such tiles; the surplus, which is what grows with
   the area panned, becomes a **last-resort** candidate rather than an exemption.
   `available_levels_` is rebuilt by `rescan()`, so the exemption is evaluated
   per pass, never cached.

5. **camp#194 hole coverage is preserved, and its reload gap is handled
   explicitly** (review finding 4 — must-fix). The predicate "this finer tile
   intersects a `loadFailed()` tile at a level ≤ the selection" is extracted into
   one helper shared by `tilesReady()`'s release loop and the budget pass, so the
   two rules cannot drift. A hole-coverer **in the current viewport** is
   protected like any selected tile — it is the only usable coverage there.
   A hole-coverer **outside** the viewport is a *last-resort* candidate: evicted
   only when nothing else can be, because it genuinely cannot reload
   (`loadTilesWorker()` skips levels > the selection). When one is dropped the
   layer says so in its status ("coverage over failed tile(s) released — use
   Rescan"), so the loss is visible and the operator has the recovery action.
   *Alternative considered and rejected:* extending the loader ceiling to admit
   finer tiles over a failed footprint would make them reloadable, but a failed
   **coarse** tile spans an enormous footprint, so this would pull in every fine
   tile in the viewport at low zoom — reintroducing the store-bounded load
   `camp-ADR-0013` exists to avoid, and inflating the protected floor with it.

6. **Eviction is deferred, debounced, and re-arms when the worker is busy**
   (review finding 3 — must-fix). `paint()` only *schedules*
   `evictIfOverBudget()` (queued invocation, `eviction_pending_` debounce);
   releasing tiles inside `paint()` would mutate the set the render pass reads.
   The slot must not mutate tiles the loader worker is iterating, so it checks
   `future_watcher_.isRunning()` — and when it finds the worker busy it
   **re-arms** on a short timer with `eviction_pending_` still set, instead of
   dropping the request. Without the re-arm the budget would almost never fire
   during a continuous pan, because every pan step re-kicks the loader — the
   exact scenario the budget exists for. Abort+join (what `loadTiles()` does)
   is rejected here: it would stall the GUI thread mid-pan for a whole tile read.

7. **Hybrid eviction ordering with a staleness term** (review finding 12).
   Candidates are ordered by, in priority: *last-resort tier* (out-of-view
   hole-coverers and surplus-coarsest last), then *staleness* (a tile not seen
   in the last `kRecentGenerations` paints goes first), then *farthest from the
   viewport centre*, then LRU generation. The staleness tier is what keeps
   distance from being primary — D4's stated objection is that distance-only
   "discards history along a path being traversed", and a tile seen a few frames
   ago along a pan track sorts behind one that has not been seen at all.
   The viewport centre is `load_viewport_.center()` — `clip.scene`'s centre,
   recorded by `paint()` (decision 4). A null viewport (headless) leaves pure
   LRU, exactly as `SonarLiveCacheLayer` degrades.

8. **One status composer** (review finding 5 — must-fix). `tilesReady()`
   currently rewrites `setStatus()` unconditionally (`:732-739`, clearing to
   `""`), which would wipe any over-budget message written from `paint()`.
   All status writes move into `updateStatus()`, which composes from state —
   no tiles / loading / no data / failed-tile count / over budget / hole coverage
   released — and every writer calls it instead of `setStatus()`.

9. **No per-kick loader volume cap** (review finding 7 — must-fix; the original
   plan's step 5 is withdrawn). The re-kick guard requires `selected_level_` or
   `load_viewport_` to change (`:996-999`), so a truncated kick with a stationary
   operator would never complete and `tilesReady()` would settle with a cleared
   status over a permanently incomplete picture. The kick set is already
   viewport-bounded — it *is* the protected set — so the cap bought nothing.

10. **Extract `releaseTiles()`.** The `releaseGL()`/`resetPixels()` pairing plus
    the `makeCurrent()`-or-skip dance in `tilesReady()` becomes one helper both
    release paths call, so the pairing invariant (`gggs_tile.h`: a CPU-only clear
    leaves a stale texture shadowing any re-load) is enforced in one place.

## Test Strategy

| Test | Asserts |
|---|---|
| `test/test_tile_residency.cpp` (new, pure) | `beginFrame()`/`protect()`/`candidates()` invariants: a protected index never appears in `candidates()`; re-protect within a frame is idempotent; `beginFrame()` returns the whole protected partition to the candidates; `sync()` admits tiles appended by `rescan()` |
| `test/test_gggs_eviction.cpp` (new, headless, GL-free) | Synthetic single-level strip walked far wider than the cap: resident count stays ≤ cap while visited count grows (the `test_map_tiles_eviction.cpp` / `test_tile_eviction_rss.cpp` bounded-count shape); no tile intersecting the current `load_viewport_` is ever evicted; pan-back reloads a dropped tile (`pixelsLoadedCount()`); a ladder's coarsest level survives a pan that evicts the fine level, and the exemption is bounded; a `loadFailed()`-covering finer tile in view is retained; over-budget (working set alone above the byte target) floors the cap and reports a non-empty status rather than evicting the visible set; `max_resident_bytes = 0` reproduces pre-#195 residency |
| `test/test_gggs_elevation.cpp` (extend — review finding 14) | `getElevation()` over an **evicted** (panned-away) area returns NaN while the in-viewport readout still answers — the consequence the plan's own table names |

Headless coverage is the CPU half of the release only: with no GL context
`releaseGL()` is a no-op and the texture half of the 7.03 MiB is untested
(review finding 14). Recorded as a known coverage gap in `camp-ADR-0014`
rather than papered over.

The A→B→A ping-pong assertion from the original plan is **withdrawn** (review
finding 12): with a cap below `|A ∪ B|` ping-pong is guaranteed, so the
assertion was either unachievable or vacuous. The reload behaviour is covered by
the pan-back assertion instead, at a cap that admits the working set.

## Files to Change

| File | Change |
|------|--------|
| `src/camp_map/raster/tile_residency.h` | **New.** Splice-partition residency helper (protected / evictable), header-only, Qt- and GL-free |
| `src/camp_map/raster/gggs_tile_layer.h` | Budget + residency members; `evictIfOverBudget()` slot; `refreshProtection()`, `releaseTiles()`, `updateStatus()`, hole-coverage helper; test seams (`residentTileCount()`, `setResidentBudgetBytesForTest()`, `refreshResidencyForTest()`) |
| `src/camp_map/raster/gggs_tile_layer.cpp` | `paint()`: protect + record the view centre + schedule eviction; `evictIfOverBudget()` with re-arm; `releaseTiles()`/`updateStatus()` extraction; ctor reads `QSettings` |
| `test/test_tile_residency.cpp`, `test/test_gggs_eviction.cpp` | **New** (see Test Strategy) |
| `test/test_gggs_elevation.cpp` | Readout over an evicted area |
| `CMakeLists.txt` | Register the two new gtest suites |
| `docs/decisions/0014-gggs-viewport-scoped-residency.md` | **New camp ADR** — the decision, cross-referencing `uma-ADR-0013` D4 and `camp-ADR-0010` |
| `docs/decisions/0013-lod-level-selection-demand-driven-load.md` | Amend four stale places: the Residency bullet (`:81-84`), the Release bullet (`:132-138`), the residency-bound/camp#195 paragraph (`:147-165`), and the camp#195 pointers at `:97` and `:131` — the overdraw + composite-depth-cap mitigations re-route to **camp#198** |

Not changed (was in the `b70311f` plan): `viewport_clip.h`,
`raster_layer.cpp`, `sonar_live_cache_layer.cpp` — decision 4 dropped the
`deriveViewportClip()` signature change.

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Test what breaks | The failure is an OOM after prolonged panning — asserted as a bounded resident count under a synthetic pan walk, the portable proxy both `test_map_tiles_eviction.cpp` and cube's `test_tile_eviction_rss.cpp` already use |
| A change includes its consequences | All four stale `camp-ADR-0013` passages are amended in this PR, `getElevation()` over an evicted area gets a test, and the two follow-ups the ADR pointed at camp#195 for get real issues (camp#197, camp#198) rather than a dangling pointer |
| Capture decisions, not just implementations | `camp-ADR-0014` records fold-vs-drop, bytes-vs-count, the hole-coverage last-resort rule, and the deferred τ lever; without it the next agent re-derives them |
| Only what's needed | No fold-to-parent, no ResourceMonitor dependency, no prefetch (`uma-ADR-0013` D6 wants measurement first), no τ lever (camp#197), no `deriveViewportClip()` churn (camp#150) |
| Improve incrementally | A sane default now; the measured budget arrives via camp#155/#156 through the same `QSettings` seam |
| Enforcement over documentation | Current-frame protection is enforced by `TileResidency`'s type boundary, not by a comment |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| `uma-ADR-0013` D4 | Yes | Budget + structural current-frame protection + hybrid ordering implemented; the quality-relaxation lever is deferred to camp#197 with the cap floor + over-budget status standing in for "never fail silently" |
| `uma-ADR-0013` D5 | Yes | Ascending (zoom-out) backdrop retention is untouched; eviction never targets the current frame, so "the picture never gets worse" holds except across a true pan-away |
| `uma-ADR-0013` D6 | No | No prefetch — the ADR requires latency measurement first |
| `uma-ADR-0013` D8 | No | `getElevation()` is a display readout, not a safety query |
| `uma-ADR-0010` D9 | Yes (respected) | No CAMP-side pyramid is manufactured for `chart`/`reference` |
| `camp-ADR-0010` | Yes | Same forces, different layer; divergences (drop vs fold) are recorded in `camp-ADR-0014` |
| `camp-ADR-0011` | Yes (untouched) | `deriveViewportClip()` is unchanged (decision 4) |
| `camp-ADR-0013` | Yes | Amended in four places — the "levels ≤ selection are never released" residency rule is superseded and the camp#195 pointers are re-routed |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| The residency rule | `docs/decisions/0013-...md` (4 passages) + new `camp-ADR-0014` | Yes |
| Tile residency | `getElevation()` returns NaN over an evicted (off-screen) area | Yes — tested, and recorded in `camp-ADR-0014`; the cursor is always in-viewport, so its tile is protected |
| Tile residency | Auto-range: deliberately **not** recomputed on evict (the fold is widening-only), so the colormap does not flicker as tiles come and go | Yes — stated in `camp-ADR-0014` |
| Per-layer budget | N GGGS layers multiply the process footprint | No — process-wide accounting is camp#155/#156; `camp-ADR-0014` records the gap |
| Per-frame overdraw | Composite-depth capping / occlusion skip | No — camp#198, and `camp-ADR-0013`'s pointers are re-routed there |
| Quality under pressure | The D4 τ lever | No — camp#197 |

## Documentation & Instruction Impact

- **Stale docs** (land in this PR):
  `docs/decisions/0013-lod-level-selection-demand-driven-load.md` (four
  passages); new `docs/decisions/0014-gggs-viewport-scoped-residency.md`.
  `.agents/README.md` was checked — its GGGS-layer notes do not restate the
  residency rule, so no edit is owed there.
- **Agent-instruction candidates** (proposals only): none new — the reusable
  lesson ("a viewport-filtered *loader* is not a residency bound") belongs in
  `camp-ADR-0014`, which is repo-tracked and discoverable.

## Open Questions

All three of the `b70311f` plan's open questions are answered by the operator
decisions above (byte budget + settings key; τ lever deferred to camp#197;
`deriveViewportClip()` unchanged). None remain open.

## Estimated Scope

Single PR, ~6 atomic commits: (1) plan revision, (2) `TileResidency` + its test,
(3) `releaseTiles()` + `updateStatus()` extraction, (4) budget/eviction +
tests, (5) ADR-0014 + ADR-0013 amendments, (6) progress entry.
