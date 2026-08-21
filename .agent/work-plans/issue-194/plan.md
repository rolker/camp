# Plan: GGGS store layer: region-disjoint native levels (ENC chart ladder) render only one band per zoom — needs multi-level compositing

## Issue

https://github.com/rolker/camp/issues/194

## Context

`GggsTileLayer` (`src/camp_map/raster/gggs_tile_layer.cpp`) picks exactly one
`selected_level_` per paint (`selectLodLevel`, ADR-0013) and treats it as an
**equality** filter everywhere: the demand-driven loader
(`loadTilesWorker`/`hasUnloadedVisibleTiles`), the render path
(`itemsIntersecting()`), and the auto-range fold (`tilesReady()`) all gate on
`tile->level() == selected_level_`. That is correct for a *derived overview
pyramid* (`overviews/` sidecar, uma ADR-0011) where every level is a MEAN-fold
of the same footprint — but an ENC-derived `chart/` store (uma ADR-0010 D7)
puts multiple **native** levels directly in the main directory, one per
compilation scale, and each level's tiles cover only the sub-region compiled
at that scale (levels are region-disjoint, not nested). At any zoom exactly
one level's tiles pass the equality filter, so only that level's regions
render — the rest of the chart's coverage never appears, no matter the zoom.

Direction 1 (settled in issue review): make `selected_level_` a **max**
threshold — render/load every available level ≤ selected, coarse-to-fine
painter's order, so fine tiles overdraw coarse where both exist and coarse
fills where fine is absent. Directions 2 (extend D9 pyramids to chart — a
`unh_marine_autonomy` ADR-0010 rationale reversal) and 3 (per-cell query
render) are rejected as out of scope / oversized.

While reading `rebuildLevelIndex()` to reconcile this fix with extent
semantics, a second bug surfaced that direction 1 must also fix or the
acceptance case still fails: `scene_bounds_` unions **finest-level tile
extents only** (comment: "the finest level present is the true footprint").
For the chart store, all four levels (5/6/7/8) sit in the main directory as
native tiles (no `overviews/` sidecar — D9 excludes chart from it), so the
"finest level" is just L8, and `scene_bounds_`/`boundingRect()` would union
only the Portsmouth L8 cells — silently excluding the Lewes L5/L7 and
Shoals-approach L6 regions from the item's extent entirely. Those regions
would never be reachable by `paint()`'s viewport clip regardless of the
compositing fix, because `QGraphicsView` culls the item outside
`boundingRect()`. This must be fixed in the same PR for the acceptance case
("54 tiles at region-disjoint levels 5/6/7/8 ... full coverage at every
zoom") to actually hold.

## Approach

1. **Track native vs. overview provenance per tile.** Add
   `GggsTile::setOverview(bool)` / `isOverview()` (default `false`).
   `GggsTileLayer::loadDirectory()` already iterates `{fine_dir,
   overview_dir}` separately — tag each constructed tile from `overview_dir`
   with `setOverview(true)`. This gives a robust (non-path-string) way to
   tell "derived, padded-to-grid-cell" tiles from "native" tiles at every
   other call site, and is the key that both fixes below key off.

2. **Fix `rebuildLevelIndex()`'s extent union.** Replace the "finest level
   only" union with "every **native** (non-overview) tile, at any level".
   For the legacy single-native-level + `overviews/` sidecar case this is
   unchanged (one native level == today's "finest level"). For a
   region-disjoint native ladder (chart) it now unions all four levels'
   regions, so `scene_bounds_`/`boundingRect()` covers the whole store
   footprint. Overview-sidecar tiles stay excluded (they're padded to their
   coarse GGGS grid cell — including them would balloon the extent, exactly
   the failure mode the original "finest level only" logic was guarding
   against).

3. **Make `selected_level_` a max threshold in the demand-driven loader.**
   In `loadTilesWorker()` and `hasUnloadedVisibleTiles()`, change
   `tile->level() != level` (skip) to `tile->level() > level` (skip) —
   load/consider every tile at a level ≤ the selection (still gated by the
   existing viewport-intersection filter, so this stays viewport-bounded,
   not store-bounded).

4. **Make `itemsIntersecting()` composite instead of select — over every
   RESIDENT tile, with no render-time level filter.** (Revised per plan
   review must-fix: an ascending pass that *skips* `level > selected_level_`
   would regress zoom-OUT — the still-resident finer tiles would vanish from
   every render the moment selection drops to a coarser level, blanking the
   view until the coarser level's tiles finish loading, exactly the flicker
   ADR-0013/camp#103 fixed.) Collapse the current two-pass loop into a
   single ascending pass over `available_levels_` that draws **every loaded
   tile at every level** — no `selected_level_` comparison at render time at
   all. Which levels are resident is governed entirely by the *loader*
   (ceiling filter, step 3: only levels ≤ selection ever load) plus the
   *release timing* (step 6: levels > selection release only once the
   selection's visible set has loaded). Consequences of ascending
   painter's order over the resident set:
   - Steady state: levels ≤ selection composite coarse→fine, fine
     overdrawing coarse where both exist, coarse filling where fine is
     absent — the issue's fix direction.
   - Zoom-IN transition: stale coarser levels draw under the arriving
     selected level — unchanged from today.
   - Zoom-OUT transition: the still-resident finer tiles (`level >
     selection`) draw **above** the coarse levels (ascending puts them
     last) and back the view until step 6 releases them — no blank frame,
     and the finer data is preferred on top exactly as the compositing
     philosophy wants while it is still resident.
   - The `selected_level_ == -1` headless special case collapses into the
     same pass (draw everything, now in deterministic ascending-level
     order) — one code path instead of two.

5. **Fold the auto-range over the same composited set.** In `tilesReady()`,
   change the range-fold's level gate from `!= selected_level_` to
   `> selected_level_` (skip), matching the render filter exactly — every
   tile that will actually appear on screen contributes to the auto-range,
   not just the topmost.

6. **Stale-level release: only levels finer than the selection, and only
   once the selection's visible set has loaded.** (Revised per plan review
   must-fix: the earlier draft dropped the `!hasUnloadedVisibleTiles(...)`
   gate, which would have released the zoom-out backdrop while the coarser
   level was still loading — with step 4 drawing the resident set, that
   release timing is exactly what protects the no-blank-frame guarantee.)
   In `tilesReady()`, change the stale-release loop's condition from
   `tile->level() == selected_level_ || !pixelsLoaded()` (skip) to
   `tile->level() <= selected_level_ || !pixelsLoaded()` (skip) — i.e.
   release exactly the tiles at levels *finer* than the current selection —
   and **keep** the existing `!future_watcher_.isRunning() &&
   !hasUnloadedVisibleTiles(load_viewport_)` gate unchanged: with
   `hasUnloadedVisibleTiles()` now testing levels ≤ selection (step 3), the
   gate reads "every visible tile of the composited picture has loaded",
   and only then do the finer-than-selection backdrop tiles release —
   mirroring the zoom-in timing camp#103 field-verified, now applied
   symmetrically to zoom-out. Keep the existing `resetPixels()` +
   `releaseGL()` pairing and GL-context handling unchanged — only the level
   predicate moves (`==` → `<=`).

7. **Update `ADR-0013` in the same PR** (required — not a follow-up):
   - "Progressive refinement across a level switch" → rewrite as "Multi-level
     compositing": `selected_level_` is a max, not an equality filter;
     residency model is "every available level ≤ selection stays resident
     permanently, coarse-to-fine painter's order; render draws the whole
     resident set with no level filter"; stale-release condition is now
     `level > selection`, still gated on the selection's visible set having
     fully loaded while idle — the gate is what carries the zoom-out
     no-blank-frame guarantee under the new model (the finer backdrop keeps
     drawing on top until the coarser picture is complete).
   - "Extent semantics" → replace "finest-level tile extents only" with
     "every native (non-overview-sidecar) tile's extent, at any level" +
     rationale (region-disjoint native ladders need every level's footprint;
     overview tiles stay excluded because they're padded to the coarse grid
     cell).
   - "Auto-range across levels" → the fold now exactly matches the
     composited render set (`level ≤ selection`) instead of approximating a
     single-level "widen only" transition smoothing; note this is a strictly
     more precise statement of the same never-reset behavior, not a policy
     change (data_min_/data_max_ still never resets on a level switch).
   - Add a short note (ADR-0010 cross-reference): `GggsTileLayer` has no
     residency/eviction budget (unlike `SonarLiveCacheLayer`, ADR-0010) —
     under compositing this now matters more, since multiple levels stay
     resident simultaneously in steady state. For the `chart` store (54
     tiles, fixed native ladder) the growth is genuinely bounded and
     harmless; for a derived-overview store the added residency is bounded
     (~33% over the fine level alone — a 4:1-folded pyramid's total size is
     a bounded geometric series). The classes to watch are `reference` (the
     issue names its exposure "once it holds mixed-level imports": S-102 +
     the pending 1 m Appledore grid at a fine level vs coarse legacy
     priors) and `draft`/`processed` (uma ADR-0010 D9 generates overview
     pyramids over potentially large fine-level survey coverage) — name
     them explicitly in the ADR note, since `chart` is the one case in this
     family that is NOT the risk. The residency/eviction-bound follow-up is
     tracked as **camp#195** (filed during plan review, given the camp#153
     `SonarLiveCacheLayer` OOM precedent); reference it from the ADR note.

8. **Regression tests** (`test/test_gggs_render.cpp`, following the existing
   `writeTile()`-based fixture pattern):
   - **New**: a synthetic region-disjoint 2-level (or 3-level) fixture —
     e.g. a level-5 tile over one lat/lon region and a level-8 tile over a
     genuinely disjoint region (non-overlapping bounding boxes, mirroring
     the Lewes-vs-Portsmouth split) — asserting (a) `sceneBounds()` covers
     the union of both regions (extent fix, step 2) and (b) after selecting
     the finer level (8) and loading, `renderImage()` over the full
     `sceneBounds()` shows opaque pixels in **both** regions' sub-rects, not
     just the L8 region — the direct symptom regression guard the issue
     asked for ("only one band renders").
   - **Update `DemandDrivenLoadsOnlySelectedLevel`** → rename to
     `DemandDrivenLoadsLevelsUpToSelection` and assert levels ≤ selection
     all load (select level 13 with an available `{0, 13}` ladder, expect
     both `pixelsLoadedCount(13) == 1` and `pixelsLoadedCount(0) == 1`, not
     `0` as today).
   - **New: mid-ladder ceiling case** (plan review must-fix — a
     select-the-max case cannot distinguish "correct ceiling filter" from
     "filter removed entirely", i.e. a regression to the pre-camp#103 eager
     whole-store load): a three-level native ladder `{0, 7, 13}`, select
     the mid level 7, assert `pixelsLoadedCount(0) == 1`,
     `pixelsLoadedCount(7) == 1`, and `pixelsLoadedCount(13) == 0` — the
     level ABOVE the selection stays excluded from loading at the new
     threshold semantics.
   - **Rewrite `LevelSwitchKeepsPriorLevelUntilNewLoads`**: its premise ("the
     outgoing coarse level gets released once the fine level finishes
     loading") is now wrong under compositing — the coarse level stays
     resident *permanently*. Replace with the full two-direction residency
     arc: (a) zooming from coarse (0) to fine (13) does **not** release
     level 0 even after level 13 finishes loading (the new steady-state
     residency model); (b) zooming back out from fine (13) to coarse (0)
     keeps level 13 resident while the coarse visible set is incomplete and
     **does** release it once the selection's visible tiles have loaded and
     the loader is idle (the new `level > selection` release condition with
     the retained load-before-release gate), while level 0 stays loaded
     throughout.
   - **New: zoom-OUT backdrop render assertion** (plan review must-fix —
     `pixelsLoadedCount` residency asserts alone cannot catch a render-time
     `level <= selected` filter regression): arrange "fine loaded, coarse
     not" via the spatial filter (fine and coarse tiles at disjoint
     regions; load with selection 13 and a viewport covering only the fine
     tile), then select the coarse level (0). Mid-transition, assert (a)
     the fine tile is still resident and (b) `renderImage()` over the full
     extent is non-blank — the still-resident finer level (now `level >
     selection`) must keep drawing as the zoom-out backdrop. Then
     `waitForLoad()` and assert the coarse level loaded and the fine level
     released. GL-gated like `LevelSwitchBackdropRendersDuringTransition`.
   - **`LevelSwitchBackdropRendersDuringTransition`** stays valid as a
     regression (mid-transition zoom-IN render is non-blank) but the
     reasoning shifts: it now demonstrates that the coarse level, being ≤
     the newly selected fine level, renders as a *permanent* part of the
     composited image rather than a transient backdrop — worth a comment
     update even though the assertions are unchanged.
   - `HeadlessDefaultsLoadEverything` and `UnloadedVisibleTilesTriggerRekick`
     are unaffected (single-level fixtures / `-1` no-selection path) —
     verify they still pass, no code changes expected.

9. **Review-driven additions** (folded into this plan during implementation —
   each was raised by a review round, not present in the original approach):

   *Round 1 (pre-push `review-code` + PR triage):*
   - **All-overview fallback in `rebuildLevelIndex()`** (`9f8780d`). Step 2's
     "union every native tile" rule leaves a *degenerate* store — one with an
     `overviews/` sidecar and no native tile at all — with a null
     `scene_bounds_`, i.e. a silently blank layer. Such a store keeps the old
     finest-level union as a fallback so it retains an extent.
   - **Coarse-first load order in `loadTilesWorker()`** (`3f0c7a8`). `tiles_`
     is in directory-scan (alphabetical) order, which queues the coarse fill
     *behind* the large fine reads on a freshly exposed region — on a slow/NFS
     store the region trickles in at fine resolution with no coarse backdrop,
     defeating the compositing this PR adds. A `stable_sort` ascending by
     level restores coarse-first progressive refinement (worker thread, off
     the GUI hot path).
   - **Paint-driven level-switch test** (`635d51a`). The `setLodForTest()`
     residency tests cannot pin paint()'s no-eager-release behavior — a plain
     setter fires no release path. `PaintDrivenLevelSwitchKeepsOutgoingLevelResident`
     drives the real `paint()` across a level boundary through a
     `QGraphicsView` scale change, with the outgoing tile deliberately kept
     offscreen so a wrong eager release cannot be masked by a reload race.
   - **Auto-range fold-ceiling assertion** (`169d818`). The disjoint-ladder
     render test asserted opacity + `pixelsLoadedCount()` only, so it pinned
     the LOAD ceiling but not the fold ceiling: a fold silently reverted to an
     equality filter would render the coarse band saturated and still pass.
     The test now asserts `dataRange()` spans both bands.

   *Round 2 (pre-push `review-code`):*
   - **Sticky `GggsTile::loadFailed()` marker.** A tile can pass the
     constructor's cheap metadata `valid()` scan and still fail `loadPixels()`
     permanently (file truncated/removed under a live layer). With no failure
     state, `pixelsLoaded()` stayed false forever, so
     `hasUnloadedVisibleTiles()` never went quiet — wedging the pan/zoom
     re-kick guard *and* `tilesReady()`'s load-before-release gate for the
     session, while the worker re-`RasterIO`s the dead tile on every kick.
     Step 3's ceiling widens the exposure from "a failing tile AT the
     selection" to "any failing tile at any level ≤ the selection", at every
     zoom — so this PR owns the fix. The marker is excluded from the
     predicate and from the worker's retry pass, cleared only by `setBand()`
     (an explicit operator retry with different parameters), and surfaced
     through `setStatus()` so the layer settles visibly-incomplete rather
     than silently so. Regression test:
     `UnreadableTileDoesNotWedgeTheReleaseGate`.
   - **Reject `level() < 0` tiles at scan.** `-1` is overloaded as both the
     no-selection sentinel and a value `tileLevel()` returns on digit
     overflow. Such a tile entering `available_levels_` can drive
     `selectLodLevel()` to return `-1`, which disables the ceiling everywhere
     — silently reverting to the eager whole-store load ADR-0013 exists to
     prevent. `loadDirectory()`/`rescan()` now drop and warn.
   - **`resetPixels()`/`releaseGL()` pairing under a failed `makeCurrent()`.**
     The release loop did a CPU-only clear when a context existed but
     `makeCurrent()` failed, leaving a stale texture to shadow any re-load.
     It now skips the release entirely in that case (leaving the finer level
     resident is harmless; the renderer has latched its GL-failed flag).
   - **ADR-0013 wording**: the unqualified "no blank frame, both directions"
     claim is scoped to level *transitions*, cross-referencing the
     coarse-zoom blank the same section documents; plus two consequences the
     original plan did not name — the **NoData/edge-padding backfill**
     (compositing now fills fine-tile NoData holes and grid-cell padding with
     coarse data, a QA-fidelity change of the same family that keeps
     `smooth_interpolation_` default-OFF) and the **`overviews/` padding
     contract** `isOverview()` depends on (it records directory provenance,
     while the scene-bounds guard needs padding — they coincide only by uma
     producer convention).

## Files to Change

| File | Change |
|------|--------|
| `src/camp_map/raster/gggs_tile.h` / `.cpp` | Add `is_overview_` member + `setOverview(bool)` / `isOverview() const` (default `false`). Step 9: sticky `load_failed_` (atomic) + `loadFailed()`, latched on every `loadPixels()` failure path, cleared by `setBand()`. Round 3 (Copilot review round 2): factor the ctor's metadata read into `readMetadata()`, record the file-identity stat (`file_size_`/`file_mtime_ms_`), add `fileChangedOnDisk()` + `refreshFromFile()` — the same-band retry that clears a latched failure when the file at the path was replaced. |
| `src/camp_map/raster/gggs_tile_layer.cpp` | `loadDirectory()`: tag overview-dir tiles via `setOverview(true)`. `rebuildLevelIndex()`: union all native (non-overview) tile extents instead of finest-level-only. `loadTilesWorker()` / `hasUnloadedVisibleTiles()`: `level != selected` → `level > selected`. `itemsIntersecting()`: collapse to one ascending pass over ALL resident levels (no render-time level filter — residency governs; the `-1` special case merges in). `tilesReady()`: range-fold gate `!= selected_level_` → `> selected_level_`; release-loop gate `== selected_level_` → `<= selected_level_`, keeping the `!hasUnloadedVisibleTiles(...)` load-before-release gate (zoom-out backdrop protection). Round 3 (Copilot review round 2): make the release **footprint-aware** — a finer tile intersecting a *failed* selected-or-coarser tile is retained (the gate opens on a failed tile, so "settled" ≠ "covered"); `rescan()` also refreshes tiles whose file changed on disk. Round 4 (Copilot review round 3): `rescan()` treats EVERY latched `loadFailed()` tile as a refresh candidate independently of the file stat (a transient NFS error leaves size/mtime unchanged, so the stat-gated retry never fired for the case the latch finding was raised against), keeping the stat gate for already-loaded replacements; the range fold is factored into `foldDataRange(bool reset)` and `rescan()` calls it with `reset = true` after a refresh so the layer aggregate `data_min_`/`data_max_` is recomputed from the resident set (the fold otherwise only ever widens, leaving a replaced tile's extremes in the Auto colormap range) — a Manual override is untouched. |
| `src/camp_map/raster/gggs_tile_layer.h` | Update the class-level Doxygen / inline comments describing `selected_level_` as an equality filter (multiple spots reference "the selected level" as the sole resident/rendered level) to describe the max-threshold / multi-level-resident model. Round 4 (Copilot review round 3): declare + document `foldDataRange(bool reset)` (incremental-widening vs. recompute-from-resident-set) and restate `rescan()`'s retry contract. |
| `docs/decisions/0013-lod-level-selection-demand-driven-load.md` | Amend "Progressive refinement across a level switch", "Extent semantics", "Auto-range across levels"; add the ADR-0010 residency cross-reference note. Round 3 (Copilot review round 2): state the overview-only-store exception to the extent-exclusion rule (`rebuildLevelIndex()`'s finest-overview fallback). |
| `test/test_gggs_render.cpp` | Add the disjoint-ladder regression test; update/rewrite the three tests listed in step 8. Step 9: `PaintDrivenLevelSwitchKeepsOutgoingLevelResident`, the `dataRange()` fold-ceiling assertion, and `UnreadableTileDoesNotWedgeTheReleaseGate`. Round 3 (Copilot review round 2): `FailedCoarseTileKeepsFinerCoverageResident` (overlapping coarse/fine fixture, mutation-verified). |
| `test/test_gggs_rescan.cpp` | Round 3 (Copilot review round 2): `RepairedTileRecoversWithoutRestart` — a truncated tile latches, the producer rewrites it at the same path, `rescan()` refreshes and the pixels re-read with no restart. Round 4 (Copilot review round 3): `TransientFailureOnUnchangedFileRetriesOnRescan` (rename-aside/back stages a transient read error with a byte-identical stat), `RefreshRecomputesAutoRangeInsteadOfWidening`, `RefreshPreservesManualRangeOverride` — all mutation-verified. |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Capture decisions, not just implementations | ADR-0013 amended in this same PR (step 7) — the fix-direction decision and its consequences for residency/extent/auto-range are recorded where the next agent will look, not just in the code diff or this plan. |
| A change includes its consequences | The plan explicitly reconciles all three review-identified equality gates (loader, `itemsIntersecting()`, `tilesReady()`'s two roles) plus a fourth site the review didn't call out by name but the same investigation surfaced (`rebuildLevelIndex()`'s extent union) — without step 2, the compositing fix alone still fails the issue's own acceptance case, since the disjoint regions outside the finest level's footprint would stay outside `boundingRect()` regardless of how rendering is fixed. |
| Test what breaks | Step 8's new fixture reproduces the reported symptom directly (disjoint regions, only one band renders) rather than only re-asserting individual filter predicates: a test suite that only checked "loads levels ≤ selection" could still pass while the extent bug silently drops a region. Existing tests whose *assertions* encoded the old equality-filter/backdrop-release model are rewritten, not left contradicting the new code (stale assertions passing for the wrong reason are worse than no test). |
| Only what's needed | No eviction/residency bound is added for `GggsTileLayer` in this PR — bounded for the `chart` store this fix targets. The real exposure (`reference`/`draft` stores growing large under multi-level residency, camp#153 precedent) is tracked as camp#195 rather than built speculatively here. `reference/`'s disjoint-ladder rendering exposure itself is covered for free by this being a generic `GggsTileLayer` fix. |
| Improve incrementally | Single PR, one file's render/load/range logic plus its governing ADR and tests — no wider refactor of `RasterFieldSource`/`RasterGlRenderer` or the LOD selector, both of which are already correctly generic (`selectLodLevel` needs no change; it already returns "the level to treat as the ceiling"). |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0013 — LOD level selection and demand-driven load | Yes | Amended in this PR (step 7) — Decision section's progressive-refinement/extent/auto-range subsections rewritten for the compositing model. |
| ADR-0001 — Adopt ADRs | Yes | Satisfied via the ADR-0013 amendment above; direction 1 is an amendment to an existing ADR, not a new one (it doesn't reverse a decision recorded elsewhere — ADR-0010 D7/D9 in `unh_marine_autonomy` are read, not touched). |
| ADR-0010 — Bounded live-tile eviction + consumer-side overview pyramid | Watch, not triggered | Confirmed in step 7's added note: governs `SonarLiveCacheLayer`'s eviction budget, a different class from `GggsTileLayer`. The compositing change increases `GggsTileLayer`'s steady-state residency (multiple levels, not one); bounded for `chart` and for derived-overview pyramids, but the `reference`/`draft` exposure is tracked as camp#195 (no eviction bound needed for this fix). |
| ADR-0011 — Viewport-clip-render-convention | Watch, not triggered | `itemsIntersecting(clip_scene)` already per-tile-clips before draw (line ~583); the compositing change only widens *which levels* enter that same per-tile clip loop, it doesn't bypass or duplicate the clip path. |

## Consequences

- `docs/decisions/0013-lod-level-selection-demand-driven-load.md` updated in
  this PR (not deferred) — see step 7 / Files to Change.
- `reference/` layer's latent same exposure (issue's own note: "same
  exposure once it holds mixed-level imports") is covered by this fix with
  no separate work — it goes through the same `GggsTileLayer` code path.
- `GggsTileLayer` residency/eviction bounds for large `reference`/`draft`
  stores under multi-level residency are tracked as camp#195 (filed during
  plan review) — referenced from the ADR-0013 amendment, out of scope for
  this PR.
- **Deferred to follow-ups** (raised in round-2 review, judged out of scope
  here — see the `## Implementation` entry in `progress.md` for the
  reasoning):
  - `rescan()` still scans the main directory only, so an `overviews/`
    pyramid generated *after* the layer loaded stays invisible until
    restart while Rescan reports success. Pre-existing since camp#103 and
    already recorded under ADR-0013's Consequences; compositing makes it
    more consequential (the missing pyramid is now backdrop for every
    level), but fixing it means extending `rescan()` to the sidecar with
    provenance tagging + tests — a separate change.
  - `rebuildLevelIndex()` leaves the `prepareGeometryChange()` + re-anchor
    contract to its callers. Folding the pair in would change the
    constructor path (which anchors conditionally on a non-empty tile-set),
    so it is a refactor rather than a fix.
  - Per-frame CPU under compositing: `itemsIntersecting()` is now
    unconditionally O(levels × tiles) per pan frame and the worker re-sorts
    an identical order every kick. A level-bucketed index in
    `rebuildLevelIndex()` serves both. This is the same axis as the
    overdraw mitigation ADR-0013 already routes to camp#195.
- No `unh_marine_autonomy` (uma) or workspace-repo consequences — D7/D9 stay
  untouched; this is entirely a `camp`-repo, single-file-plus-ADR change.
- camp#109 (sibling-layer z-order/opacity) and camp#189 (zoom-in teleport)
  are adjacent but not implicated: #109 composites *across* layers (a
  different axis), #189 is a coordinate/index-overflow bug in the same
  `selected_level_`/`lod_level_selector.h` machinery but a different
  symptom — this fix should not be assumed to explain or resolve #189, and
  the plan makes no change to `lod_level_selector.h` (`selectLodLevel`
  already returns the correct max-threshold value; only its *consumers*
  change their treatment of that value from equality to ceiling).

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `docs/decisions/0013-lod-level-selection-demand-driven-load.md` — its Decision section documents the exact equality-filter/backdrop-release/finest-level-extent model this PR replaces (see step 7).
- **Agent-instruction candidates**: None — this is a self-contained rendering-logic fix inside one layer class; no new pattern or pitfall emerged that would generalize beyond what the ADR-0013 amendment already captures for the next agent to read.

## Open Questions

- [ ] None — direction 1 was settled in issue review, the extent-union bug found during codebase exploration is folded into the same fix (not split into a follow-up), and the residency/eviction question is resolved by tracked follow-up camp#195 (no bound needed for this fix; `reference`/`draft` growth tracked there).

## Estimated Scope

Single PR: one class (`GggsTile` + `GggsTileLayer`), its governing ADR
(0013), and its test file. No cross-repo or multi-package coordination.
