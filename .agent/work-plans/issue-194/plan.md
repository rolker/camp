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

4. **Make `itemsIntersecting()` composite instead of select.** Collapse the
   current two-pass loop (draw every *other* level, then the selected level
   last) into a single ascending pass over `available_levels_` that skips
   `level > selected_level_`. Since `available_levels_` is sorted ascending
   and `selected_level_` is itself a member of it (chosen by
   `selectLodLevel` from the available set), this still draws coarse→fine
   with the selected level naturally last (on top) — painter's-order
   compositing, per the issue's fix direction, with less code than today's
   two-pass version.

5. **Fold the auto-range over the same composited set.** In `tilesReady()`,
   change the range-fold's level gate from `!= selected_level_` to
   `> selected_level_` (skip), matching the render filter exactly — every
   tile that will actually appear on screen contributes to the auto-range,
   not just the topmost.

6. **Simplify stale-level release — no more backdrop-then-release
   two-phase dance.** Under compositing, all resident levels ≤ selected are
   *permanently* part of the picture (not a transient zoom backdrop), and
   levels > selected are *never* drawn once excluded by step 4 — so there is
   no visual gap to protect by delaying their release. In `tilesReady()`,
   change the stale-release loop's condition from `tile->level() ==
   selected_level_ || !pixelsLoaded()` (skip) to `tile->level() <=
   selected_level_ || !pixelsLoaded()` (skip) — i.e. release exactly the
   tiles at levels *finer* than the current selection — and **drop** the
   `!hasUnloadedVisibleTiles(load_viewport_)` gate that used to delay release
   until the incoming level's visible set had fully loaded. That gate existed
   only to avoid blanking the view under the old equality-filter model; under
   compositing the coarser levels ≤ selection are already resident and
   rendering, so releasing newly-stale finer levels as soon as idle
   (`!future_watcher_.isRunning()`) is both safe and simpler. Keep the
   existing `resetPixels()` + `releaseGL()` pairing and GL-context handling
   unchanged — only the level predicate and the extra gate move.

7. **Update `ADR-0013` in the same PR** (required — not a follow-up):
   - "Progressive refinement across a level switch" → rewrite as "Multi-level
     compositing": `selected_level_` is a max, not an equality filter;
     residency model is "every available level ≤ selection stays resident
     permanently, coarse-to-fine painter's order"; stale-release condition is
     now `level > selection`, fired promptly on idle (no more
     load-before-release gate, and why: nothing to protect visually anymore).
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
   - Add a short note (ADR-0010 cross-reference, "Watch, not triggered" from
     the issue review): `GggsTileLayer` has no residency/eviction budget
     (unlike `SonarLiveCacheLayer`, ADR-0010) — under compositing this now
     matters more, since multiple levels stay resident simultaneously in
     steady state. For a derived-overview store the added residency is
     bounded (~33% over the fine level alone, since a 4:1-folded pyramid's
     total size is a bounded geometric series); for a region-disjoint native
     ladder it's bounded by the store's total tile count (fixed at
     production time, not by zoom behavior). No eviction bound is needed for
     this fix; flag as a future ADR-0010-style follow-up only if a
     disjoint-ladder store ever grows large enough for it to matter.

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
   - **Update `DemandDrivenLoadsOnlySelectedLevel`** → rename/extend to
     assert levels ≤ selection all load (e.g. select level 13 with an
     available `{0, 13}` ladder, expect both `pixelsLoadedCount(13) == 1`
     and `pixelsLoadedCount(0) == 1`, not `0` as today).
   - **Rewrite `LevelSwitchKeepsPriorLevelUntilNewLoads`**: its premise ("the
     outgoing coarse level gets released once the fine level finishes
     loading") is now wrong under compositing — the coarse level stays
     resident *permanently* once selected ≤ it was ever loaded at a coarser
     selection that includes it. Replace with two assertions: (a) zooming
     from coarse (0) to fine (13) does **not** release level 0 even after
     level 13 finishes loading (the new steady-state residency model); (b)
     zooming back out from fine (13) to coarse (0) **does** release the
     now-stale level 13 tile once idle (the new `level > selection` release
     condition) while level 0 stays loaded throughout.
   - **`LevelSwitchBackdropRendersDuringTransition`** stays valid as a
     regression (mid-transition render is non-blank) but the reasoning
     shifts: it now demonstrates that the coarse level, being ≤ the newly
     selected fine level, renders as a *permanent* part of the composited
     image rather than a transient backdrop — worth a comment update even
     though the assertions are unchanged.
   - `HeadlessDefaultsLoadEverything` and `UnloadedVisibleTilesTriggerRekick`
     are unaffected (single-level fixtures / `-1` no-selection path) —
     verify they still pass, no code changes expected.

## Files to Change

| File | Change |
|------|--------|
| `src/camp_map/raster/gggs_tile.h` / `.cpp` | Add `is_overview_` member + `setOverview(bool)` / `isOverview() const` (default `false`). |
| `src/camp_map/raster/gggs_tile_layer.cpp` | `loadDirectory()`: tag overview-dir tiles via `setOverview(true)`. `rebuildLevelIndex()`: union all native (non-overview) tile extents instead of finest-level-only. `loadTilesWorker()` / `hasUnloadedVisibleTiles()`: `level != selected` → `level > selected`. `itemsIntersecting()`: collapse to one ascending `level <= selected_level_` pass. `tilesReady()`: range-fold gate `!= selected_level_` → `> selected_level_`; release-loop gate `== selected_level_` → `<= selected_level_`, drop the `!hasUnloadedVisibleTiles(...)` condition. |
| `src/camp_map/raster/gggs_tile_layer.h` | Update the class-level Doxygen / inline comments describing `selected_level_` as an equality filter (multiple spots reference "the selected level" as the sole resident/rendered level) to describe the max-threshold / multi-level-resident model. |
| `docs/decisions/0013-lod-level-selection-demand-driven-load.md` | Amend "Progressive refinement across a level switch", "Extent semantics", "Auto-range across levels"; add the ADR-0010 residency cross-reference note. |
| `test/test_gggs_render.cpp` | Add the disjoint-ladder regression test; update/rewrite the three tests listed in step 8. |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Capture decisions, not just implementations | ADR-0013 amended in this same PR (step 7) — the fix-direction decision and its consequences for residency/extent/auto-range are recorded where the next agent will look, not just in the code diff or this plan. |
| A change includes its consequences | The plan explicitly reconciles all three review-identified equality gates (loader, `itemsIntersecting()`, `tilesReady()`'s two roles) plus a fourth site the review didn't call out by name but the same investigation surfaced (`rebuildLevelIndex()`'s extent union) — without step 2, the compositing fix alone still fails the issue's own acceptance case, since the disjoint regions outside the finest level's footprint would stay outside `boundingRect()` regardless of how rendering is fixed. |
| Test what breaks | Step 8's new fixture reproduces the reported symptom directly (disjoint regions, only one band renders) rather than only re-asserting individual filter predicates: a test suite that only checked "loads levels ≤ selection" could still pass while the extent bug silently drops a region. Existing tests whose *assertions* encoded the old equality-filter/backdrop-release model are rewritten, not left contradicting the new code (stale assertions passing for the wrong reason are worse than no test). |
| Only what's needed | No eviction/residency bound is added for `GggsTileLayer` — the review flagged this as "Watch, not triggered," and the plan confirms why (bounded by store size / bounded pyramid-fold overhead) rather than pre-building infrastructure ADR-0010 doesn't currently require. `reference/`'s same latent exposure (mixed-level imports) is covered for free by this being a generic `GggsTileLayer` fix — no separate follow-up issue needed. |
| Improve incrementally | Single PR, one file's render/load/range logic plus its governing ADR and tests — no wider refactor of `RasterFieldSource`/`RasterGlRenderer` or the LOD selector, both of which are already correctly generic (`selectLodLevel` needs no change; it already returns "the level to treat as the ceiling"). |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0013 — LOD level selection and demand-driven load | Yes | Amended in this PR (step 7) — Decision section's progressive-refinement/extent/auto-range subsections rewritten for the compositing model. |
| ADR-0001 — Adopt ADRs | Yes | Satisfied via the ADR-0013 amendment above; direction 1 is an amendment to an existing ADR, not a new one (it doesn't reverse a decision recorded elsewhere — ADR-0010 D7/D9 in `unh_marine_autonomy` are read, not touched). |
| ADR-0010 — Bounded live-tile eviction + consumer-side overview pyramid | Watch, not triggered | Confirmed in step 7's added note: governs `SonarLiveCacheLayer`'s eviction budget, a different class from `GggsTileLayer`. The compositing change increases `GggsTileLayer`'s steady-state residency (multiple levels, not one) but stays bounded by store size in both the legacy-pyramid and native-ladder cases — no eviction bound needed for this fix. |
| ADR-0011 — Viewport-clip-render-convention | Watch, not triggered | `itemsIntersecting(clip_scene)` already per-tile-clips before draw (line ~583); the compositing change only widens *which levels* enter that same per-tile clip loop, it doesn't bypass or duplicate the clip path. |

## Consequences

- `docs/decisions/0013-lod-level-selection-demand-driven-load.md` updated in
  this PR (not deferred) — see step 7 / Files to Change.
- `reference/` layer's latent same exposure (issue's own note: "same
  exposure once it holds mixed-level imports") is covered by this fix with
  no separate work — it goes through the same `GggsTileLayer` code path.
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

- [ ] None — direction 1 was settled in issue review, the extent-union bug found during codebase exploration is folded into the same fix (not split into a follow-up), and the residency/eviction question is answered (no bound needed) rather than deferred.

## Estimated Scope

Single PR: one class (`GggsTile` + `GggsTileLayer`), its governing ADR
(0013), and its test file. No cross-repo or multi-package coordination.
