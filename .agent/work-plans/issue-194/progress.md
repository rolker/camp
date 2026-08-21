---
issue: 194
---

# Issue #194 — GGGS store layer: region-disjoint native levels (ENC chart ladder) render only one band per zoom — needs multi-level compositing

## Issue Review
**Status**: complete
**When**: 2026-08-20 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #194
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Summary

Verified against source: `GggsTileLayer::itemsIntersecting()`
(`src/camp_map/raster/gggs_tile_layer.cpp:602-629`) and the worker/reset-pixel
filter (lines 375, 412, 428-445) all gate on `tile->level() == selected_level_`
— an equality filter, not a max-level threshold. `selectLodLevel` (ADR-0013)
picks exactly one level per zoom; for a store whose native levels are
region-disjoint (ENC ladder, ADR-0010 D7 in `unh_marine_autonomy`), that one
level's tiles cover only the sub-region compiled at that scale, and
`tilesReady()`'s stale-level release (line ~428-445) eventually zeroes the
other bands' pixels. The issue's root-cause description matches the code
exactly.

ADR-0013 (`docs/decisions/0013-lod-level-selection-demand-driven-load.md`)
explicitly anticipated a "future natively multi-level layer (chart ENC scale
ladder, uma ADR-0010 — native levels, no derived overviews)" feeding
`selectLodLevel` its own ladder (line 39), but its selection/render model
implicitly assumes the sparse ladder is still a nested/overlapping pyramid
(coarser levels cover the same footprint as finer ones) — true for a
derived-overviews sidecar, false for a region-disjoint native ENC ladder. The
progressive-refinement design (stale levels drawn as backdrop, then released
once the selected level's visible set completes) and the extent/auto-range
sections all assume single-selected-level steady-state residency, so
whichever fix direction is chosen, ADR-0013 needs an amendment (not just a
code diff) documenting the changed semantics.

### Scope Assessment

**Well-scoped?** Yes — root cause is fully diagnosed with file/line citations,
three candidate fix directions are offered with a size ordering (direction 1
flagged as probably smallest), and the blast radius is one file
(`gggs_tile_layer.cpp`) plus its governing ADR. No sub-issue split needed;
plan-task can pick a direction.

**Right repo?** Yes. `GggsTileLayer` and ADR-0013 both live in `camp`; the
store-side native-level generation this depends on (ADR-0010 D7/D9) correctly
stays out of scope and is tracked as `unh_marine_autonomy#310`.

**Dependencies**: None blocking. Two open camp issues touch adjacent code in
the same render path and are worth plan-task's awareness (not duplicates):
- camp#109 — sibling-layer z-order/opacity compositing (different axis:
  across layers, not across levels within one layer).
- camp#189 — zoom-in teleport at max zoom, same ADR-0013 arc, different
  symptom (coordinate/index overflow, not disjoint coverage). Not implicated
  by this issue's root cause but touches the same `selected_level_`/
  `lod_level_selector.h` machinery — a fix here should not be assumed to
  explain or fix #189.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Capture decisions, not just implementations | Action needed | ADR-0001 is triggered: the fix-direction decision (max-level compositing vs. extend D9 pyramids vs. per-cell best-available query) is a real design choice with consequences for residency/eviction and for the uma-side pyramid-exclusion rationale (D9). Record it — as an ADR-0013 amendment if direction 1 is chosen (smallest, in-place semantics change), or a new ADR if a store-side rationale reversal (direction 2) is chosen. |
| A change includes its consequences | Action needed | ADR-0013's progressive-refinement release logic (`tilesReady()`), extent semantics (`sceneBounds()` unions finest-level only), and auto-range-across-levels sections all assume single-selected-level residency at steady state. Direction 1 (selected level becomes a max, not equality) changes what "steady state" residency means — multiple levels resident permanently for a disjoint ladder — and those ADR sections need explicit revisiting, not just the Decision section. |
| Test what breaks | Watch | No existing gtest fixture appears to model a region-disjoint native ladder (the live repro used a real store). plan-task should include a regression test with a synthetic disjoint-level fixture reproducing the "only one band renders" symptom, not just a visual/manual field re-check. |
| Improve incrementally | OK | Three directions offered, smallest-first flagged; issue explicitly defers the pick to plan-task rather than prescribing implementation. |
| Only what's needed | Watch | Issue notes `reference/` has the same exposure "once it holds mixed-level imports" — currently latent, not yet triggered. Fine to note as follow-up scope rather than pulling into this PR; plan-task should confirm whichever fix direction is chosen naturally covers `reference/` too (both layer types go through the same `GggsTileLayer`), so no separate issue should be needed later. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0013 — LOD level selection and demand-driven load | Yes | This is the ADR whose Decision section documents the exact code being changed; must be amended regardless of which fix direction is picked (see Principle Alignment above). |
| 0001 — Adopt ADRs | Yes | The fix-direction choice is a design decision needing a durable record (amendment or new ADR per above). |
| 0010 — Bounded live-tile eviction + consumer-side overview pyramid | Watch, not triggered | This ADR governs `SonarLiveCacheLayer`'s eviction budget, a different class from `GggsTileLayer` (the store layer this issue targets). `GggsTileLayer` does not appear to be under the same VRAM budget — worth an explicit one-line confirmation in the plan that multi-level-resident compositing (direction 1) doesn't quietly need a residency bound of its own, since ENC/chart stores are small but a mixed-level `reference/` store could grow. |
| 0011 — Viewport-clip-render-convention | Watch, not triggered | `itemsIntersecting(clip_scene)` already clips per-tile before draw; multi-level compositing should compose with the existing clip path rather than bypass it — worth a plan-task sanity check, not a design change. |

### Consequences

- ADR-0013 text must be updated in the same PR as the code change (see above)
  — not deferred as a follow-up, since the ADR is the executable spec other
  agents will read next.
- If direction 2 (extend D9 pyramids to chart/reference) is chosen instead,
  that decision crosses into `unh_marine_autonomy` (ADR-0010 D9's exclusion
  rationale) and needs its own review/amendment there — flag as a
  cross-repo dependency if picked.
- No workspace-level (`ros2_agent_workspace`) consequences — this is
  entirely a `camp`-repo change.

### Recommendations

- Prefer direction 1 unless plan-task finds a concrete reason it doesn't
  generalize — it's the smallest change, keeps the store-side D9 exclusion
  rationale intact, and the issue's own sizing note agrees.
- Regression test should assert on the disjoint-ladder symptom directly
  (all region bands visible in one render), not just that `selectLodLevel`'s
  return value changed meaning.

### Actions
- [ ] Record the fix-direction decision durably (ADR-0013 amendment, or a new
      ADR if direction 2's store-side rationale reversal is chosen).
- [ ] Update ADR-0013's progressive-refinement release logic, extent
      semantics, and auto-range-across-levels sections to match the new
      steady-state residency model, in the same PR as the code change.
- [ ] Add a regression test with a synthetic region-disjoint-ladder fixture
      reproducing the "only one band renders" symptom.
- [ ] Confirm whichever fix direction is chosen naturally covers `reference/`
      too, so no separate follow-up issue is needed for it.
- [ ] One-line confirmation that multi-level-resident compositing does not
      need its own residency/eviction bound (ADR-0010 governs a different
      layer class, `SonarLiveCacheLayer`, not `GggsTileLayer`).
- [ ] If direction 2 is chosen, flag the cross-repo dependency on
      `unh_marine_autonomy` ADR-0010 D9's pyramid-exclusion rationale.


## Plan Authored
**Status**: complete
**When**: 2026-08-20 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-194/plan.md` at `b036a06`
**Branch**: feature/issue-194 at `b036a06`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-08-20 20:16 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-194/plan.md` at `b036a06`
**PR**: PR-less
**Verdict**: changes-requested

### Findings
- [ ] (must-fix) `itemsIntersecting()`'s collapse to a single ascending `level <= selected_level_` pass drops the zoom-OUT progressive-refinement backdrop: today's two-pass loop draws ALL non-selected levels (coarser AND finer) first, then the selected level last (`gggs_tile_layer.cpp:609-617` comment: "no blank/flicker on zoom in EITHER direction ... finer on zoom-out"). Under the plan's step 4, once selection drops to a coarser level, the still-resident finer tiles (now `level > selected_level_`) are excluded from every render until the new coarser level's tiles finish loading — reintroducing the exact "zoom flicker blank" ADR-0013/camp#103 fixed, but only for zoom-out. Step 8's rewritten `LevelSwitchKeepsPriorLevelUntilNewLoads` only checks `pixelsLoadedCount` (residency), not render content, so it cannot catch this; `LevelSwitchBackdropRendersDuringTransition` only exercises zoom-IN (coarse→fine), which is unaffected since `0 <= 13` always draws. Either keep drawing stale finer-than-selection tiles as a backdrop until `tilesReady()` releases them (composite over "resident" tiles, not strictly `level <= selected`), or explicitly document+test the zoom-out blank-until-load behavior change in the ADR-0013 amendment. — `plan.md:73-81`, `plan.md:161-166`
- [ ] (must-fix) The rewritten `DemandDrivenLoadsOnlySelectedLevel` (renamed/extended) only tests selecting the ladder's topmost available level (`{0, 13}`, select 13, expect both load) — this cannot distinguish "correct ceiling filter" from "filter removed entirely" (a regression to the pre-camp#103 eager whole-store load), since with selection at the max, every available level trivially satisfies `level <= selected`. Add a case selecting a level below the ladder's max (e.g. select 0 with `{0, 13}` available) asserting `pixelsLoadedCount(13) == 0` — proving the `level > selected` exclusion still holds at the new threshold semantics. — `plan.md:147-150`
- [ ] (suggestion) The residency/eviction note (step 7's ADR-0010 cross-reference) is technically correct in isolation but understates real risk: `GggsTileLayer` already has no within-level eviction (a resident tile is never released just because it scrolled off-screen; `tilesReady()`'s release only fires across a level change). This PR multiplies that pre-existing unbounded-within-level growth across every level <= a selection ever reached in a session. For the `chart` store (54 tiles, fixed native ladder) this is genuinely bounded and harmless, matching the plan's math. But the issue body itself states `reference/` "has the same exposure once it holds mixed-level imports (S-102 + the pending 1 m Appledore grid at a fine level vs coarse legacy priors)," and per `unh_marine_autonomy` ADR-0010 D9, `draft`/`processed` layers get generated overview pyramids over potentially large fine-level survey coverage — both go through this same `GggsTileLayer` code path today, not hypothetically. Camp's own ADR-0010 (`docs/decisions/0010-bounded-eviction-overview-pyramid.md`) exists because `SonarLiveCacheLayer`'s identical unbounded-accumulation shape OOM'd the salmon operator station (camp#153). Recommend filing a tracked follow-up issue now (not a maybe-someday note) and having the ADR-0013 amendment name `reference`/`draft` explicitly (not just "a disjoint-ladder store") as the classes to watch, since chart is the one case in this family that is NOT the risk. — `plan.md:124-134`

### Verified correct (no finding)
- `scene_bounds_` extent-union claim (step 2): confirmed against `rebuildLevelIndex()` (`gggs_tile_layer.cpp:167-198`) — today unions only the finest available level's tiles; for the chart store (no `overviews/` sidecar, all 4 levels native in the main dir) this collapses to L8 only, leaving Lewes L5/L7 and Shoals L6 outside `boundingRect()` regardless of the compositing fix. The plan's replacement (union every non-overview tile via the new `isOverview()` tag from `loadDirectory()`'s `{fine_dir, overview_dir}` split) is correct and leaves the legacy single-native-level+overviews case unchanged (only one native level present there either way).
- `LevelSwitchBackdropRendersDuringTransition`'s "assertions unchanged" claim: correct for the zoom-IN direction it actually tests (coarse resident level stays `<= ` a newly-selected finer level, so it keeps drawing under the new model too) — see must-fix above for why this doesn't extend to zoom-out.

### Summary
The extent-union fix (finding area 1) is well-verified against source and correctly reasoned. The residency claim (area 2) is mathematically sound for the chart store but glosses over the `reference`/`draft` exposure the issue itself names — recommend explicit follow-up tracking rather than a soft "flag if it ever matters." The compositing collapse in `itemsIntersecting()` (area 3) has a genuine z-order regression on zoom-out that the plan doesn't identify, and the rewritten regression tests (area 4) have a coverage gap that would let both the zoom-out blank regression and a wholesale removal of the demand-driven level filter pass unnoticed. Not ready for implementation as written — address the two must-fix items (they're implementation-shaping, not just doc polish) before starting.

### Recommended Actions
- [ ] Redesign step 4 (or explicitly accept+document+test the zoom-out blank-transition behavior change) so the zoom-out progressive-refinement backdrop is preserved or its removal is a deliberate, tested decision.
- [ ] Extend the `DemandDrivenLoadsOnlySelectedLevel` rewrite with a selection-below-max case that still asserts the `level > selected` exclusion.
- [ ] File a tracked follow-up issue for `GggsTileLayer` residency/eviction bounds scoped to `reference`/`draft` stores (not deferred as an implicit "if it ever matters" note), given camp#153 precedent.

## Implementation
**Status**: complete
**When**: 2026-08-20 20:40 -04:00
**By**: Claude Code Agent (Claude Fable 5)

**Branch**: feature/issue-194 at `b708e49`
**Build**: camp builds clean; full camp suite green (235 tests, 0 errors, 0 failures, 1 skipped — the opt-in `GGGS_TEST_STORE` real-store render).

### Commits
- `190deb7` plan: fold in plan-review findings for #194 (plan-first revision, committed before implementation)
- `9f8780d` Union scene bounds over all native tiles, not the finest level only — `GggsTile::isOverview()` provenance tag + `rebuildLevelIndex()` native-union extent (the verified second bug), with an all-overview degenerate-store fallback
- `dc26daa` Composite every level up to the LOD selection instead of selecting one — ceiling filter in `loadTilesWorker()`/`hasUnloadedVisibleTiles()`; `itemsIntersecting()` collapsed to one ascending pass over ALL resident tiles (no render-time level filter; the `-1` headless branch merged in); `tilesReady()` range fold over levels ≤ selection and release of only levels > selection, keeping the load-before-release gate. Behavior tests rewritten in the same commit so the suite stays green: `DemandDrivenLoadsLevelsUpToSelection`, `DemandDrivenCeilingExcludesFinerLevels` (mid-ladder), `LevelSwitchResidencyAcrossZoomInAndOut`, `ZoomOutRetainsFinerBackdropUntilCoarseLoads`
- `da42e95` ADR-0013 amended to the compositing model (Multi-level compositing section supersedes progressive refinement; extent + auto-range sections amended; residency-bound note names `reference`/`draft` and references camp#195)
- `b708e49` Regression tests: `DisjointNativeLadderExtentCoversAllRegions` + `DisjointNativeLadderRendersAllRegions` — the issue's acceptance-case symptom guards

### Review findings addressed
- **[must-fix] zoom-OUT backdrop regression**: resolved by redesign, not by accepting the behavior change — `itemsIntersecting()` draws the whole resident set (no `level <= selected` render filter), so still-resident finer tiles keep drawing (on top, ascending painter's order) after a zoom-out; `tilesReady()` keeps the `!hasUnloadedVisibleTiles()` gate so they release only once the coarser selection's visible set has loaded. Guarded by `ZoomOutRetainsFinerBackdropUntilCoarseLoads` (render-content assertion, not just residency) — **verified discriminating**: temporarily patching the rejected `level <= selected` render filter back in makes exactly this test fail (blank mid-transition render).
- **[must-fix] mid-ladder ceiling test gap**: `DemandDrivenCeilingExcludesFinerLevels` selects level 7 of a native `{0, 7, 13}` ladder and asserts level 13 does NOT load — distinguishing the ceiling from a removed filter (eager whole-store-load regression).
- **[suggestion, resolved] residency follow-up**: camp#195 referenced from the plan, the ADR-0013 residency note (which names `reference` and `draft`/`processed` explicitly as the classes to watch, `chart` as the non-risk), and the plan's Consequences.

### Verification
- `DisjointNativeLadderExtentCoversAllRegions` fails under the old finest-level-only union (observed while probe rects were exact-edge; now inset 5% for float tolerance and passing against the native union).
- camp#102/#103 protections retained: `HeadlessDefaultsLoadEverything`, `UnloadedVisibleTilesTriggerRekick`, `LevelSwitchBackdropRendersDuringTransition` (zoom-in, comment updated) all pass unchanged.
- Full suite: 235 tests, 0 failures.

### Notes for review
- Commit grouping: the two behavior-encoding tests were rewritten inside the compositing commit (`dc26daa`) rather than the tests commit so every commit on the branch builds and tests green.
- `rebuildLevelIndex()` gained an all-overview degenerate-store fallback (finest-level union) so a store with an `overviews/` sidecar but no native tiles keeps an extent instead of a null boundingRect.

## Implementation
**Status**: complete
**When**: 2026-08-20 20:58 -04:00
**By**: Claude Code Agent (Claude Fable 5)

**Branch**: feature/issue-194 at `635d51a` (pre-push review round 2 fixes)
**Build**: camp builds clean; full camp suite green (236 tests, 0 errors, 0 failures, 1 skipped — the opt-in `GGGS_TEST_STORE` case).

### Review findings addressed (two-lens pre-push review)
1. **[must-fix, Lens B] all-regions overclaim** — `306d1ee`: ADR-0013's render bullet and the mirrored `itemsIntersecting()` comment now claim coverage "at every zoom at-or-finer than each region's native level" and state the coarse-zoom limitation explicitly (a region whose only native level is finer than the selection never loads — the intended viewport-bounded tradeoff), with the residency/coverage follow-up family referenced to camp#195.
2. **[suggestion, Lens A] vacuous mid-transition residency asserts** — `635d51a`: new `PaintDrivenLevelSwitchKeepsOutgoingLevelResident` drives the REAL `paint()` path across a level boundary (QGraphicsView scale change + grab; ground-mpp math via `metersPerUnit`, ~100 m/px selects L0, ~0.15 m/px selects L13) and pins that the `level_changed` branch does not eager-release the outgoing level. Discrimination made deterministic by placing the coarse tile OUTSIDE the zoomed-in viewport (first attempt with an in-viewport coarse tile was masked by the kick reloading it in microseconds — a race, reworked). Verified: a temporary eager release in the `level_changed` branch fails exactly this test; removed, all green. The two formerly-vacuous EXPECTs kept as state-sequencing documentation with corrected comments pointing at the paint-driven test.
3. **[suggestion, Lens B] fine-before-coarse load order** — `3f0c7a8`: `loadTilesWorker()` now iterates a stable ascending-by-level view of the tile set (sorted on the worker thread, off the GUI hot path; tiles_ immutable while the worker runs), restoring coarse-first progressive refinement on slow stores.
4. **[suggestion, Lens B] overdraw note** — `306d1ee`: ADR residency note acknowledges per-frame overdraw multiplication for deep nested pyramids (~14 stacked near-full-viewport quads at fine zoom on a full L13 pyramid, software-GL included); mitigation (skip-covered-tiles / composite-depth cap) rides camp#195 if pan latency regresses.

### Commits
- `3f0c7a8` Load coarse levels first in the demand-driven worker (finding 3)
- `306d1ee` ADR-0013: scope the all-regions claim; note compositing overdraw (findings 1 + 4)
- `635d51a` Pin paint()'s no-eager-release on level switch with a paint-driven test (finding 2)

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-20 20:58 -04:00
**By**: Claude Code Agent (Claude Fable 5)
**Verdict**: approved

**Branch**: feature/issue-194 at `d8366d1`
**Mode**: pre-push
**Depth**: Standard (reason: Qt render-path behavior change on the operator map; two adversarial lenses)
**Must-fix**: 1 | **Suggestions**: 3
**Round**: 1 | **Ship**: recommended — all findings fixed in-session by the implementer (paint-driven test mutation-verified both directions); suite 236 green

Static analysis: camp colcon test targets (green). Governance: ADR-0013 amended in-PR and corrected per Lens B; residency/overdraw family tracked on camp#195. Local model: skipped per operator guidance (--no-local).

### Findings
- [x] (must-fix, Lens B) ADR-0013 + mirrored code comment overclaimed "all regions at every zoom" — false for regions whose only native level is finer than the selection (coarse-zoom blank, intended tradeoff) — texts corrected, limitation stated — `docs/decisions/0013-...` + `gggs_tile_layer.cpp`
- [x] (suggestion, Lens A) mid-transition residency asserts were vacuous (nothing between load and assert could release) — real paint()-driven level-switch test added, fixture reworked until mutation-discriminating — `test/test_gggs_render.cpp`
- [x] (suggestion, Lens B) worker load order not coarse-first — ascending-by-level sorted view at kick time — `gggs_tile_layer.cpp`
- [x] (suggestion, Lens B) per-frame overdraw multiplication untracked — ADR note added; mitigation family recorded on camp#195

### False positives
- (none)

## Integrated Review
**Status**: complete
**When**: 2026-08-21 08:35 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #196 at `ecbeb08`
**Sources**: 3 (Copilot R1 @ `ecbeb08`, Local Review (Pre-Push) @ `d8366d1`, CI rollup @ `ecbeb08`)
**Cross-source confirmations**: 1
**CI**: all-pass (`build-and-test` success, `copilot-pull-request-reviewer` success)

### Findings
- [ ] (cross-confirmed) Unbounded cross-level tile residency: `tilesReady()` releases only levels FINER than the selection, so every level <= the selection stays resident permanently while panning accumulates tiles within each level. Raised by Copilot R1 (inline, "critical", asks for an eviction budget or a merge-blocking dependency) and by the pre-push Local Review / plan review (which quantified it in ADR-0013 and filed camp#195). Scoped assessment: the unbounded-across-pan axis PRE-EXISTS this PR (the old release path also spared the selected level); this PR multiplies it by the levels <= selection — a bounded ~33% geometric overhead for a legacy `overviews/` pyramid, genuinely bounded (54 tiles) for the `chart` store this PR ships for, and materially larger only for future mixed-level `reference` / large `draft`/`processed` stores. Recommended disposition: merge with camp#195 as the named gate before large multi-level stores route through the compositing path in routine operation, rather than blocking this PR on an eviction budget — but this is a merge-gate decision for the operator, not the reviewer. — `src/camp_map/raster/gggs_tile_layer.cpp` (~468-500), `docs/decisions/0013-lod-level-selection-demand-driven-load.md`
- [ ] (suggestion, Copilot R1 suppressed comment) `DisjointNativeLadderRendersAllRegions` asserts opacity + `pixelsLoadedCount()` only, so it pins the LOAD ceiling but not the AUTO-RANGE fold ceiling: if `tilesReady()`'s gate silently reverted to an equality filter, only the L8 band (60000) would fold, the L5 band (20000) would render saturated/clipped, and the test would still pass. Fix: after `waitForLoad()`, assert `layer->dataRange()` spans both bands (min 20000, max 60000). Cheap and mutation-discriminating. — `test/test_gggs_render.cpp:1003-1052`

### False positives
- (none) — Copilot filed one inline comment and one suppressed comment; both describe real properties of the current code. The only correction applied is to the inline comment's severity framing: its "the fixed-size chart ladder is not representative" premise is accurate, but its implicit claim that this PR introduces the unbounded path is not — `GggsTileLayer` had no within-level eviction before this PR either, and ADR-0013 (as amended here) already records the multiplier, the at-risk store classes, and the camp#153 OOM precedent.
