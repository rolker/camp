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
