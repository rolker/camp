---
issue: 181
---

# Issue #181 — Depth layers: land/water-distinguishing topo-bathy colormap pivoted at chart datum

## Issue Review
**Status**: complete
**When**: 2026-08-22 18:45 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Issue**: #181
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: needs-more-detail

### Actions
- [ ] Scope this PR to the **interim form only** (constant per-layer pivot
  parameter). Given the Tuesday 2026-08-25 nearshore survey, do not let this
  PR wait on `unh_marine_autonomy#288`'s per-point datum sampling — state
  that explicitly in the plan so scope doesn't creep toward the full form.
- [ ] Correct the "Fit with existing work" framing before planning: `camp::map::ColorMap`
  (camp#63) was already retired and replaced by `marine_colormap` + LUT-bake per
  camp ADR-0008 — camp#63 is stale/superseded, not the live facility. The actual
  gap is narrower than "doesn't fit current per-layer ramp assumptions" suggests:
  `marine_colormap` already ships `oleron`/`hypsometric` topo-bathy palettes with
  `PaletteDomain::shoreline_position`, and both already appear in CAMP's colormap
  picker menu (`raster_layer.cpp` iterates `marine_colormap::palette_names()`
  unconditionally). That is very likely today's reported symptom: **the palettes
  are selectable, but `RangeModel::update_auto(data_min_, data_max_)` stretches
  them linearly over the visible data span with no anchor, so the baked shoreline
  color lands wherever the data range happens to put it — not at chart datum.**
  The plan should investigate/confirm this before assuming a new colormap
  facility is needed.
- [ ] Use `marine_colormap`'s `BreakpointMap` (`lookup.hpp`) to anchor the
  palette's `shoreline_position` onto the pivot value, rather than inventing a
  new anchoring scheme in CAMP. Confirm during planning that its output plugs
  into the existing `RasterGlRenderer::ensureLut()` / `RangeModel` seam without
  a new render path.
- [ ] Any pivot implementation must preserve ADR-0008's Consequence #1:
  `bake_lut`'s `TransferParams` **must stay identity** — the pivot has to be
  expressed as which colors land in the 256-entry LUT (i.e., an asymmetric
  effective lo/hi feeding the *existing* linear shader normalize), not as
  gain/contrast baked into the texture. Flag this explicitly in the plan so it
  isn't rediscovered mid-implementation.
- [ ] Decide explicitly whether ADR-0009's "Colormap range…" dialog
  (`colormap_range_dialog.{h,cpp}`, the `ColormapLegendWidget`) needs to surface
  the pivot for topo-bathy layers, or whether that's out of scope / a follow-up.
  The dialog and `RangeModel` currently model range as a single Auto/Manual
  lo/hi with no pivot concept — leaving this unaddressed risks the dialog
  silently fighting the new anchoring (e.g. a manual range override that no
  longer keeps the shoreline pinned).
- [ ] This is an architecturally significant decision (introduces a pivoted/
  anchored normalize into the shared ADR-0007 scalar-layer render path used by
  `RasterLayer`, `GggsTileLayer`, and `SonarLiveCacheLayer`). Per the "capture
  decisions" principle and the existing camp ADR-0008/0009 precedent for this
  exact code path, plan-task should record the anchoring mechanism as a new
  camp ADR rather than only as an issue/PR.
- [ ] Confirm whether `unh_marine_autonomy#288`'s pivot-source dependency is a
  hard blocker for the *interim* form (it should not be — the issue already
  frames a per-layer constant pivot as decoupled) and record that in the plan
  so a reviewer doesn't misread this PR as blocked.
- [ ] Add tests for the pivot placement itself (LUT color at the pivot value
  matches `shoreline_position`; degenerate cases — pivot outside the data
  range, pivot at/near a data-range boundary, zero-width range) alongside
  whatever `RangeModel`/dialog tests already exist (`test_range_persist`).

## Plan Authored
**Status**: complete
**When**: 2026-08-22 18:51 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Plan**: `.agent/work-plans/issue-181/plan.md` at `56ac199`
**Branch**: feature/issue-181 at `56ac199`
**Phases**: single PR, internally sequenced (pure LUT-bake helper -> renderer wiring -> datum-service/fallback wiring -> ADR + follow-up issue)

### Open questions
- [ ] Polygon overrides / `lake_datum` deferred (VDatum-only) — confirm acceptable for Tuesday's survey area, or fast-follow if inland/estuarine fringe is in scope.
- [ ] `~/data/world/datum/` QSettings-defaulted grid paths are a new precedent (no existing consumer defaults to this path) — confirm intended, or require explicit one-time settings entry.
- [ ] `test_chart_datum_service.cpp` fixture approach (real small VDatum grid fixture vs. manual-exercise-only for the PROJ-dependent path) — decide during implementation; the nullopt-on-missing-grids path is unit-testable regardless.
- [ ] Rebake-on-every-pan-tick cost is unmeasured — land the simple form, add a coarse-grid pivot-query threshold only if manual exercise shows visible cost.
