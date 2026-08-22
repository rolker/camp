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

## Plan Review
**Status**: complete
**When**: 2026-08-22 18:56 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-181/plan.md` at `56ac199`
**PR**: PR-less (`--issue` mode; branch `feature/issue-181`)
**Verdict**: changes-requested

Scope is correct and not narrowed: the plan builds the operator-chosen FULL
scope (automatic per-region chart-datum pivot via `resolve_datum()`), and its
"steps 1-2 alone are shippable" fallback is an honest degradation, not a quiet
retreat. ADR-0008 Consequence #1 is genuinely satisfied — the pivot path calls
`Palette::sample()` through a `BreakpointMap`, never `bake_lut`'s
`TransferParams` — and the `nullopt` fallback is specified as render-unpivoted
everywhere it matters, with no code path that could pivot at 0. The threading
claim matches `vdatum_query.hpp`'s documented contract. The must-fixes below are
bounded plan amendments (not a redesign) plus one pre-Monday provisioning check.

### Findings
- [ ] (must-fix) `GggsTileLayer` fallback status must go through `updateStatus()`, not `setStatus()` — camp#195 made `updateStatus()` THE status composer precisely so writers cannot clobber each other (over-budget/failed-tile/loading reports); a direct per-render `setStatus()` reintroduces the clobber the ADR-0014-era work removed — `plan.md:170-180`
- [ ] (must-fix) Do not call `setStatus()` from `renderImage()` (it runs inside `paint()`): `MapItem::setStatus` → `Map::updateDisplay` → `emit dataChanged`, i.e. a model/tree update re-entered from a scene paint. Record pivot state in a member during render; compose + publish outside paint — `plan.md:170-180`
- [ ] (must-fix) QSettings defaults `~/data/world/datum/...` must be expanded via `QDir::homePath()`; a literal `~` handed to PROJ/`std::filesystem` silently finds no grids → permanent `nullopt` → the feature appears "honestly absent" on Tuesday while actually being misconfigured — `plan.md:126-131`
- [ ] (must-fix) Documentation & Instruction Impact "None" is wrong. This PR makes stale: (a) `raster_gl_renderer.cpp`'s `ensureLut()` comment and the fragment-shader comment, both of which assert "the LUT carries only the palette ramp" / range-independence; (b) ADR-0008 Decision #2's same sentence. ADR-0015 must be recorded as amending ADR-0008 (LUT is now range-dependent), and the code comments updated in this PR — `plan.md:262-265`
- [ ] (must-fix) Add a pre-Monday provisioning check to the plan: the claim "no new provisioning step is needed for the Tuesday survey" is verified only on this dev host. Confirm `world/datum/{geoid,vdatum}` grids AND an Isles-of-Shoals GGGS/S-102 store exist on the machine that will actually run CAMP, before treating full scope as deployable — `plan.md:129-131`
- [ ] (suggestion) Gate the datum *query*, not just the bake, on `palette.domain()->shoreline_position`. Today every layer render would construct the PROJ singleton (recursive grid scan + pipeline compile, on the GUI thread) even for grayscale layers, and `test/test_gggs_render.cpp` / `test_gggs_band_select.cpp` call `renderImage()` directly — making the GL tests host-dependent. Gating removes the one-time GUI stall for every non-topo-bathy user and keeps tests hermetic — `plan.md:100-104, 166-169`
- [ ] (suggestion) Bound the first-call cost explicitly: `make_vdatum_query()` is the expensive part (recursive `.gtx` scan + PROJ pipeline creation) and the plan puts it lazily on the GUI thread. camp#187 is the standing precedent for a GUI-thread load stalling CAMP in the field. Note the expected one-time cost in ADR-0015 or warm it at layer-add time — `plan.md:132-136`
- [ ] (suggestion) The pan/repaint cost concern is overstated *and* under-bounded. Both consumers short-circuit on `cached_image_`, so `pivotAt()` fires only on cache misses, not every repaint — but a continuous pan misses every frame. A two-line "last rounded lat/lon → pivot" memo is cheaper than measuring; land it rather than deferring — `plan.md:181-188, 290-293`
- [ ] (suggestion) ADR-0009 interaction is under-stated. `ColormapLegendWidget` samples the palette **linearly** (`colormap_legend_widget.cpp:197`) and `marine_colormap_widgets` has no breakpoint support, so with a pivot active the colorbar's colour↔value mapping is wrong, not merely missing a pivot marker. It is on-demand (modal dialog) so it is not always visible — but a survey operator reading it would mis-map colour to depth. Minimum in-scope mitigation: show the resolved pivot value (or "no chart datum") in the dialog, ~5 lines — `plan.md:200-211`
- [ ] (suggestion) State the value-convention assumption explicitly: the pivot works because the stores hold **ellipsoidal, up-positive heights** (issue #181) and `PaletteDomain` is likewise "metres, positive up", so `chart_datum_z` is directly comparable. Any depth-positive-down source would need a sign flip. This is the crux of correctness and the plan never says it — `plan.md:65-104`
- [ ] (suggestion) `web_mercator::mapToGeo()` already exists — drop the "confirm/add an inverse helper" hedge and the `web_mercator.h/.cpp` row from Files to Change — `plan.md:225`
- [ ] (suggestion) The "new precedent" framing on `~/data/world/datum/` is not quite right: `enc_updater`'s `config/region_example.yaml:68,76` already carries exactly these two paths as config values. The precedent is operator-visible config, which QSettings matches — cite it rather than claiming no prior art — `plan.md:58-61, 277-282`
- [ ] (suggestion) Minor: the LUT texture is 256×1 with `Linear` min/mag filtering, so a hard shoreline discontinuity smears across ~1 texel and the pivot quantizes to `(hi-lo)/255`. Fine at nearshore spans; note it, and consider `Nearest` LUT filtering if the break must be crisp — `plan.md:65-104`
