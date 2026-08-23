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

## Plan Authored
**Status**: complete
**When**: 2026-08-22 19:06 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-181/plan.md` at `b94a7d4`
**Branch**: feature/issue-181 at `b94a7d4`
**Phases**: two stacked PRs (PR1 = anchoring mechanism + manual anchor + colorbar fix + ADR; PR2 = tide-linked anchor via TF)

**This entry supersedes the `## Plan Authored` entry of 2026-08-22 18:51 and
the plan committed at `56ac199`.** That plan built a `chart_datum_service`
querying VDatum once per render to compute a per-region chart-datum pivot. The
operator directed a redesign at the run-issue checkpoint because it contradicts
`docs/vision.md` on `feature/issue-12` of `rolker/marine_colormap` (PR mc#14,
unmerged), which states that there is no `chart_datum` runtime frame (uma
ADR-0010 D5), that datum conversion happens at import via
`marine_vertical_datum`, and that breakpoints are constants in the chosen
frame — express the field relative to `map_tide` and the shoreline break is
0.0. The rewritten plan drops VDatum/PROJ/grids entirely and reads the anchor
from the `map_tide` frame. The `BreakpointMap`-in-the-LUT-bake mechanism from
the old plan survives unchanged; only the source of the anchor value changed,
which deletes the GUI-thread PROJ stall, the `~`-expansion bug and the
grid-provisioning gate along with it.

Ground-truth corrections found while replanning, both of which changed the
design:

- `sea_surface_estimator` is a node inside `mru_transform`, not a missing
  package. `/tf` is bridged to the operator station, so `map_tide` is
  reachable from CAMP when the boat is up (unverified on the ROC machine).
- **The handoff's claim that the raster layers can call TF is wrong.**
  `RasterLayer`/`GggsTileLayer`/`RasterGlRenderer` live in `libcamp_map`,
  declared ROS-free with a one-directional boundary (ADR-0002,
  `CMakeLists.txt:239`), and derive from `map::Layer`, not `camp::ros::Layer`.
  The anchor must be pushed in from `camp_map_ros` through a ROS-free holder,
  not pulled by the layers.
- `ColormapLegendWidget::setLut()` already exists, so the ADR-0009 colorbar
  problem is a three-line fix (the colorbar becomes exact under an anchor)
  rather than the "show a value, document the gap" mitigation the plan review
  settled for.

All four surviving Plan Review must-fixes are carried: status through
`GggsTileLayer::updateStatus()`; no model updates inside `paint()`;
Documentation Impact corrected (ADR-0008 D#2 amended plus two stale code
comments land in this PR); the colorbar fixed. The provisioning must-fix is
re-pointed from VDatum grids to "is `map_tide` actually visible on the ROC
machine", which is now an open question gating PR2.

### Open questions
- [ ] Is `map_tide` visible to CAMP's own TF buffer on the ROC machine? Needs a `tf2_echo` before PR2 can be trusted; if not, PR1's manual anchor is the whole feature.
- [ ] Confirm the PR1-first sequencing given Tuesday 2026-08-25, or whether PR2 should be attempted regardless of the verification gap.
- [ ] Frame auto-discovery policy: "unique frame ending in `map_tide`" vs. requiring an explicit configured frame pair (two vehicles, bizzy and izzy, can share one graph).
- [ ] Confirm D1 — uniform shift rather than GeoZui4D's below-surface-only asymmetry. Uniform shift makes displayed land elevation tide-dependent; the asymmetry would open a ~28 m discontinuity at the shoreline in our ellipsoidal frame.
- [ ] Schedule honesty: PR1 is achievable and verifiable by Monday; PR2 is writable but not safely verifiable before Tuesday (needs the owed ROC CAMP rebuild plus a live boat).

## Plan Authored
**Status**: complete
**When**: 2026-08-22 19:17 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-181/plan.md` at `32812ce`
**Branch**: feature/issue-181 at `32812ce`
**Phases**: three stacked PRs (PR1 = mechanism + manual anchor + colorbar fix + ADR; PR2 = `map_tide` anchor; PR3 = chart-datum source)

**This entry supersedes the `## Plan Authored` entry earlier today at
`b94a7d4` (which itself superseded `56ac199`).** Two operator corrections,
both verified in-tree before adopting:

1. **Retracted an incorrect schedule finding.** `b94a7d4` claimed the
   tide-linked anchor "is not safely verifiable before Tuesday — it needs the
   already-owed ROC CAMP rebuild and a live boat". Wrong. `marine_simulation/
   launch/sim_robot_launch.py:338-358` brings up `mru_transform_node`
   broadcasting `<ns>/map_tide`, and `asv_sim/config/environment.yaml`
   carries a harmonic tide (NOAA Station 8423898, Fort Point NH) with
   `speed_factor` documented as compressing a 12-hour cycle into ~12 s. The
   same file gives `ellipsoid_to_mllw: -28.104` — within 7 cm of the measured
   −28.038 m at the Isles of Shoals — so the sim is an oracle for **both**
   anchor sources. PR2 is now targeted at Tuesday, sim-verified. The residual
   field-side unknown narrows to whether `map_tide` survives the bridge to the
   ROC box specifically.
2. **A chart-datum anchor is wanted, not forbidden.** `b94a7d4` over-corrected
   from the `56ac199` rejection and treated datum knowledge in CAMP as
   off-limits. What was rejected was a *colormap-private* per-render PROJ query
   on the GUI thread; what is wanted is a CAMP-level datum capability the
   colormap merely consumes. CAMP already anticipates it — two comments
   (`autonomousvehicleproject.h:86-87`, `projectview.cpp:213-215`) show two
   differently-labelled depths "until the datum service (#288)" for camp#180's
   readout, independent of this issue.

The anchor is therefore now **source-agnostic**: a plain `optional<double>`
written by pluggable sources, precedence chart datum > `map_tide` > manual >
none. Default per S-98 Ed. 2.0.0 Appendix D (which the vision doc itself
endorses): chart datum is the default, `map_tide` is an operator-selectable
adjustment, off by default, with permanent on-screen indication while active.
The vision tension is reconciled explicitly in the plan rather than left
implicit — uma ADR-0010 D5 excludes a `chart_datum` **TF frame in the
navigation loop**, and the vision says plainly that "D5 keeps models out of the
navigation loop, not out of the system" and that a modelled tier is "entirely
appropriate in camp". "`marine_colormap` should never know about tides, datums
or drafts" stays honored untouched: the library receives a number.

**New ground-truth finding that shaped Phase C: there is no datum service to
call.** A workspace-wide grep finds no `.srv` mentioning datum at all, and
uma#288's actual title is "world/: canonical updater-managed home for
geospatial support data … ADR-0010 D3 amendment" — it decides where grids
*live*, not who resolves them. The available capability is the ROS-free
`marine_vertical_datum` library (`resolve_datum() -> optional<DatumResult>`
with `chart_datum_z`, `datum_config.hpp:82-106`). Phase C scopes CAMP linking
it directly, off the GUI thread and region-cached, rather than assuming a
service exists.

Carried forward unchanged from `b94a7d4`: the `BreakpointMap`-in-the-LUT-bake
mechanism; the ROS-free `libcamp_map` boundary finding (ADR-0002,
`CMakeLists.txt:239` — the layers derive from `map::Layer`, not
`camp::ros::Layer`, so every source must push, never pull; this is also what
structurally prevents the rejected design from recurring);
`ColormapLegendWidget::setLut()` making the colorbar a real fix rather than a
documented gap; all four surviving Plan Review must-fixes; and the uniform-shift
decision with its ~28 m-discontinuity reasoning.

### Open questions
- [ ] Is `map_tide` visible to CAMP's TF buffer on the ROC machine? One `tf2_echo` there; sim covers everything else about PR2.
- [ ] Confirm the D3 default — chart datum default with tide as a selectable adjustment (S-98 model), vs. tide as default since under-keel clearance is the live question among rocks. One-line change either way.
- [ ] Phase C scope: does PR3 also convert camp#180's cursor readout to the new provider (retiring both `#288` comments), or does that follow in a sibling PR?
- [ ] Frame auto-discovery policy: "unique frame ending in `map_tide`" vs. an explicit configured pair (bizzy and izzy can share one graph).
- [ ] Confirm D1 — uniform shift rather than GeoZui4D's below-surface-only asymmetry.

## Plan Authored
**Status**: complete
**When**: 2026-08-22 19:26 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-181/plan.md` at `3a276f2`
**Branch**: feature/issue-181 at `3a276f2`
**Phases**: three stacked PRs — PR1 = Phase A+D (mechanism + manual anchor + colorbar fix + ADR); **PR2 = Phase C (chart-datum source, the default)**; PR3 = Phase B (platform-scoped tide)

**Supersedes the `## Plan Authored` entry at `32812ce` earlier today.** The
operator settled the anchor-default open question and the consequences reshaped
the sequencing.

**Decision: chart datum is the default; switching to a platform's tide is one
click.** The decisive reason is one not visible in the code and now recorded in
both the plan and the ADR: **`map_tide` is per-platform**. With one platform up
"the tide" reads as unambiguous; with two, the display would have to decide
*whose* tide it tracks. Chart datum is a property of the **location**, not of a
vehicle, so it has no such ambiguity. A display-wide vertical reference that
silently depends on which boat happens to be up is a latent correctness bug, not
an ergonomic wrinkle. This argument is independent of — and stronger than — the
S-98 Appendix D reasoning from the previous version, which is kept as
corroboration (two unrelated lines of reasoning reaching the same default).

The plan now **distinguishes the fallback *order* from the *default active
source*** (D3), which the previous version conflated. Order stays chart datum >
platform tide > manual > none; chart datum is additionally the mode a layer
starts in.

**New decision D7 — the tide source is platform-scoped from day one**, i.e.
"platform X's `map_tide`", never "the `map_tide` frame", even while only one
platform exists. Checked camp before designing a selector, and **Roland's
recollection was right**: `/marine/platforms` support is already present.
`platform_manager.cpp:25` subscribes to `/marine/platforms`
(`marine_interfaces::msg::PlatformList`); `Platform.msg` carries
`platform_namespace` and `platform.cpp:107-108` already reads it; and camp
already has an active-platform concept (`PlatformManager::currentPlatform` →
`AutonomousVehicleProject::updateActivePlatform` → `activePlatform()`,
`mainwindow.cpp:105`). So the selector hangs off the existing enumeration with
a read-only namespace accessor — **no parallel selector, no view-locking, no new
platform UI**; that stays Roland's unopened work, and this design consumes a
richer selector unchanged because the key is already `platform_namespace`.

**This dissolved a prior open question**: the "unique frame ending in
`map_tide`" auto-discovery heuristic is dropped outright. `platform_namespace`
composes directly into `<platform_namespace>/map_tide`, so platform identity and
frame name are the same fact. A heuristic that guesses wrong with two vehicles up
is precisely the ambiguity the default exists to remove.

**Re-sequencing — PR2 and PR3 swap.** With chart datum as the default, building
it last would have shipped a default that does not exist: every layer would fall
through to tide, making the "default" fictional and the fallback the de-facto
policy. So the chart-datum source moves ahead of the tide source. **This does not
change what ships for Tuesday**, and that is the load-bearing point: PR1's manual
anchor already delivers the default's semantics, because for a single-region
survey the number the operator types *is* the chart datum (−28.038 m at the
Shoals; the sim's `ellipsoid_to_mllw: -28.104` agrees to 7 cm). PR1 is
chart-datum anchoring done by hand; PR2 is the same thing done automatically and
spatially. Neither PR2 nor PR3 is needed for Tuesday. Contingency recorded: if
exactly one of them could land early, PR3 is the one that fits, needing only the
sim and no grids — stated as a contingency, not the recommendation.

**One open question reinstated**: VDatum/geoid grid provisioning on the machine
that runs CAMP. It was dropped as moot when the datum source left scope; with
chart datum as the default it gates PR2.

Everything else carried forward: the `BreakpointMap`-in-the-LUT-bake mechanism;
the ROS-free `libcamp_map` boundary (ADR-0002 — sources push, layers never
pull); `ColormapLegendWidget::setLut()` as a real colorbar fix; there being no
datum service to call (uma#288 is storage layout); the four surviving Plan Review
must-fixes; and uniform-shift (D1) with its ~28 m-discontinuity reasoning.

### Open questions
- [ ] Are the VDatum/geoid grids present on the machine that runs CAMP? Reinstated; gates PR2 now that chart datum is the default.
- [ ] Is `map_tide` visible to CAMP's TF buffer on the ROC machine? One `tf2_echo` there; sim covers everything else about PR3.
- [ ] Does the tide anchor's platform selector belong here or in Roland's unopened `/marine/platforms` work? Scoped here today as "active platform, else sole platform, else say none".
- [ ] Phase C scope: does PR2 also convert camp#180's cursor readout to the new provider (retiring both `#288` comments), or a sibling PR?
- [ ] Confirm D1 — uniform shift rather than GeoZui4D's below-surface-only asymmetry.

## Plan Review
**Status**: complete
**When**: 2026-08-23 00:05 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-181/plan.md` at `3a276f2`
**PR**: PR-less (`--issue` mode; branch `feature/issue-181`, camp gitcloud origin → field-mode)
**Verdict**: changes-requested

Independent review of the 4th plan iteration (author was Claude Sonnet; this
reviewer is Claude Opus — genuinely independent, no self-review annotation,
matching the prior Opus-on-Sonnet Plan Review precedent in this file). All five
must-fixes from the `56ac199` Plan Review are carried forward and verified
(status via `updateStatus()`, no model updates in `paint()`, `QDir::homePath()`
expansion, ADR-0008 amendment + stale comments in Documentation Impact,
pre-Monday provisioning check). Load-bearing source claims were checked against
the tree and hold up: ADR-0008 Decision #2's LUT range-independence (the
amendment target) is accurately quoted; `updateStatus()` is confirmed the camp#195
status composer; `ColormapLegendWidget::setLut()`, `web_mercator::mapToGeo()`,
the two `#288` comments, the `/marine/platforms` enumeration + `platform_namespace`,
and the `bathymetry_layer` `map_tide` guards all exist as described. The headline
must-fix is a sequencing defect the reorder introduced, not a redesign.

### Findings
- [ ] (must-fix) PR2↔PR3 reorder inverts a dependency: Phase C (PR2, chart-datum "default") is sequenced before Phase B (PR3), but the seam it pushes into — the ROS-free `shoreline_anchor` holder (step 5) and per-layer anchor-mode/fallback/status wiring (step 7) — is all Phase B. As written PR2 ships inert (nothing to push into, no mode to activate) until PR3, so the reorder doesn't achieve its own "a default built last is not a default" goal. Fix: pull the holder + minimal per-layer anchor-mode/status seam into PR1/PR2; keep only `sea_surface_tracker` + platform-scoping in PR3 — `plan.md:293-349`, `plan.md:457-459`
- [ ] (must-fix) `bathymetry_layer.cpp:653-672` (the copied `map_tide` guard pattern) lives in `unh_marine_autonomy`/core_ws, NOT camp — verified the guards are real and accurate there, but the citation names no repo, so a camp implementer can't find it. Qualify it as cross-repo (uma); note `#220` is a uma issue — `plan.md:80-87`, `plan.md:306-312`
- [ ] (suggestion) ADR-0014 glossed as "(status composition)" but it is GGGS viewport-scoped residency (camp#195); cite camp#195/`updateStatus()` directly — `plan.md:319-323`, `plan.md:411`
- [ ] (suggestion) `shoreline_position` "metres, positive up" is asserted as if quoted but `palette.hpp:56`'s comment states neither unit nor sign; the "no sign flip" correctness crux rests on it — verify concretely or record as an assumption — `plan.md:151-160`
- [ ] (suggestion) ADR-0015 should separate the recorded default *policy* (chart datum) from *when* it is the live runtime default (source at PR2, mode selector at PR3; no runtime chart-datum mode between PR1 and PR3) — `plan.md:199-219`, `plan.md:351-362`
- [ ] (nit) "zero ROS includes" is at `CMakeLists.txt:332`; only "pure Qt/GDAL (ROS-free)" is at `:239` — the plan attributes both to `:239` — `plan.md:107-116`
