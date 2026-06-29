---
issue: 142
---

# Issue #142 — Per-layer colormap range override with interactive colorbar legend

## Issue Review
**Status**: complete
**When**: 2026-06-29 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #142
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Actions
- [ ] Plan must stage the work: range plumbing + persist (PR 1) before the colorbar widget (PR 2) — confirm staging in plan-task.
- [ ] Add tests for range persist/restore and auto↔manual flag round-trip, modelled on `test_gggs_persistence.cpp`.
- [ ] Record the colorbar widget placement decision (dock / map overlay / layer panel) in a camp ADR or detailed plan section before implementing the widget.
- [ ] Verify no ordering conflict with camp#138 (live-cache auto-range refold) — both touch SonarLiveCacheLayer's range folding; coordinate or sequence accordingly.
- [ ] Confirm SonarLiveCacheLayer is included in the manual-override plumbing (the issue says all three sources; the live-cache layer is the most likely to be missed).

### Full review

#### Scope

**Well-scoped?** Yes, with staging. Two distinct concerns live in this issue:
1. **Range plumbing** — replace auto `data_min_`/`data_max_` with operator-settable values in each `renderImage()` caller, persist via `settingsKey()` (auto/manual flag + explicit min/max). The render API already threads `data_min`/`data_max` through `renderToImage()`, so the override drops in at the three caller sites. This is a clean, reviewable single PR.
2. **Interactive colorbar legend widget** — a new widget (no existing colorbar/legend widget found anywhere in camp's source tree). Draggable min/max handles are non-trivial UI. The issue's author acknowledges this is the heavier lift and asks the plan to determine staging. It should be a separate PR.

**Right repo?** Yes — camp project repo, correct worktree (`issue-camp-142`).

**Dependencies**:
- camp#134 (RasterFieldSource consolidation) — prerequisite, already merged; the consolidated `RasterGlRenderer` with `u_min`/`u_max` uniforms is the baseline.
- camp#138 (live-cache auto-range refold) — also touches `SonarLiveCacheLayer` range. If #138 is in-flight, the range override in this issue must not conflict; sequence or coordinate.

#### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Improve incrementally | Watch | Two large concerns (plumbing + new widget) in one issue. The plan must split into stageable, independently-reviewable PRs — PR 1: range plumbing; PR 2: colorbar widget. |
| Only what's needed | Watch | Draggable colorbar handles are more complex than a numeric override in the context menu. The plan should justify the full drag UX vs. a simpler first step (e.g., context-menu min/max inputs). Operator-decided UX per the issue is acceptable, but scope should be explicit. |
| A change includes its consequences | Action needed | Tests for range persist/restore and the auto↔manual flag must ship with the plumbing PR. Existing `test_gggs_persistence.cpp` is the model. The colorbar widget PR likewise needs widget tests or at minimum manual verification recorded. |
| Capture decisions, not just implementations | Action needed | Colorbar widget placement (dock / map overlay / layer panel) is a new UI topology decision. It should be recorded in a camp ADR or at minimum as a detailed plan section justifying the choice. |
| Human control and transparency | OK | Override is explicit, per-layer, with a reset-to-auto affordance. Persistence keeps operator settings across sessions. Aligns well. |
| Workspace vs. project separation | OK | Change is entirely within the camp project repo; no workspace infra touched. |

#### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| camp ADR-0007 (RasterFieldSource) | Yes | Override must flow through `renderToImage(data_min, data_max)` — each layer's `renderImage()` replaces the auto-folded values with the operator's when in manual mode. This is the correct single insertion point per ADR-0007's "one shader / one render path" mandate. |
| camp ADR-0002 (web-mercator scene + layer model) | Watch | Colorbar widget placement must fit the existing layer/panel model. A map-overlay widget that introduces new rendering outside the scene model needs scrutiny. |
| camp ADR-0005 (stores browser + flat display layers) | Watch | Context-menu additions follow the established GggsTileLayer/RasterLayer pattern (colormap submenu, band submenu). Range override entries should follow the same pattern. |
| workspace ADR-0001 (adopt ADRs) | Yes | Colorbar widget placement is a design decision warranting a camp ADR (or at minimum a well-documented plan entry). |
| workspace ADR-0013 (progress.md vocabulary) | Yes | This entry uses `## Issue Review` per ADR-0013. |

#### Consequences

- If a colorbar widget is introduced: its placement is a new UI topology decision — record it (see Action above).
- `SonarLiveCacheLayer` must be included in the manual-override plumbing; it is the third source and the most likely to be accidentally excluded given its different code path under `ros/live_coverage/`.
- Persistence key shape: follow `settingsKey()` pattern already established in `GggsTileLayer::readSettings`/`writeSettings` (colormap + band round-trip). Add `range_mode` (`auto`/`manual`), `range_min`, `range_max` values.
- The existing `test_gggs_persistence.cpp` exercises the persist/restore flow — extend it or add a parallel test for range persistence.

## Issue Review (updated — dependencies merged)
**Status**: complete
**When**: 2026-06-29 12:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #142
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Actions
- [ ] Plan must address "active/selected layer" binding: no existing single-layer-selection API connects MapTreeView selection to an external widget; the plan must propose an approach (e.g. signal from MapTreeView on selection change → mainwindow wires colorbar widget to that layer's range model).
- [ ] Uncertainty band default: plan must specify how a layer identifies an "uncertainty" band (e.g. by band name substring, metadata field from RasterBandMeta, or explicit operator action). camp#108 added band selection but not band-type classification; that mechanism needs to be clarified in the plan.
- [ ] Add camp ADR for colorbar widget placement (dock / treeTabs 4th tab / map overlay). Placement decision is recorded in plan; full ADR lands with the widget PR.
- [ ] Ensure range override goes to `u_min`/`u_max` (the shader uniforms) — NOT into the LUT `TransferParams`. camp ADR-0008 Consequences explicitly flags this: "The colorbar/range UI (camp#142) must respect this split."
- [ ] Add tests for range persist/restore and auto↔manual flag round-trip (modelled on `test_gggs_persistence.cpp`).
- [ ] Confirm SonarLiveCacheLayer is covered by the manual-override plumbing (separate code path under `ros/live_coverage/`; most likely to be missed).
- [ ] Verify no ordering conflict with camp#138 (live-cache auto-range refold) — both touch SonarLiveCacheLayer's range; coordinate or sequence accordingly.

### Updated scope notes (vs. prior review)

The prior review noted "no existing colorbar/legend widget found anywhere in camp's source tree." That is now superseded: **`marine_colormap_widgets::ColormapLegendWidget` is available and installed** (marine_colormap#7 MERGED). camp embeds this widget — it does NOT build its own. This resolves the prior "Watch: draggable complexity" concern; the widget design is settled.

The prior "Only what's needed" watch on drag-handle complexity is now **OK** — the widget is a ready dependency, not scope to be built.

#### Updated Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Improve incrementally | Watch | Still two concerns in one issue (range plumbing + widget embed). Plan should stage: PR 1 range plumbing + persist, PR 2 widget embed + binding. With ColormapLegendWidget pre-built, PR 2 is now "wire + place" rather than "design + build + wire", reducing risk. |
| Only what's needed | OK | Widget is pre-built; embedding it is the correct minimal step. Uncertainty band default is concrete operator pain (backscatter band2 Max=925/Mean=0.16 example). |
| A change includes its consequences | Action needed | Tests for range persist/restore and auto↔manual flag round-trip must ship with plumbing PR. Widget binding tests or manual verification record needed with widget PR. |
| Capture decisions, not just implementations | Action needed | Two decisions still need recording: (1) colorbar widget placement in camp's UI topology, (2) uncertainty-band identification mechanism. Both are non-trivial design choices. |
| Human control and transparency | OK | Explicit per-layer override, visible colorbar, reset-to-auto affordance, persistence across sessions. ColormapLegendWidget's UX (`lo ≤ hi` clamp, drag → Manual, double-click → reset) all align. |
| Workspace vs. project separation | OK | Change entirely within camp project repo; marine_colormap_widgets is a project-level dependency. |

#### Updated ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| camp ADR-0007 (RasterFieldSource) | Yes | Range override flows through `renderToImage(data_min, data_max)` — each layer's `renderImage()` replaces auto-folded values with the operator's when in Manual mode. The three caller sites (GggsTileLayer, SonarLiveCacheLayer, RasterLayer) each need this substitution. |
| camp ADR-0008 (marine_colormap LUT bake) | Yes | **New trigger vs. prior review.** ADR-0008 Consequences explicitly states: "Any future use of gain, contrast, alpha-ramp or below/no-data must go through the shader (or a deliberate shader rewrite), not by baking them into this LUT — otherwise the range the shader applies is double-counted. The colorbar/range UI (camp#142) must respect this split." Range overrides MUST go to `u_min`/`u_max` only. |
| camp ADR-0002 (web-mercator scene + layer model) | Watch | Colorbar widget placement must fit the existing treeTabs / dock model. A new 4th tab in treeTabs is the least-invasive option; a dock or map overlay should be justified in the ADR. |
| camp ADR-0005 (stores browser + flat display layers) | Watch | Context-menu range entries should follow GggsTileLayer/RasterLayer's colormap-submenu pattern. |
| workspace ADR-0001 (adopt ADRs) | Yes | Widget placement and uncertainty-band identification are design decisions needing recorded rationale. |
| workspace ADR-0013 (progress.md vocabulary) | Yes | This entry uses `## Issue Review` per ADR-0013. |

#### New dependency now available

`marine_colormap_widgets::ColormapLegendWidget` (marine_colormap#7, installed):
- Owns a `marine_colormap::RangeModel` (Auto/Manual).
- `setDomain(min, max)` — the data extent.
- `updateAuto(min, max)` — no-op in Manual; use for data-driven auto range.
- `reset()` — returns to Auto.
- `rangeChanged(float lo, float hi)` signal → feed to `renderToImage(lo, hi)` to override `u_min`/`u_max`.
- `setPalette(int index)` — wire to the layer's active colormap palette so the legend shows the correct ramp.
- Layers should use `marine_colormap::RangeModel` directly (or delegate to the widget's owned model) for persist; serialize `mode()` + `lo()`/`hi()`.

#### Uncertainty band default — open design question

The issue requires: when the operator views an uncertainty band → default colormap = `quality` ramp + a sensible `[0, threshold]` manual range. The band identification mechanism is unspecified:
- `RasterBandMeta` (returned by `GggsTileLayer::metadata(band)`) carries band name and data-type info. If uncertainty bands are consistently named (e.g. "uncertainty", "quality"), name-matching is sufficient.
- The plan must choose: (a) name-match at band-switch time, (b) explicit metadata field, or (c) operator-only (no auto-default). Option (a) is consistent with the existing band-name-driven context-menu pattern and is the least-invasive path.
- The `threshold` is unspecified in the issue; plan should propose a default (e.g. 1.0, or the band's 95th-percentile from metadata if available) and record it.

## Plan Authored
**Status**: complete
**When**: 2026-06-29 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-142/plan.md` at `faf2374`
**Branch**: feature/issue-142 at `faf2374`
**Phases**: 2 (PR1 this run — backend + numeric UI; PR2 — colorbar widget embed)

### Open questions
- [ ] camp#138 sequencing: if live-cache auto-range refold lands before PR1 merges, verify the `range_model_.update_auto` call site in `foldAutoRange` does not conflict with #138's fold-logic changes (expected orthogonal, but warrants diff review at merge time).

## Plan Review
**Status**: complete
**When**: 2026-06-29 18:10 +00:00
**By**: Claude Code Agent (Claude Opus)
<!-- Independent review: fresh-context sub-agent, distinct model (plan authored by Claude Sonnet). The name-only self-review heuristic is degenerate here (all agents share "Claude Code Agent"); this review is genuinely independent, so no self-review annotation. -->

**Plan**: `.agent/work-plans/issue-142/plan.md` at `faf2374`
**PR**: PR-less (`--issue` mode)
**Verdict**: approve-with-suggestions

Plan is well-targeted, correctly staged (PR1 backend+numeric UI; PR2 widget), and
ADR-compliant. File targeting verified against live code: all three layers call
`renderer_.renderToImage(…, data_min_, data_max_, size)` (exact substitution
points); fold sites `tilesReady()`/`foldAutoRange()`/`imageReady()` and
`contextMenu()`/settings groups all present. `RangeModel` API
(`update_auto`/`set_manual`/`reset`/`lo`/`hi`/`mode`) confirmed in
`marine_colormap/transfer.hpp`. ADR-0008 lands with the #141 rebase (step 1);
pre-#141 `map::ColorMap::allTypes()` in `raster_layer.cpp` confirms the rebase is
genuinely required. gh is unauthenticated in this environment — issue body was
read from the two `## Issue Review` entries above rather than live `gh issue view`.

### Findings
- [ ] (suggestion) Uncertainty-band auto-default (a #142 requirement, flagged in issue review) is deferred to a post-#104 follow-on, but Estimated Scope says "PR2 closes #142" — reconcile: narrow #142's scope + open a follow-on, or keep #142 open past PR2. — `plan.md:75`
- [ ] (suggestion) Persist round-trip tests use only `TestableGggsTileLayer`; the Consequences table claims the test pins keys for all three layers. Add a per-layer round-trip for SonarLiveCacheLayer + RasterLayer, or record manual verification. — `plan.md:78`
- [ ] (suggestion) Step 7 says persist "via `settingsKey()`" but `RasterLayer::read/writeSettings` group under `itemID()`; implementer should use `itemID()` for RasterLayer. — `plan.md:69`
