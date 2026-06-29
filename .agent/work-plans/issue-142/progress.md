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
