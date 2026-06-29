---
issue: 141
---

# Issue #141 — Adopt marine_colormap in camp (replace internal ColorMap)

## Issue Review
**Status**: complete
**When**: 2026-06-29 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #141
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Summary

Replace camp's internal `camp::map::ColorMap` (3 named ramps: Grayscale, Viridis, Turbo) with
`marine_colormap` palettes + `bake_lut()` / GPU shader across the unified render path
(`RasterGlRenderer` + ~7 consumer files). Explicitly defers the colorbar widget and range UI to
camp#142. `marine_colormap` is already merged (jazzy); camp#134 (render path unification) is
merged (#140). The scope is moderate but coherent: one PR, one dependency addition, one
architectural decision to record.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Colormap menu + per-band selection preserved; settingsKey() persistence maintained; defaults are user-overridable |
| Capture decisions, not just implementations | Watch | LUT integration decision (use marine_colormap's GPU shader vs. bake its ramps into the existing LUT-texture path) is architectural — should be recorded in a project ADR or the plan's decision block, not left implicit |
| A change includes its consequences | Watch | camp#63 "camp-internal ColorMap" wording reconciliation is in scope; render-parity verification must be part of the PR, not a follow-up |
| Only what's needed | OK | Widget and range UI correctly deferred to #142; no premature abstraction |
| Improve incrementally | OK | ~7 consumer files; staging guidance (RasterGlRenderer/GGGS first, then grids/raster_layer) already noted; fits one PR |
| Test what breaks | Action needed | Issue names behavior parity as the key risk but does not specify a test mechanism; plan must define how render equivalence is verified |
| Workspace vs. project separation | OK | Project-repo change; marine_colormap is a project-level library |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0001 (Adopt ADRs) | Watch | LUT path decision is architectural; capture in a camp project ADR or plan decision block |
| ADR-0002 (Worktree isolation) | OK | feature/issue-141 branch already exists in camp repo |
| ADR-0008 (ROS 2 Conventions) | OK | Adding marine_colormap via package.xml + CMakeLists.txt; standard ament pattern |

### Consequences

- **package.xml + CMakeLists.txt**: Must add `<depend>marine_colormap</depend>` and ament find/target_link.
- **Settings persistence**: Existing persisted colormap names (e.g., "Grayscale", "Viridis", "Turbo") must map to marine_colormap palette names; a name-migration fallback may be needed if names differ.
- **camp#63 wording**: Reconcile "camp-internal ColorMap" issue description once this ships.
- **All ~7 consumers must be migrated together**: a partial migration risks a mixed CPU/GPU LUT path for the same data.

### Actions
- [ ] Define render-parity test approach in the plan: specify how equivalence between the old camp LUT and marine_colormap `bake_lut()` output is verified (e.g., unit test comparing LUT entries at sampled points, host-side visual test, or `./ui_ws/test.sh camp` coverage).
- [ ] Resolve "bathy/turbo" and "sequential" palette name ambiguity: marine_colormap's registry is grayscale/bronze/thermal/viridis/turbo/quality — map each per-band default to a named entry or add a new palette to marine_colormap before this issue can ship.
- [ ] Record the LUT integration decision (marine_colormap shader vs. bake ramps into existing LUT-texture path) in a camp project ADR or the plan's decision block.
- [ ] Verify settings-key persistence: ensure that persisted colormap names survive the rename (add migration/fallback if marine_colormap's ramp names differ from current `ColorMap::name()` strings).

## Plan Authored
**Status**: complete
**When**: 2026-06-29 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-141/plan.md` at `d14ca9b`
**Branch**: feature/issue-141 at `d14ca9b`
**Phases**: single

### Open questions
- [ ] No open questions — scope decisions are explicit in the issue; plan is review-plan-ready.
