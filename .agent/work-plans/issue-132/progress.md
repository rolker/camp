---
issue: 132
---

# Issue #132 — Operator-selectable data-layer interpolation (Nearest default)

## Issue Review
**Status**: complete
**When**: 2026-07-24 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #132
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Summary

Issue #132 proposes making interpolation mode for data layers operator-selectable,
defaulting to Nearest (faithful/unblended) — an operator request following camp#122
which fixed the GGGS data texture but left three QPainter blit sites still applying
`SmoothPixmapTransform`. The suggested shape: per-layer `smooth_interpolation_` bool
(default false), persisted via `settingsKey()`, plumbed to `RasterGlRenderer` via a
`smooth` flag on `RasterFieldItem`, with a context-menu toggle. Basemap/OSM/WMTS/chart
imagery stays smoothed unconditionally.

### Scope

- **Well-scoped?** Yes — one PR can add the bool, persist it, wire the context-menu item,
  plumb it through `RasterFieldItem`, and update the three blit sites. No sub-splitting needed.
- **Right repo?** Yes — all changes are in `camp` (project repo, `ui_ws`).
- **Dependencies?** Critical: this worktree branches from `feature/issue-103` (PR#173, OPEN,
  approved, awaiting operator merge). The PR for #132 **must target `feature/issue-103`** (stacked PR),
  not `jazzy`. The diff description must not re-include #103's commits.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Operator-selectable toggle with clear default (Nearest); per-layer control is more transparent than a hidden blit hint |
| Only what's needed | OK | Minimal scope — adds one bool per layer class, one settings key, one menu item; no speculative additions |
| Improve incrementally | OK | Follows up camp#122's texture-level fix; completes the Nearest path without a blanket change |
| Capture decisions, not just implementations | Watch | The default-Nearest-for-data-layers design decision is meaningful; the PR rationale (or a follow-up ADR) should record *why* this is the default — fabrication risk in QA context |
| A change includes its consequences | Watch | RGBA chart layers (`raster_layer.cpp`) have mipmaps and use `Linear`/`LinearMipMapLinear` in the GL renderer; verify that the smooth=false path in `RasterGlRenderer` does not accidentally set `Nearest` on chart-type `RasterLayer` instances (the orchestrator context says RGBA chart/basemap should stay smoothed) |
| Test what breaks | Watch | Rendering behavior is hard to unit-test, but if any snapshot/integration tests exist for the map view, they may need updating to reflect the default Nearest appearance |
| Workspace vs. project separation | OK | All changes are project-repo (camp) code |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0002 — Worktree isolation | Yes | Already in worktree `issue-camp-132`; OK |
| 0013 — progress.md vocabulary | Yes | This entry; OK |
| 0001 — Adopt ADRs | Watch | Default-Nearest-for-data-layers is a meaningful persistent design decision; capturing it in an ADR or PR description is recommended so it isn't accidentally reverted in a future "let's make things smoother" change |

### Consequences

- Changing `SmoothPixmapTransform` behavior in `gggs_tile_layer.cpp`, `raster_layer.cpp`,
  `sonar_live_cache_layer.cpp`: check whether any operator documentation or onboarding material
  describes the current rendering behavior that would need updating.
- The `RasterGlRenderer` currently branches on `Scalar` vs `Rgba` for texture filters. If
  `smooth` is plumbed as a flag on `RasterFieldItem`, ensure the flag is ignored (or always
  smooth=true) for Rgba items representing charts — so the per-layer toggle applies only to data
  layers, not accidentally to chart rasters routed through the same renderer.
- `settingsKey()`-persisted preference: verify the key namespace doesn't collide with existing
  per-layer settings from camp#126.

### Actions
- [ ] Verify that the `smooth` flag path in `RasterGlRenderer` does not override Nearest→Linear for Rgba chart rasters (RGBA chart layers must remain smoothed regardless of per-layer toggle).
- [ ] Ensure the stacked PR targets `feature/issue-103`, not `jazzy`; PR diff description should not include #103 commits.
- [ ] Add a note in the PR description capturing *why* Nearest is the default (QA use case: interpolation fabricates values, masking artifacts the operator is looking for).
- [ ] Check for any snapshot or integration tests that render the map view and may need updating to reflect the new Nearest default appearance.
