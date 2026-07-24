---
issue: 103
---

# Issue #103 — Visible-region render + LOD for GggsTileLayer (visible-region half)

## Issue Review
**Status**: complete
**When**: 2026-07-24 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #103
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Scope Assessment

The issue proposes two halves: (1) visible-region render — warp only
viewport-intersecting tiles into an FBO covering the visible extent instead of the
whole layer extent, and (2) LOD selection — pick a resolution matching pixels-per-metre
from store-generated pyramid overviews.

The orchestrator has explicitly deferred the LOD half (depends on
unh_marine_autonomy#188 and a shared pyramid library not yet available). The
**visible-region half alone is the deliverable** for this PR; the blur regression is
confirmed field-observed and confirmed code-confirmed (Roland, 2026-07-23). This is a
clean, independently deliverable scope boundary.

The change touches `gggs_tile_layer.{h,cpp}`, `raster_layer.h`, and
`sonar_live_cache_layer.h`, but camp ADR-0007 establishes that the right fix point is
the `RasterFieldSource`/`RasterGlRenderer` seam — one change there benefits all three
callers.

**Right repo?** Yes — camp UI project change, correctly in the camp repo.

**Dependencies**: The visible-region half has no external dependency. Camp#171
(eviction fold frees no memory) and camp#172 (no in-session reload of evicted tiles)
are related but explicitly out of scope. The design must not make them harder to fix.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Crisp zoomed-in rendering is the expected outcome; no new hidden state or behavior |
| Enforcement over documentation | OK | Not a process change |
| Capture decisions, not just implementations | Action needed | The viewport-clip API choice (add `viewport_bounds` param to `renderToImage` vs. caller pre-filters items + passes viewport rect as `scene_bounds`) is a design decision for the ADR-0007 seam; record as a camp ADR-0007 addendum |
| A change includes its consequences | Watch | Three code sites use the seam (GggsTileLayer, RasterLayer, SonarLiveCacheLayer) — all must be updated or benefit from the seam fix; any existing tests calling `renderToImage` need updating if signature changes |
| Only what's needed | Watch | The full issue mentions texture-cache + eviction, but the visible-region half needs neither — tile textures already exist (lazily loaded per camp#102); the fix is: filter items to viewport-intersecting tiles and size the FBO to the viewport, not to the whole extent; don't add cache machinery in this PR |
| Improve incrementally | OK | Visible-region render alone closes the field-confirmed blur regression; good scope boundary |
| Test what breaks | Watch | No automated regression test for zoom-level rendering fidelity exists; update any existing `renderToImage` call sites in tests; a new viewport-clip unit test would be valuable |
| Workspace vs. project separation | OK | Camp-specific change with no workspace-repo impact |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| camp ADR-0007 (RasterFieldSource seam) | Yes — strongly | The fix belongs at this seam; `renderToImage` must learn to accept a viewport clip rect or callers must pre-filter items and shrink `scene_bounds` to the viewport; the implementation plan should commit to one approach and record it |
| camp ADR-0010 (bounded eviction + overview pyramid) | Partially | The issue mentions evicting off-screen textures; for the visible-region-only PR this is NOT needed — existing lazy load defers reads; ensure the design doesn't interfere with ADR-0010's `SonarLiveCacheLayer` eviction logic |
| workspace ADR-0001 (capture decisions) | Yes | If the `renderToImage` signature changes or a new viewport-clip calling convention is established, capture it as a camp ADR-0007 addendum |
| workspace ADR-0002 (worktree isolation) | Yes — always | Worktree already exists for this issue |

### Consequences

- `renderToImage` signature or calling convention change → update all three callers
  (GggsTileLayer, RasterLayer, SonarLiveCacheLayer) and any tests
- If camp ADR-0007 is amended → update this review guide's ADR table (workspace
  `principles_review_guide.md`) — this is a project-level ADR only, not workspace-level,
  so workspace consequences are minimal
- Related open defects camp#171 and camp#172: the visible-region design should set up
  camp#172's on-demand disk-reload fix shape (a visible-region loader is the natural
  trigger), not close off that option

### Recommendations

- [ ] Fix at the RasterGlRenderer seam (ADR-0007), not per-layer: the `paint()` caller
  should derive the viewport-intersecting rect (intersection of `boundingRect()` and the
  painter's viewport in scene coords), filter `items()` to tiles whose extents overlap
  that rect, and pass the clipped rect as `scene_bounds` to `renderToImage` — this
  requires no signature change and immediately benefits all three render callers
- [ ] Do NOT add texture-cache eviction or LOD machinery in this PR (that is the deferred
  LOD half); keep the change minimal: viewport filter + clipped FBO size
- [ ] Record the viewport-clip calling convention as a camp ADR-0007 addendum (or a new
  camp ADR-0011) so future agents understand the seam's viewport contract
- [ ] Verify the change covers all three callers at the seam (GggsTileLayer, RasterLayer,
  SonarLiveCacheLayer), or explicitly note which callers are deferred and why
- [ ] Note in the PR that camp#172 (on-demand reload of evicted tiles) is the natural
  next step enabled by a visible-region render loop; ensure the implementation doesn't
  foreclose it
