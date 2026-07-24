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

## Plan Authored
**Status**: complete
**When**: 2026-07-24 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-103/plan.md` at `83d9945`
**Branch**: feature/issue-103 at `83d9945`
**Phases**: single

### Open questions
- [ ] `option->exposedRect` under Qt's BSP update scheme may equal `boundingRect()` for large layers even when only part is visible — should we add a viewport-rect fallback via `painter->clipBoundingRect()`?

## Plan Review
**Status**: complete
**When**: 2026-07-24 14:04 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-103/plan.md` at `83d9945`
**PR**: PR-less (`--issue 103`, layer worktree)
**Verdict**: changes-requested

The approach is fundamentally sound — the fix belongs at the ADR-0007
`renderToImage()` seam, the coordinate math is correct (verified against
`raster_gl_renderer.cpp`'s MVP build), the scope is a clean single PR, and it
correctly defers the LOD half. But one **critical correctness gap** would make
the fix a silent no-op, plus two small mechanical must-fixes.

### Findings
- [ ] (must-fix) `option->exposedRect` defaults to `boundingRect()` unless `QGraphicsItem::ItemUsesExtendedStyleOption` is set on each layer — grep confirms it is set nowhere in camp. With the default, `clip_local == boundingRect()`, FBO sizing is unchanged, and the blur is **not fixed**. The plan's Open Question dismisses this ("Likely no") with flawed reasoning: if `clip_local == boundingRect()`, `mapRect(clip_local)` does *not* correct the resolution — it reproduces today's buggy full-extent sizing. Resolve by either `setFlag(ItemUsesExtendedStyleOption)` per layer, or (more robust, no flag) derive the viewport as `painter->clipBoundingRect().intersected(boundingRect())`. — `plan.md:139-143`, `plan.md:48-52`
- [ ] (must-fix) `CMakeLists.txt` is missing from "Files to Change". A new `test/test_gggs_tile_layer.cpp` needs a full `ament_add_gtest(...)` + `target_include_directories` + `target_link_libraries` block (see `CMakeLists.txt:602-635`). Either add `CMakeLists.txt` to the plan, or extend the already-registered `test/test_gggs_render.cpp` (whose synthetic-store + `renderImage()` seam matches this test exactly). — `plan.md:105`
- [ ] (must-fix) Testability contradiction: step 2 makes `renderImage(size, clip)` a **private** overload (`plan.md:98`), but the step-7 test calls `renderImage(size, clip_A)` directly (`plan.md:88-91`). Make the clip overload **public** (mirroring the existing public `renderImage(size)`), or add a friend/test seam. — `plan.md:61`, `plan.md:98`
- [ ] (suggestion) The planned test bypasses the actual fix wiring: calling `renderImage(size, clip)` directly exercises the tile-filter seam but not `paint()`'s `exposedRect`/clip derivation — the risky part that closes the field bug. The test would pass even with the no-op derivation of finding 1. Note that `paint()`'s viewport derivation needs visual/manual verification (extend `test_gggs_render.cpp`'s `/tmp/*.png` inspection), and consider asserting improved resolution, not just tile presence/absence. — `plan.md:87-92`
- [ ] (suggestion) Adding `cached_clip_` to the cache key means a **pan** (clip_scene changes each frame) now re-renders every frame, where today pan reuses the cached image via the world transform. Cheap for a viewport-sized FBO and arguably required, but the inherited "pan reuses the cached image" comment becomes stale — acknowledge the behavior change. — `plan.md:56-58`
- [ ] (suggestion) `SonarLiveCacheLayer::items()` appends overviews-first then fine tiles for the ADR-0010 LOD fallback (`sonar_live_cache_layer.cpp:855-863`). The clip filter must preserve that ordering while filtering **both** pools, or a clipped region that lost its fine tiles could also drop its overview fallback (a blank gap). — `plan.md:78-80`

### Notes (verified positives)
- The item-local → scene coordinate conversion (`plan.md:39-44`) is **correct** — verified against the `setTransform(fromScale(1,-1))` + NW-anchor placement and `renderToImage()`'s `origin = scene_bounds.topLeft()` / `ortho(0,w,0,h)` MVP. Passing a `scene_bounds_` sub-rect as `scene_bounds` clips correctly (out-of-rect vertices fall outside NDC).
- The three seam callers (GggsTileLayer / RasterLayer / SonarLiveCacheLayer) are correctly identified and all share the same `paint()` → `renderImage(size)` → `renderToImage(draw, scene_bounds_, …)` shape, so the change is uniform. Matches the Issue Review's "cover all three callers" recommendation.
- ADR-0011 is the correct next number (0001–0010 present; 0004 already absent). No `RasterGlRenderer` API change is required — confirmed.
- ROS conventions: N/A (Qt/GL offscreen rendering, no topics/QoS/params).
