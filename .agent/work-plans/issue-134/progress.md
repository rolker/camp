---
issue: 134
---

# Issue #134 — RasterFieldSource: unified source-agnostic render abstraction

## Issue Review
**Status**: complete
**When**: 2026-06-29 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #134
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Summary

Issue #134 proposes extracting a `RasterFieldSource` interface (with `bands()`,
`metadata(band)`, `read(band, extent, lod)` API) once camp#121 Part B (live tile
cache rendering through `GggsTileLayer`) is complete. The two concrete duplications
to unify are confirmed in code:

- `src/camp_map/raster/gggs_tile_layer.cpp` — fragment shader with
  `if(u_has_nodata != 0 && v == u_nodata) discard;` (NaN-unsafe)
- `src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp` — identical shader,
  explicitly marked `// NOTE: shader duplicated from GggsTileLayer; will unify via
  RasterFieldSource in camp#134`
- `src/camp_map/raster/raster_layer.cpp` — separate QPainter path (no shader/colormap);
  CPU-side path already handles NaN correctly via `std::isnan(v)` checks

The NaN NoData bug is real: verified data stores use NaN as NoData (chart/backscatter)
and the exact-equality `v == u_nodata` test silently fails for NaN in GLSL, rendering
those cells opaque.

### Scope Assessment

**Well-scoped?** Yes — the interface is small (3 methods), the three concrete
implementations are pre-identified, and the NaN fix scopes tightly to the unified
shader. The issue correctly defers until camp#121 Part B lands (the "second
implementation" that justifies extracting the abstraction).

**Right repo?** Yes — this is a camp UI/rendering concern; the camp project repo is
the correct home.

**Dependencies**:
- **camp#121 Part B** (live tile cache rendering) — hard prerequisite; implementation
  must not begin until Part B is merged. The duplication marker in
  `sonar_live_cache_layer.cpp` confirms Part B exists but is not yet integrated.
- `marine_colormap` GPU-shader lib (`unh_marine_autonomy#175` / I4) — referenced for
  per-band colormap defaults; should confirm whether the lib is available/merged before
  implementation.
- camp#121 (parent) — issue #134 is part of camp#121's scope.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Only what's needed | OK | Explicitly deferred until justified by a second implementation (camp#121 Part B). No speculative abstraction. |
| Improve incrementally | Watch | Issue body acknowledges this is a LARGE refactor and raises staging. Plan should commit to a staging approach: extract shared GL renderer first, then migrate each source, to keep each PR reviewable. |
| A change includes its consequences | Action needed | Behavior parity checklist needed: band select, colormap LUT, NoData discard (including NaN), Nearest sampling. The CPU range path in GggsTileLayer also excludes NaN via `isfinite` — consistency with the unified shader must be verified. |
| Capture decisions, not just implementations | Action needed | `RasterFieldSource` interface design (method signatures, ownership model, threading contract for `read()`) is a significant design choice — should be captured in an ADR or the plan must reference the issue's rationale explicitly. |
| Test what breaks | Watch | The NaN NoData fix is testable (chart + backscatter stores render transparent cells after fix). Plan should include a render verification step. |
| Human control and transparency | OK | Per-band colormap user-overridability is preserved in scope. |
| Workspace vs. project separation | OK | Change is entirely within the camp project repo. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0001 (Adopt ADRs) | Yes | `RasterFieldSource` interface design is a durable decision; rationale should be recorded (the issue body is a good start, but the plan should reference or create an ADR). |
| ADR-0002 (Worktree isolation) | Yes | Already satisfied — `feature/issue-134` branch exists in the camp repo. |
| ADR-0008 (ROS 2 conventions) | Partial | OpenGL/Qt rendering is not ROS-specific, but if `read(band, extent, lod)` produces ROS message data, conventions apply. |
| ADR-0013 (progress.md entry type) | Yes | This entry satisfies it. |

### Consequences

- **Removing duplicate shaders**: when the unified renderer lands, the `kFragmentShader`
  literals in both `gggs_tile_layer.cpp` and `sonar_live_cache_layer.cpp` go away — both
  files need to delegate to the shared renderer. Tests and layer-level UI (band select,
  colormap menus, NoData display) must all pass through the new path.
- **NaN shader fix**: the fix (`if(v != v) discard;` before the finite sentinel check)
  must be in the unified shader only — do NOT add it to the two duplicated shaders first,
  defeating the whole point of #134.
- **`raster_layer.cpp` migration**: bringing non-tiled rasters into the shader path
  changes their rendering semantics (CPU QPainter → GPU colormap). Verify visual parity
  with the existing output; existing NaN handling in the CPU path (`std::isnan(v)`)
  must be replicated in the shader (which the `v != v` check already covers).
- **`marine_colormap` integration**: per-band colormap defaults (depth→bathy/turbo,
  uncertainty→sequential, intensity→grayscale) are new behavior — document defaults
  and confirm they match user expectations before merging.

### Recommendations

- Confirm camp#121 Part B is merged before starting implementation.
- Confirm `marine_colormap` lib availability (`unh_marine_autonomy#175` / I4) before
  coding the colormap-default wiring.
- Stage the refactor: (1) extract `RasterFieldSource` interface + shared GL renderer
  with `GggsTileLayer` as the first adapter, (2) migrate `SonarLiveCacheLayer`, (3)
  migrate `RasterLayer`. Each stage should be a reviewable PR.
- Add a NaN render test: open a chart store (NoData = NaN), verify cells render
  transparent after the fix.
- Consider an ADR for the `RasterFieldSource` interface design (especially the
  threading contract for `read()`).

### Actions
- [ ] Confirm camp#121 Part B is merged before beginning implementation.
- [ ] Confirm `marine_colormap` lib (`unh_marine_autonomy#175`) is available before implementation.
- [ ] Behavior parity checklist: band select, colormap LUT, NaN + finite NoData discard, Nearest sampling — verify all three adapters match current behavior.
- [ ] Stage the refactor into at minimum two PRs: shared renderer + GGGS adapter first, then live cache + raster_layer.
- [ ] Verify CPU range path in GggsTileLayer (`isfinite`) stays consistent with the unified shader's NaN discard.
- [ ] Consider an ADR for `RasterFieldSource` interface design (threading contract for `read()`).

## Plan Authored
**Status**: complete
**When**: 2026-06-29 04:09 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-134/plan.md` at `6fbe701`
**Branch**: feature/issue-134 at `6fbe701`
**Phases**: single

### Open questions
- [ ] `RasterLayer` RGB/palette bands: composite on CPU → RGBA8 texture (recommended for this PR) vs per-band R32F + channel compositor — decide before implementing step 6.
