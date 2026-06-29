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

## Plan Review
**Status**: complete
**When**: 2026-06-29 04:16 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-134/plan.md` at `6fbe701`
**PR**: PR-less (`--issue 134`, layer worktree `issue-camp-134`)
**Verdict**: approve-with-suggestions

Independent review (plan authored by a Sonnet agent; this review is a fresh
host-dispatched Opus sub-agent — not author self-review).

### Findings
- [ ] (suggestion) Single PR migrates 3 adapters + 2 new components (~9–11 files), exceeding the >3-component split heuristic and overriding review-issue's 2-PR staging action. Operator-decided and documented — accept, but enforce ordered per-adapter commits (steps 4→5→6) so the PR is reviewable commit-by-commit — `plan.md:26`, `plan.md:117`
- [ ] (suggestion) RasterLayer RGBA/palette path vs single-band R32F renderer: recommended option (a) (CPU composite → RGBA8) needs a colormap-bypass mode in the renderer, but the sketched `render(items, data_min, data_max, mvp)` + `RasterFieldItem{texture, nodata}` only expresses scalar R32F+LUT. Add a format/mode field to `RasterFieldItem` (or a render overload) and resolve the open question before step 6 — `plan.md:46-52`, `plan.md:142-146`
- [ ] (suggestion) ADR-table label wrong: "ADR-0001 (camp: Adopt ADRs)" — camp ADR-0001 is "TopicBridge and the executor contract"; no camp "Adopt ADRs" ADR exists (that's a workspace ADR). Also camp ADR-0006 (live tile cache) governs `SonarLiveCacheLayer` — confirm the render-path migration leaves its persistence/subscription contract untouched and note it in the table — `plan.md:124-129`
- [ ] (suggestion) New `test_raster_gl_renderer.cpp` requires an offscreen GL context; mirror `test_gggs_render`'s harness and confirm CI provides GL — `plan.md:93-96`

## Implementation
**Status**: complete
**When**: 2026-06-29 04:51 +00:00
**By**: Claude Opus

**Branch**: feature/issue-134 — commits `85e66a8`..`10f1159` (5, ordered)
**Plan**: `.agent/work-plans/issue-134/plan.md` at `85e66a8` (synced)

### What landed

Full consolidation in one PR, committed step-by-step (review suggestion 1):

- **(a) `85e66a8`** — ADR-0007 + `raster/raster_field_source.h` (interface:
  `bands()`/`metadata()`/`items()`/`dataRange()` + `RasterFieldItem`) +
  `raster/raster_gl_renderer.{h,cpp}` (the shared renderer). One compiled program +
  colormap LUT per GL context, the unified fragment shader, the geo→Web-Mercator
  tessellation, and the data-texture filter all live here now. The renderer owns
  its own offscreen context/surface/FBO; sources release their textures between
  `makeCurrent()`/`doneCurrent()`.
- **(b) `26bedba`** — `GggsTileLayer`: deleted its kVertex/kFragment shaders,
  `ensureProgram`/`ensureLut`, per-vertex tessellation and own GL context/FBO;
  implements `RasterFieldSource`; `renderImage()` delegates. QtConcurrent load +
  atomic `pixelsLoaded()` unchanged.
- **(c) `f517d27`** — `SonarLiveCacheLayer`: deleted the duplicated shader block +
  its `// NOTE: shader duplicated; unify via camp#134` marker; delegates. ADR-0006
  invariants (persistence, opt-in subscription, reconcile/prune, GUI-thread
  marshalling) untouched — only `renderImage()`'s GL internals changed.
- **(d) `3f2e855`** — `RasterLayer`: replaced the QPainter/mipmap path with the GL
  renderer (own offscreen context via the shared renderer). Adopts the GggsTileLayer
  placement convention (NW anchor + `fromScale(1,-1)`). Async GDAL load + abort +
  RAII handle-close kept; a scalar colormap change is now an LUT re-bake (no
  re-warp).
- **(e) `10f1159`** — tests.

### NaN fix location

Lands ONCE, in the unified fragment shader in `raster_gl_renderer.cpp`:
`if(v != v) discard;` (NaN, GLSL 1.20-portable) before
`if(u_has_nodata != 0 && v == u_nodata) discard;` (finite sentinel). All three
adapters inherit it; the two old per-layer copies are gone.

### RGBA format handling (review suggestion 2)

`RasterFieldItem::Format{Scalar,Rgba}` resolves the open question. Scalar bands →
R32F + colormap LUT (NaN/finite discard); RasterLayer's palette/RGB charts →
CPU-composited RGBA8 sampled directly (LUT bypassed; transparency from the
composited alpha), with mipmaps for zoomed-out LOD. A `geographic` flag lets the
one renderer serve both lat/lon tiles (warped in-shader) and RasterLayer (already
GDAL-reprojected → a single linear quad). Per-band channel select for colour files
is left a future issue (composited on CPU as today).

### Parity verification

- Multi-band select, colormap LUT + per-band/auto-range, Nearest sampling: preserved
  per adapter (the renderer reproduces the prior vertex math + mvp + Nearest filter
  exactly; the geographic warp is byte-for-byte the old tessellation).
- NoData discard NaN **and** finite: now both, in the one shader.
- CPU range fold excludes NaN consistently in all three (`std::isfinite` in
  `GggsTile::loadPixels`, `SonarLiveTile::refoldRange`, and RasterLayer's range loop),
  so CPU range and shader discard agree.
- RasterLayer GPU change: scalar colormap, palette/RGB (RGBA bypass), and NoData
  transparency covered; north-up placement mirrors GggsTileLayer's verified convention.

### Review suggestions addressed

1. Ordered per-adapter commits — done (a→e). 2. Format/mode field — done
(`RasterFieldItem::Format` + `geographic`). 3. ADR table fix — ADR-0007 + plan
reference camp ADR-0001 correctly (*TopicBridge…*, not "Adopt ADRs") and state
ADR-0006 governs `SonarLiveCacheLayer` and is untouched. 4. Test harness —
`test_raster_gl_renderer.cpp` mirrors `test_gggs_render`'s offscreen-GL harness +
`offscreenGLAvailable()` SKIP guard.

### Build / test — HONEST STATUS (env cannot do a full build)

`source setup.bash → ./ui_ws/build.sh camp` **fails to configure**: every lower
layer is unbuilt in this container (`underlay/core/platforms/site/sensors/simulation`
install dirs are empty; `setup.bash` warns "exists but is not built"), so camp's
ROS message deps are absent:

```
CMake Error at CMakeLists.txt:14 (find_package):
  Could not find a package configuration file provided by "marine_ais_msgs"
```

Building all lower layers from source here is impractical, so `./ui_ws/test.sh camp`
was **not run** — no verbatim ctest counts to report. I did NOT fake a build.

To validate the change I instead ran standalone `g++ -std=c++17 -fsyntax-only`
against system Qt5 + GDAL on every new/migrated ROS-free unit — all clean (only the
pre-existing `QOpenGLTexture::setData(void*)` deprecation warning the codebase
already uses):

- `raster_gl_renderer.cpp` — clean
- `gggs_tile_layer.cpp` — clean
- `raster_layer.cpp` — clean
- `test_gggs_render.cpp` — clean
- `test_raster_gl_renderer.cpp` — clean

`sonar_live_cache_layer.cpp` pulls in ROS headers so can't be syntax-checked in
isolation; its diff mirrors the (compiling) GggsTileLayer migration exactly.

The offscreen-GL render tests (`test_gggs_render`, `test_raster_gl_renderer`) are
designed to **SKIP in-container** and **RUN on a GL host** — expected. A full
`build.sh`/`test.sh` on a host with the lower layers built (and verbatim counts) is
the remaining verification step.

### Next step

Run `./ui_ws/build.sh camp && ./ui_ws/test.sh camp` on a host with the lower layers
built; confirm a clean build and that the offscreen-GL render tests RUN (not SKIP)
and pass. Then code review.
