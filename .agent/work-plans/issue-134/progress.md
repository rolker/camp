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

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-29 05:07 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-134 at `f94bd6d`
**Mode**: pre-push
**Depth**: Deep (reason: large cross-cutting GL refactor, new ADR-0007, concurrency/lifecycle)
**Must-fix**: 2 | **Suggestions**: 5
**Round**: 1 | **Ship**: continue — two verified must-fixes (a runtime GPU-texture leak on prune; a silent scalar-colormap parity regression), both with precise low-risk fixes; warrants one more round.

Deep review: 2 disjoint-lens Claude Adversarial passes (Lens A logic, Lens B
lifecycle/concurrency). Both must-fixes were verified by the lead against the code
and against the pre-refactor (`origin/jazzy`) baseline. Static analysis limited (env
cannot build lower layers; cppcheck cannot resolve Qt headers); implementer's
`g++ -fsyntax-only` pass was clean. Plan adherence strong; ADR-0006/0002/0001
contracts verified untouched; ADR-0007 added cleanly.

### Findings
- [ ] (must-fix) Scalar-colormap parity regression: shared shader floors denominator at `max(span,1.0)`; old RasterLayer used true span via `ColorMap::color` — sub-unit-range scalar charts lose contrast — `src/camp_map/raster/raster_gl_renderer.cpp:76`
- [ ] (must-fix) Prune destroys `Entry::texture` (QOpenGLTexture) without a current GL context → runtime GPU leak as tiles churn; every other teardown path guards with makeCurrent() — `src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp:333`
- [ ] (suggestion) Scalar RasterIO read failure sets `result.ok=true` + `setStatus("")` (reports success, shows nothing) — `src/camp_map/raster/raster_layer.cpp:224`
- [ ] (suggestion) Pre-existing null-deref: `GetColorEntry(buffer[i])` may return nullptr before `ce->c3` — `src/camp_map/raster/raster_layer.cpp:280`
- [ ] (suggestion) `renderToImage` doc says null-on-shader-failure but returns a non-null transparent image (then cached) — `src/camp_map/raster/raster_gl_renderer.h:64`
- [ ] (suggestion) RGBA mipmaps generated from straight (non-premultiplied) alpha → possible dark fringes zoomed out — `src/camp_map/raster/raster_layer.cpp:379`
- [ ] (suggestion) Colormap-range floor (must-fix 1) untested; add a sub-unit-range renderer case — `test/test_raster_gl_renderer.cpp`

### Next step

Verdict is **changes-requested** → host (`/run-issue`) should dispatch
**`address-findings`** to work the two must-fixes (and the suggestions as warranted),
then re-dispatch `review-code` for a round-2 pre-push read. The diff is not pushed
until a pre-push review returns **approved**.

## Implementation
**Status**: complete
**When**: 2026-06-29 05:20 +00:00
**By**: Claude Code Agent (Claude Opus)
**Phase**: address-findings (Round 1 — pre-push review changes-requested)
**Branch**: feature/issue-134

Worked the 2 must-fixes and all 4 folded suggestions from the Round-1 Local Review,
plus the requested sub-unit-range test.

### Must-fixes
1. **Scalar-colormap true-span parity** — `raster_gl_renderer.cpp` fragment shader.
   Changed the Scalar denominator from `max(u_max - u_min, 1.0)` to
   `max(u_max - u_min, 1e-6)`. The `1.0` floor crushed contrast for any data span
   < 1.0 (silent regression once small-range RasterLayer charts joined the shared
   shader); the `1e-6` floor keeps the divide-by-zero guard for a genuinely
   degenerate (zero-width) range while normalizing over the TRUE span like the old
   `ColorMap::color`. Verified against baseline (`85e66a8~1`): GGGS and Sonar used
   the same `1.0` floor but their depth/backscatter spans are always ≫1, so they
   render identically; only sub-unit RasterLayer/GGGS data now stretches correctly.
   Negative-VALUED ranges are unaffected (span stays positive).
2. **Prune GPU-texture leak** — `sonar_live_cache_layer.cpp` `handleCatalog()`.
   The prune loop erased `Entry`s (destroying each `QOpenGLTexture`) without a current
   GL context → GPU resources leaked as tiles churn. Wrapped the erase loop in
   `renderer_.makeCurrent()` / `doneCurrent()` (guarded on `!to_prune.empty()` and a
   successful makeCurrent, mirroring the dtor's null-context handling) and reset each
   pruned texture under the current context before erase.
   **Other eviction sites verified**: the only other `tiles_` mutation that can
   destroy a live texture is `warmLoad()`'s `insert_or_assign` on a disable→re-enable
   (disable leaves textured entries intact). Guarded that loop with the same
   makeCurrent pattern (skipped on the first, empty-map warm load).

### Suggestions (folded in)
- **Silent scalar RasterIO failure** (`raster_layer.cpp`): a failed scalar read no
  longer falls through to `result.ok = true`; it returns with `ok = false` so
  `imageReady()` reports `"(load failed)"` instead of silent success + blank layer.
- **`renderToImage` doc/behavior** (`raster_gl_renderer.cpp`): on shader-compile
  failure it now returns a null `QImage()` (matching the header contract) BEFORE
  allocating/binding the FBO, instead of returning a non-null transparent image the
  caller would cache as a valid empty render.
- **RGBA mipmap alpha fringing** (`raster_layer.cpp` + shader): the RGBA upload now
  converts to `Format_RGBA8888_Premultiplied` so mipmap/linear filtering box-filters
  premultiplied texels (no dark fringes from transparent cells' black RGB). The
  shared shader's Rgba branch correspondingly emits the texel as-is (`gl_FragColor =
  c`) instead of premultiplying again (`c.rgb * c.a`). Only RasterLayer produces Rgba
  items, so the change is contained; the opaque-red unit test is unaffected.
- **Pre-existing null-deref** (`raster_layer.cpp`): guarded
  `GDALColorEntry* ce = GetColorEntry(...)` with `if(!ce) continue;` before reading
  `ce->c3` (palette index out of range now skips the pixel rather than crashing).

### Test (must-fix 1)
Added `RasterGlRendererTest.ScalarSubUnitRangeSpansColormap` to
`test/test_raster_gl_renderer.cpp`: a 2×1 R32F tile (col0 = data_min 0.0,
col1 = data_max 0.3), Grayscale, range [0.0, 0.3]. Asserts the max sample reaches the
top of the ramp (`hi.red() > 200`) — under the old `max(span, 1.0)` floor it would
only reach t = 0.3 → grey ~76, so the test FAILS under the floor and PASSES with the
true-span fix. Also asserts preserved contrast (`hi.red() - lo.red() > 150`).

### Build / test — HONEST STATUS (not built in-container)
`source setup.bash → ./ui_ws/build.sh camp` **fails to configure** here exactly as the
prior entries reported: the lower layers are unbuilt in this container, so camp's ROS
message deps are absent:

```
CMake Error at CMakeLists.txt:14 (find_package):
  Could not find a package configuration file provided by "marine_ais_msgs"
```

So `./ui_ws/test.sh camp` was **not run** — no verbatim ctest counts. I did NOT fake a
build. Instead I `g++ -std=c++17 -fsyntax-only`'d the changed ROS-free units against
system Qt5 (Core/Gui/Widgets/OpenGL/Positioning/Concurrent) + GDAL — all clean (only
the codebase's pre-existing `QOpenGLTexture::setData(void*)` deprecation warning):

- `raster_gl_renderer.cpp` — clean
- `raster_layer.cpp` — clean
- `test_raster_gl_renderer.cpp` — clean

`sonar_live_cache_layer.cpp` pulls in ROS headers so can't be syntax-checked in
isolation; its diff is a localized makeCurrent/doneCurrent guard mirroring the dtor.

The offscreen-GL render tests SKIP in-container and RUN on a GL host (by design). A
full `build.sh`/`test.sh` on a host with the lower layers built — including the new
`ScalarSubUnitRangeSpansColormap` case — is the remaining verification step.

### Next step
Host should re-dispatch `review-code` for a Round-2 pre-push read; on a GL host,
confirm `test_raster_gl_renderer` RUNs (not SKIP) and the new sub-unit case passes.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-29 05:32 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-134 at `396bb74`
**Mode**: pre-push
**Depth**: Deep (reason: large cross-cutting GL refactor, new ADR-0007, concurrency/lifecycle)
**Must-fix**: 0 | **Suggestions**: 3
**Round**: 2 | **Ship**: recommended — both Round-1 must-fixes verified correct and complete; two independent Deep-tier adversarial passes (Lens A logic, Lens B lifecycle) plus the lead found no new must-fix; only low-priority consistency nits remain.

Round-2 read of the address-findings fix commit `736922b`. Verified against the code
and the `origin/jazzy` baseline:
- **Must-fix 1 (scalar-colormap true-span)** — `raster_gl_renderer.cpp:84` `max(span, 1e-6)`
  matches `ColorMap::color`'s true-span normalization (`color_map.cpp:95-100`); new test
  `ScalarSubUnitRangeSpansColormap` discriminates the fix from the old `1.0` floor. Correct.
- **Must-fix 2 (prune GPU-texture leak)** — `sonar_live_cache_layer.cpp:342-362` guards the
  prune-erase loop with makeCurrent/doneCurrent; warmLoad's `insert_or_assign` displacement
  (`:251,:264`) guarded the same way. All texture destroy/create sites audited (Lens B):
  every one runs under a current GL context; member-decl order safe via dtor-body release.

Static analysis limited (env cannot build lower layers; cppcheck clean but Qt/ROS headers
unresolved). Governance: ADR-0007 added cleanly; ADR-0006 persistence/subscription and
ADR-0001 GUI-thread marshalling verified untouched. Plan adherence strong (ordered
per-adapter commits a→e; `RasterFieldItem::Format` resolved the plan's open RGBA question).

### Findings
- [ ] (suggestion) Gate sonar prune/warm-load `makeCurrent()` on `hasContext()` like `GggsTileLayer::applyBand` — avoids forcing the offscreen context into existence just to reset null textures — `src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp:342,251`
- [ ] (suggestion) Declare `renderer_` before `tiles_` in the two ROS-layer headers so safe teardown order is intrinsic, not dtor-body-dependent (current code is safe) — `src/camp_map/raster/gggs_tile_layer.h`, `src/camp_map/ros/live_coverage/sonar_live_cache_layer.h`
- [ ] (suggestion) Drop the now-redundant `Nearest` filter set at upload (renderer sets the scalar filter at draw time; `textureFor` already dropped it) — `src/camp_map/raster/gggs_tile.cpp:180` (pre-existing, unchanged file)

### Next step
Verdict is **approved** (Round 2, 0 must-fix). Lifecycle: **Local Review** → push / open PR
→ **triage-reviews**. The 3 suggestions are optional and non-blocking; they can be applied
before push or tracked as follow-ups. Remaining verification (unchanged): on a GL host with
the lower layers built, run `./ui_ws/build.sh camp && ./ui_ws/test.sh camp` and confirm
`test_raster_gl_renderer` RUNs (not SKIP) — including the new sub-unit case — and passes.

---

## Implementation — Round-2 consistency nits (address-findings)

**By**: Claude Code Agent (Claude Opus)
**Status**: complete
**Commit**: `7a80af9`
**Branch**: feature/issue-134

Applied all 3 low-priority consistency suggestions from the Round-2 approved review.
No behavior change.

1. **hasContext()-gate `makeCurrent()`** — `sonar_live_cache_layer.cpp` warm-load
   (`:251`) and prune (`:342`) paths now read
   `!… .empty() && renderer_.hasContext() && renderer_.makeCurrent()`, mirroring
   `GggsTileLayer::applyBand` (`gggs_tile_layer.cpp:553`). When no GL context exists
   yet, no texture was ever uploaded, so we skip `makeCurrent()` rather than forcing
   the offscreen context into existence to reset null textures. Textures that DO
   exist are still destroyed under a current context (correctness preserved).
2. **Member declaration order** — `renderer_` now declared BEFORE `tiles_` (the
   texture-holding member) in both `gggs_tile_layer.h` and `sonar_live_cache_layer.h`,
   so reverse-declaration destruction tears tiles/textures down before the
   renderer/GL context intrinsically. The dtor bodies already enforced this
   explicitly; the change makes the invariant structural. No initializer-list
   reorder warning introduced (each ctor init-list names only one early member —
   `directory_` / `base_namespace_`).
3. **Drop redundant Nearest filter set** — removed
   `setMinMagFilters(Nearest, Nearest)` at upload in `gggs_tile.cpp:180`. The
   renderer sets the scalar data-texture Nearest filter per-item at draw time
   (`raster_gl_renderer.cpp:307-309`), which fully covers this path; sampling stays
   Nearest. `setWrapMode(ClampToEdge)` retained (renderer does not set wrap mode).

**Build status**: not built in-container — `./ui_ws/build.sh camp` fails at the
configure step (`find_package` cannot locate `marine_ais_msgs`; lower layers
unbuilt in this container). Edits made cleanly; host to verify with the lower
layers built. Tests not run for the same reason.

### Next step
Lifecycle unchanged: **Local Review** (approved) → push / open PR → **triage-reviews**.
All 3 optional suggestions now applied, so nothing remains pending from Round 2. On a
GL host with lower layers built, run `./ui_ws/build.sh camp && ./ui_ws/test.sh camp`
and confirm `test_raster_gl_renderer` RUNs (not SKIP) and passes.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-29 05:52 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-134 at `8bbd868` (code under review: `7a80af9`)
**Mode**: pre-push
**Depth**: Deep (reason: large cross-cutting GL refactor, new ADR-0007, concurrency/lifecycle)
**Must-fix**: 0 | **Suggestions**: 0
**Round**: 3 | **Ship**: recommended — Round-3 read of the consistency-nits commit; 0 must-fix, 0 surviving suggestions; delta since Round-2 approval is a no-behavior-change refactor and all 3 Round-2 suggestions are applied.

Round-3 read of the address-findings consistency commit `7a80af9` (the 3 Round-2
suggestions: hasContext-gated makeCurrent, member-decl reorder, redundant Nearest
filter drop). Deep tier: 2 disjoint-lens Claude Adversarial passes (Lens A logic,
Lens B lifecycle/concurrency). Lens A clean. Lens B raised 4 findings — the lead
adversarially verified ALL 4 as false positives / non-actionable against the code:
- warmLoad `insert_or_assign` "leak": FP — `makeCurrent()` is hoisted ABOVE the
  insert loop (`sonar_live_cache_layer.cpp:32-33` before `:44`), so displaced
  textures destruct under a current context.
- stale-texture race between `items()` and `renderToImage()` (raised twice): FP —
  `renderImage()` calls both back-to-back synchronously (`:7-12`); the Qt event loop
  never spins between them, so the queued `handleCatalog` slot cannot interleave;
  textures are touched only on the GUI thread.
- dtor leak if `makeCurrent()` fails: not actionable — if makeCurrent fails, uploads
  (also gated on makeCurrent) never succeeded, so there are no GPU textures to leak.

Lead verification of the delta: member reorder is correct (no `-Wreorder`; each ctor
init-list names one early member; dtor bodies already release textures under a current
context, so the reorder is a sound defensive backstop). hasContext() gating sound
(no context ⟹ no texture uploaded). Redundant Nearest-filter drop safe (renderer sets
the scalar filter per-item at draw, `raster_gl_renderer.cpp:307-309`). Prior Round-1
must-fixes verified still in place (true-span `max(span,1e-6)` `:84`; null-image on
shader-compile-fail before FBO `:220`; Rgba emitted as-is `:93`). Governance: ADR-0007
added cleanly; ADR-0006/0001 contracts untouched. Plan adherence strong. Static
analysis limited (env cannot build lower layers; cppcheck misparses Qt namespaces);
implementer `g++ -fsyntax-only` clean.

### Findings
- [ ] No issues found. LGTM.

### Next step
Verdict is **approved** (Round 3, 0 must-fix). Lifecycle: **Local Review** → push /
open PR → **triage-reviews**. Nothing remains pending. Remaining verification
(unchanged, environmental only): on a GL host with the lower layers built, run
`./ui_ws/build.sh camp && ./ui_ws/test.sh camp` and confirm `test_raster_gl_renderer`
RUNs (not SKIP) — including the sub-unit-range case — and passes.
