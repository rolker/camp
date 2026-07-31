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
- [x] (must-fix) `option->exposedRect` defaults to `boundingRect()` unless `QGraphicsItem::ItemUsesExtendedStyleOption` is set on each layer — grep confirms it is set nowhere in camp. With the default, `clip_local == boundingRect()`, FBO sizing is unchanged, and the blur is **not fixed**. The plan's Open Question dismisses this ("Likely no") with flawed reasoning: if `clip_local == boundingRect()`, `mapRect(clip_local)` does *not* correct the resolution — it reproduces today's buggy full-extent sizing. Resolve by either `setFlag(ItemUsesExtendedStyleOption)` per layer, or (more robust, no flag) derive the viewport as `painter->clipBoundingRect().intersected(boundingRect())`. — `plan.md:139-143`, `plan.md:48-52`
- [x] (must-fix) `CMakeLists.txt` is missing from "Files to Change". A new `test/test_gggs_tile_layer.cpp` needs a full `ament_add_gtest(...)` + `target_include_directories` + `target_link_libraries` block (see `CMakeLists.txt:602-635`). Either add `CMakeLists.txt` to the plan, or extend the already-registered `test/test_gggs_render.cpp` (whose synthetic-store + `renderImage()` seam matches this test exactly). — `plan.md:105`
- [x] (must-fix) Testability contradiction: step 2 makes `renderImage(size, clip)` a **private** overload (`plan.md:98`), but the step-7 test calls `renderImage(size, clip_A)` directly (`plan.md:88-91`). Make the clip overload **public** (mirroring the existing public `renderImage(size)`), or add a friend/test seam. — `plan.md:61`, `plan.md:98`
- [x] (suggestion) The planned test bypasses the actual fix wiring: calling `renderImage(size, clip)` directly exercises the tile-filter seam but not `paint()`'s `exposedRect`/clip derivation — the risky part that closes the field bug. The test would pass even with the no-op derivation of finding 1. Note that `paint()`'s viewport derivation needs visual/manual verification (extend `test_gggs_render.cpp`'s `/tmp/*.png` inspection), and consider asserting improved resolution, not just tile presence/absence. — `plan.md:87-92`
- [x] (suggestion) Adding `cached_clip_` to the cache key means a **pan** (clip_scene changes each frame) now re-renders every frame, where today pan reuses the cached image via the world transform. Cheap for a viewport-sized FBO and arguably required, but the inherited "pan reuses the cached image" comment becomes stale — acknowledge the behavior change. — `plan.md:56-58`
- [x] (suggestion) `SonarLiveCacheLayer::items()` appends overviews-first then fine tiles for the ADR-0010 LOD fallback (`sonar_live_cache_layer.cpp:855-863`). The clip filter must preserve that ordering while filtering **both** pools, or a clipped region that lost its fine tiles could also drop its overview fallback (a blank gap). — `plan.md:78-80`

### Notes (verified positives)
- The item-local → scene coordinate conversion (`plan.md:39-44`) is **correct** — verified against the `setTransform(fromScale(1,-1))` + NW-anchor placement and `renderToImage()`'s `origin = scene_bounds.topLeft()` / `ortho(0,w,0,h)` MVP. Passing a `scene_bounds_` sub-rect as `scene_bounds` clips correctly (out-of-rect vertices fall outside NDC).
- The three seam callers (GggsTileLayer / RasterLayer / SonarLiveCacheLayer) are correctly identified and all share the same `paint()` → `renderImage(size)` → `renderToImage(draw, scene_bounds_, …)` shape, so the change is uniform. Matches the Issue Review's "cover all three callers" recommendation.
- ADR-0011 is the correct next number (0001–0010 present; 0004 already absent). No `RasterGlRenderer` API change is required — confirmed.
- ROS conventions: N/A (Qt/GL offscreen rendering, no topics/QoS/params).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-24 15:26 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-103 at `db8d2d7`
**Mode**: pre-push
**Depth**: Deep (reason: ADR addition — docs/decisions/0011; 3-layer seam change)
**Must-fix**: 0 | **Suggestions**: 3
**Round**: 1 | **Ship**: recommended — no must-fix; coordinate math verified by 2 adversarial passes + lead, all 3 plan-review must-fixes resolved

### Findings
- [x] (suggestion) Headless tests call renderImage(size,clip) directly; paint()'s clipBoundingRect() clip (the field-bug path) needs manual GUI verification on a >4096px store before merge — `gggs_tile_layer.cpp:472` / `viewport_clip.h`
- [x] (suggestion) Per-frame geoToMap trig in itemsIntersecting on GUI thread during pan (O(tiles)); cache tile scene-rect at load for large stores — `gggs_tile_layer.cpp:404` · `sonar_live_cache_layer.cpp:848`
- [x] (suggestion) cached_clip_ float-equality key relies on view-transform determinism for the "cheap idle" claim; add a one-line comment noting it — `viewport_clip.h:51`

### Notes
- Static analysis: cppcheck clean (parse-only syntaxError, not defects); ament_cpplint 144 errors are all pre-existing repo convention (short RASTER_*_H guards, no copyright, C-style casts) and unenforced by CI (colcon build/test only) or pre-commit — new viewport_clip.h matches its neighbors; dropped as noise. Pre-commit-enforced checks (trailing-ws, final-newline) clean.
- Governance: ADR-0007 seam fix with no renderer API change; ADR-0010 overviews-first order + no-blank-gap preserved; ADR-0011 accurate. All consequences (3 callers, cached_clip_, ADR) addressed.
- Plan drift: matches plan; positive deviation — clip derivation factored into shared raster/viewport_clip.h vs planned per-layer copies. No CMake change (test extended in already-registered test_gggs_render.cpp).
- Local Model Adversarial skipped: Ollama not installed on this host. Copilot Adversarial off (default, --copilot not passed).

## Issue Review
**Status**: complete
**When**: 2026-07-31 17:42 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #103
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Context

Re-review of issue #103 for its **LOD half only**. The visible-region half
landed in PR#173 (merged 2026-07-24, implementing ADR-0011 viewport-clip render
convention). The issue stays open for LOD selection + demand-driven loading, now
unblocked by unh_marine_autonomy#188 (ADR-0011 producer-side overview pyramids,
build_sidescan_overviews — validated: 1012 fine L13 tiles → 479 overview tiles to
a single L0 apex).

### Scope Assessment

**Well-scoped?** Yes, with one clarification item.

The LOD half has three sub-deliverables:
1. **Level selection by view scale** — pick the GGGS level whose cell size best
   matches on-screen pixel density, reading coarse tiles from the `overviews/`
   sidecar (`<level>_<row>_<col>.tif`; ADR-0011 consumer contract in
   unh_marine_autonomy). Same selection logic must apply to "natively multi-level
   layers" (the roadmap names ADR-0010, but camp's ADR-0010 describes
   SonarLiveCacheLayer eviction/overviews — not an ENC chart layer; this may mean
   a future layer type or a different ADR number. **Clarify before implementation.**).
2. **Demand-driven loading** — replace the eager whole-store pixel read (the
   observed 3.6 GB open cost from camp#102's loader) with loading only the tiles
   the viewport needs at the selected level, streaming finer tiles on zoom-in.
3. **Memory bounding** — resident tiles bounded by viewport footprint, not survey
   size.

All three are naturally scoped to `GggsTileLayer` + its new `overviews/` sidecar
scan; the visible-region clip from #173 (ADR-0011) is the natural trigger site for
demand-driven loading. The scope is right-sized for a single PR.

**Right repo?** Yes — camp UI project change.

**Dependencies**:
- unh_marine_autonomy#188 (ADR-0011 pyramid producer) — now complete; consumer
  contract is pinned: `overviews/` flat dir, `<level>_<row>_<col>.tif` naming.
- camp#173 (visible-region half) — merged; ADR-0011 viewport-clip convention is
  in place.
- camp#172 (evicted live tiles never reload) — the demand-driven load path here
  is the fix shape for #172, per the roadmap; care needed to not foreclose it.
- camp#171 (live cache adopts shared fold engine) — step 4 after this issue.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | Watch | Demand-driven streaming changes the loading experience visibly (tiles appear as user pans/zooms); a loading-state signal to the UI (status bar or layer status) is the user-facing handle. Thresholds for level selection may need QSettings knobs. |
| Enforcement over documentation | OK | Not a process change. |
| Capture decisions, not just implementations | Action needed | The LOD level-selection algorithm (pixels-per-metre → GGGS level mapping) is a design decision; record it as a camp ADR (new ADR-0013 or an addendum to ADR-0011). The choice of trigger site (viewport-clip path from ADR-0011) should also be noted. |
| A change includes its consequences | Action needed | loadDirectory() and the async loadTiles() worker are refactored; existing tests that call renderImage() or inspect tiles_ after construction will need updating. Tests for level-selection math, overview sidecar scanning, and demand-driven streaming are new work. |
| Only what's needed | Watch | The roadmap mentions #172 (evicted tile reload) as the "fix shape" — ensure the implementation enables but does not implement #172 machinery in this PR. Scope boundary: LOD selection + demand-driven loading only; eviction lifecycle (#171/#172) deferred. |
| Improve incrementally | OK | Clean boundary: this issue adds LOD and demand-driven loading; #172 adds reload of evicted tiles; #171 adds the live-cache fold-engine unification. Each step leaves the system better. |
| Test what breaks | Action needed | No automated test currently covers: level selection math, overview sidecar enumeration, or demand-driven tile streaming. The renderImage() headless tests cover the render path but not the load path. New tests needed targeting these paths. |
| Workspace vs. project separation | OK | Camp-specific change; no workspace-repo impact. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| camp ADR-0007 (RasterFieldSource seam) | Yes | items() must filter by selected LOD level; the level-selection hook is at the clip/items boundary established by ADR-0011. API change may be needed if items() gains a level parameter. |
| camp ADR-0010 (bounded eviction + overview pyramid) | Watch | The overview pyramid structure for SonarLiveCacheLayer (fold-on-evict, overviews-first draw order) is the reference design; GggsTileLayer's approach should align or consciously diverge. |
| camp ADR-0011 (viewport-clip render convention) | Yes | The demand-driven load path fits at the visible-region clip trigger in paint(); level selection feeds into items() before the render. |
| workspace ADR-0001 (capture decisions) | Yes | Level-selection algorithm and the sidecar consumer contract need an ADR. |
| workspace ADR-0002 (worktree isolation) | Yes — already satisfied | Worktree exists. |

### Consequences

- `loadDirectory()` refactor → update or extend tests in `test_gggs_render.cpp`
  and `test_gggs_tile_layer.cpp`.
- Overview sidecar `overviews/` scanning → new code path; test with a synthetic
  store that has both fine and overview tiles.
- Level-selection math → record algorithm as a camp ADR; must handle both the
  `overviews/` sidecar case (GggsTileLayer coarse levels) and the "natively
  multi-level" case once that layer type is clarified.
- items() filtering by level → verify ADR-0010's overviews-first draw order is
  preserved for SonarLiveCacheLayer (unchanged; #173 already preserved it — but
  confirm the new GggsTileLayer items() does not accidentally break the
  SonarLiveCacheLayer path).
- #172 (on-demand reload) — the demand-driven load path here is the natural hook;
  ensure interface doesn't foreclose it.

### Actions
- [x] Clarify "ADR-0010 chart layer's ENC scale ladder" reference before implementing level-selection for natively multi-level layers — confirm which ADR and which layer type are meant, or whether a new ADR is needed.
  - **RESOLVED (operator checkpoint, 2026-07-31):** the roadmap's "ADR-0010" = **unh_marine_autonomy ADR-0010** (world-model re-split; the chart layer keeps its native ENC scale ladder and needs no derived overviews), not camp ADR-0010. **Scope decision: INCLUDE the generic native multi-level wiring in this PR** — the level-selection machinery must serve both the overviews/ sidecar case (GggsTileLayer) and natively multi-level layers (chart scale ladder), even though no camp chart layer exists yet.
- [ ] Record the LOD level-selection algorithm (pixels-per-metre → GGGS level mapping, sidecar consumer contract) as a new camp ADR.
- [ ] Add tests for: level-selection math, overview sidecar enumeration, and demand-driven load path (tile streaming at the viewport clip trigger).
- [ ] Ensure the demand-driven loader enables #172 (evicted tile reload) without implementing it — keep the hook point clean and documented.
- [ ] Add a loading-state / status-bar signal so the operator knows tiles are streaming (transparency requirement).

## Plan Authored
**Status**: complete
**When**: 2026-07-31 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-103/plan.md` at `601b21d`
**Branch**: feature/issue-103 at `601b21d`
**Phases**: single

### Open questions
- [ ] Should `rescan()` also scan `overviews/` for newly-landed coarse tiles? Accepted for this PR: `rescan()` covers fine tiles only (overviews/ built at processing time, not live-updated). Follow-up if needed.

## Plan Review
**Status**: complete
**When**: 2026-07-31 18:07 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-103/plan.md` at `601b21d`
**PR**: PR-less (`/review-plan` for #103, layer worktree feature/issue-103)
**Verdict**: changes-requested

Approach is sound and well-targeted: the LOD level-selection function is a clean
pure-function seam over `gggs::Level::fromCellSize`, the demand-driven diagnosis
(loadTilesWorker loads every tile's `data_`; only viewport tiles get freed via the
`texture()` upload → the 3.6 GB residency) is correct, ADR-0013 is the right next
number, and all five 2026-07-31 issue-review actions are addressed. But the
load-*trigger* design has two correctness gaps that would ship a broken/blank layer,
plus API/ODR details to pin down.

### Findings
- [x] (must-fix) Demand-driven load never re-kicks on a pure **pan**. `paint()` calls `loadTiles()` only inside the `target != selected_level_` branch; a pan at the same level updates `load_viewport_` but never re-kicks the worker, so tiles panned into view are never `loadPixels()`'d → `itemsIntersecting()` skips them (`pixelsLoaded()==false`) → permanent blank regions. Also `load_viewport_ = clip.scene` is assigned AFTER the `loadTiles()` call in the pseudocode, so even the level-change re-kick reads a stale viewport. Re-kick when the viewport exposes not-yet-loaded intersecting tiles, and assign `load_viewport_` before kicking. — `plan.md:82-97`
- [x] (must-fix) The `loadTilesWorker()` filter breaks existing headless tests. The worker keeps only `tile->level() == selected_level_` ∧ intersects `load_viewport_`, but both fields are set **only in `paint()`**. The ~6 existing tests (`test_gggs_render.cpp` OffscreenWarp/NoData/NanNoData/ClipFilter/ClipRenderScales, `test_gggs_band_select.cpp`) call `waitForLoad()` then `renderImage()` directly, never through `paint()` — so `selected_level_` stays -1 (no tile matches; synthetic tiles are level 13) and `load_viewport_` is null → the worker loads zero tiles → blank renders → tests fail. The plan's consequences note "waitForLoad() sets selected_level_" is factually wrong (it does not). Fix: have `waitForLoad()`/the pre-paint path establish a default `selected_level_` (e.g. finest available) + `load_viewport_ = scene_bounds_`, or treat `selected_level_ == -1` / null viewport as "no filter". — `plan.md:90`, `plan.md:99-109`, `plan.md:190`
- [x] (suggestion) Level-selection input basis: `metres_per_pixel = clip.scene.width()/clip.size.width()` is in **Web-Mercator scene metres**, inflated by ~sec(latitude) (≈1.37 at Massabesic 43°N), but `gggs::Level::fromCellSize` expects **true-ground** cell size in metres — biasing selection up to ~half a level coarser. Convert to ground metres (×cos(lat) at the viewport-centre latitude) or explicitly document the approximation in ADR-0013. — `plan.md:81-83`
- [x] (suggestion) API detail: `gggs::Level::fromCellSize(float)` returns a `Level` object (marine_autonomy/include/marine_autonomy/gggs/level.h:64) — the int comes from `.level()`. The plan's `selectLodLevel` prose/signature should reflect the `.level()` extraction and the `<marine_autonomy/gggs.h>` include. — `plan.md:47-56`
- [x] (suggestion) `selectLodLevel()` in the new `lod_level_selector.h` must be declared `inline` (header included by both `gggs_tile_layer.cpp` and the test TU) or it is an ODR multiple-definition link error — follow `gggs_tile_util.h`'s `inline` pattern. — `plan.md:47-51`, `plan.md:157`
- [x] (suggestion) `resetPixels()` must always be paired with `releaseGL()` for the same tile. After `texture()` uploads, `data_` is already freed but `pixels_loaded_` stays true and `texture_` is non-null; `resetPixels()` alone clears the flag but leaves the stale texture, so a re-load re-reads `data_` yet `texture()` returns the OLD texture and never frees the new `data_`. The paint() pseudocode does pair them — state the invariant so it isn't lost in implementation. — `plan.md:40-44`, `plan.md:84-95`
- [x] (suggestion) Level switch does not reset `data_min_/data_max_` (unlike `applyBand()`); `tilesReady()`'s incremental fold only ever widens the auto-range across levels, so switching to a narrower-range overview level won't tighten the ramp. Usually benign (overviews ⊆ fine range) — note it or reset on switch. — `plan.md:112-116`
- [x] (suggestion) Pan-storm cost: once the load re-kicks on viewport change (finding 1's fix), the whole-tile abort+join runs on the GUI thread each pan; a fast pan can stall on the in-flight tile's RasterIO. Consider re-kicking only when the needed-but-unloaded tile set changes, or debounce (relates to the #173 Local Review's per-frame-trig note). — `plan.md:99-109`

### Notes (verified positives)
- `gggs::Level::fromCellSize` exists and camp links it (`marine_autonomy/gggs.h`, included via `sonar_live_tile.h`); level ordering is as the plan assumes (higher level = finer = smaller cell; L13 fine, L0 apex). Monotonic, so `selectLodLevel`'s "finest available ≤ ideal" is well-defined and both planned unit tests pass under it.
- Demand-driven diagnosis is correct: `loadTilesWorker()` currently `loadPixels()`-es every tile; only clip-intersecting tiles get `texture()`-uploaded (and their `data_` freed), so off-viewport tiles keep resident `data_` — the 3.6 GB. Filtering the worker fixes it.
- `parseTileLevel` reuses the existing anchored `isValueTile` grammar with a capture group; works for both `dir/<l>_<r>_<c>.tif` and `dir/overviews/<l>_<r>_<c>.tif` (matches on filename, not path). ADR-0013 is the correct next number (0001–0012 present, 0012 = WMS GetMap). No CMake change needed — tests extend already-registered `test_gggs_tile.cpp` / `test_gggs_render.cpp`; new selector is header-only (subject to the `inline` fix above).
- ROS conventions: N/A (Qt/GL offscreen rendering; no topics/QoS/params).

**Plan revision `04b6bc9` (2026-07-31):** all 17 Plan Review findings folded into plan.md — viewport-delta re-kick with pre-kick `load_viewport_` assignment (MF1, doubles as the pan-storm debounce), -1/no-viewport headless no-filter defaults + regression test (MF2), ground-metres conversion, `.level()` extraction, `inline` selector, resetPixels/releaseGL invariant, auto-range note in ADR-0013. Operator approved revise-plan-and-implement (no second review-plan round; pre-push review-code validates).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-31 18:50 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-103 at `e141449`
**Mode**: pre-push
**Depth**: Deep (reason: ADR-0013 addition + QtConcurrent worker concurrency + GL lifecycle + cross-layer camp_map→marine_autonomy PUBLIC link)
**Must-fix**: 0 | **Suggestions**: 2
**Round**: 1 | **Ship**: recommended — no must-fix; concurrency (value-copy snapshot + abort+join at all 5 mutation sites) and resetPixels/releaseGL GL pairing verified correct by 2 disjoint-lens adversarial passes + lead; static analysis clean; all plan-review must-fixes present as-built

### Findings
- [x] (suggestion) plan.md §4 still says scene_bounds_ "unions ALL tile extents … as before" but as-built (and ADR-0013 §Extent semantics) unions FINEST-level tiles only; the as-built sync at e141449 missed this line — doc consistency only — `.agent/work-plans/issue-103/plan.md:91`
- [x] (suggestion) selectLodLevel's std::max(mpp,1e-6) guard catches non-positive but not NaN; a NaN ground_mpp (unreachable for a valid-extent viewport, only via cos(NaN) from an out-of-range centre latitude) would hit static_cast<int>(ceil(NaN)) UB in fromCellSize — a one-token std::isfinite guard hardens the seam for the future chart-ladder caller — `src/camp_map/raster/lod_level_selector.h:657`

### Notes
- Static analysis: cppcheck clean (only an unknownMacro on Qt `slots` in map_item.h — parse limitation, not a defect); `git diff --check` clean (no trailing-ws/EOF, which ARE camp pre-commit-enforced). ament_cpplint's 80 findings (legal/copyright, readability/casting functional-cast style, short RASTER_*_H guards) are pre-existing camp house style, unenforced by camp's pre-commit (trailing-ws/EOF/yaml/xml/cmake-lint/yamllint only) or CI — dropped as noise; new files match their neighbors.
- Adversarial: Lens A (logic) and Lens B (concurrency/lifecycle) both returned clean. Lens A's "selectLodLevel requires sorted input" was a false positive (best_coarser = running max over qualifying levels, coarsest = running min — both order-independent) and discarded. Lens B confirmed the value-copy worker snapshot timing (load_viewport_ assigned at :613 before every kick), abort+join at paint level-change/rescan/applyBand/loadTiles/dtor, and the makeCurrent-failure degradation matching applyBand's GL-failed latch.
- Governance: ADR-0013 added (ws ADR-0001); ADR-0007 seam filter with no renderer API change; ADR-0011 clip-first ordering preserved. All consequences addressed (tilesReady guard, itemsIntersecting filter, resetPixels shared by setBand, headless -1 defaults regression-guarded). rescan() overview re-scan deferred per documented open question.
- Plan drift: two POSITIVE deviations the ADR reflects — finest-only extent union (vs "all extents"), and the last_kick_* moved-since-kick re-kick guard (vs the plan's simpler hasUnloadedVisibleTiles-only form) that avoids per-frame abort+join churn and infinite re-kick on a permanently-failing tile.
- Local Model Adversarial: off (--no-local, workspace#590 standing decision). Copilot Adversarial: off (default).

**Post-review fix pass (a95a773):** both R1 suggestions actioned host-inline — std::isfinite guard in selectLodLevel (+<cmath>), plan.md extent wording synced to as-built. camp rebuilt, selector tests 5/5 green.
