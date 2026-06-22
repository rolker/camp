---
issue: 96
---

# Issue #96 — GDAL dataset leak in `loadAndReprojectFile`

## Issue Review
**Status**: complete
**When**: 2026-06-22 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #96
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Scope Assessment

The issue identifies a specific resource leak in `loadAndReprojectFile` in
`src/camp2/raster/raster_layer.cpp`: two GDAL dataset handles (`GDALOpen` result
and `GDALAutoCreateWarpedVRT` result) are opened but never closed, on every return
path including early-exit paths. The sibling `initExtent()` shows the correct
pattern and already uses `GDALClose`. The suggested fix — RAII `unique_ptr` with a
custom deleter — is idiomatic C++ and covers all return paths automatically. The
HOST NOTE confirms the leak is still present at HEAD (post-#102/#104 single-band
colormap work). Scope is minimal: fix the function, add a test.

The issue correctly distinguishes this from the zoom/pan OOM (#98) — these are
separate phenomena with separate causes.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| A change includes its consequences | Watch | Issue explicitly calls for a test. Implementation must include one — the pattern of GDAL resource mis-management could silently recur without a regression guard. |
| Only what's needed | OK | Fix is narrowly scoped: RAII wrappers in one function. No interface changes, no `.msg`/`.srv` changes. |
| Improve incrementally | OK | Small, self-contained bug fix. Does not require a companion redesign. |
| Test what breaks | Watch | GDAL dataset lifecycle is hard to assert at the unit-test level (no mock). A functional/integration test (load a raster N times, verify memory does not grow unboundedly, or verify `GDALOpen` call count with a GDAL callback) may be more practical than a pure unit test. Implementation should choose a feasible test strategy and document it if a simpler option is unavailable. |
| Human control and transparency | OK | No behavioral change to the user-visible layer — same pixels, same colormap. Fix is transparent to users. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0002 — Worktree isolation | Yes | Worktree `issue-camp-96` (branch `feature/issue-96`) already exists — OK. |
| ADR-0008 — ROS 2 conventions | Yes | C++ change in a ROS 2 package. RAII is consistent with ROS 2 / modern C++ style. No convention concerns. |
| ADR-0013 — progress.md vocabulary | Yes | This entry uses `## Issue Review` — correct. |

### Consequences

- Only `raster_layer.cpp` changes; no package interface changes, no `.msg`/`.srv` files, no callers affected.
- The early-return path `if(!reprojected_dataset) return result;` also leaks `dataset` — the RAII approach covers this automatically; the implementation must not miss it.
- Callers (`RasterLayer` ctor, `setColormap`, `readSettings`) do not need changes — the fix is internal to `loadAndReprojectFile`.

### Actions
- [ ] Implement RAII fix for both GDAL handles in `loadAndReprojectFile`, covering all return paths (including the `if(!reprojected_dataset)` early return that also leaks `dataset`).
- [ ] Add a regression test (automated or documented manual) for the resource leak.

## Plan Authored
**Status**: complete
**When**: 2026-06-22 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-96/plan.md` at `c327fe2`
**Branch**: feature/issue-96 at `c327fe2`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-06-22 02:45 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-96/plan.md` at `c327fe2`
**PR**: PR-less (reviewed via worktree; `gh` unauthenticated in this environment)
**Verdict**: approve-with-suggestions

Independence note: the `## Plan Authored` entry shares the workspace agent name
("Claude Code Agent") but was a separate Sonnet sub-agent invocation; this review
is a fresh-context Opus sub-agent, so it is genuinely independent — the
`(in-context — author self-review)` annotation is omitted as it would be inaccurate.

### Findings
- [ ] (suggestion) Assert load success, not just zero open handles — if the synthetic raster fails to load/reproject, every early-return path leaks nothing, so a `GetOpenDatasets()==0` assertion passes vacuously even if `loadAndReprojectFile` never reached the warp path. Add an assertion that the load produced a non-empty image/mipmaps (e.g. via `imageReady`/`boundingRect`) so the test is a meaningful regression guard. — `plan.md:34`
- [ ] (suggestion) Prefer a baseline-delta over `EXPECT_EQ(n, 0)` — `GDALDataset::GetOpenDatasets()` counts *all* process-wide open handles, not just the layer's. Capture the count before constructing `RasterLayer` and assert equality after destruction, so the test won't flake if `Map`/GDAL internals hold an unrelated handle. — `plan.md:39`
- [ ] (suggestion) Preserve declaration order to keep close order = reprojected-then-source — `initExtent()` deliberately `GDALClose(reprojected)` before `GDALClose(dataset)` (the warped VRT references the source). Local `unique_ptr`s destruct in reverse declaration order, so keep `dataset` declared before `reprojected_dataset` (as the current code already does); a note in the implementation avoids an accidental reorder. — `plan.md:29`

## Implementation
**Status**: complete
**When**: 2026-06-22 03:30 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-96/plan.md` (synced as-built)
**Branch**: feature/issue-96
**Commits**: `bb2dd9d` (fix), `8552267` (test + CMake + plan sync)

### What was done

**RAII fix** (`src/camp2/raster/raster_layer.cpp`, `bb2dd9d`) — wrapped both GDAL
handles in `loadAndReprojectFile` in `std::unique_ptr<GDALDataset, decltype(deleter)>`
with a `[](GDALDataset* d){ if(d) GDALClose(d); }` deleter, so every return path
(normal, the `!reprojected_dataset` early return that previously leaked the source,
and the abort-flag `return {}` paths) frees both. `GDALAutoCreateWarpedVRT(dataset, …)`
→ `(dataset.get(), …)`; the `reprojected_dataset->…` accesses are unchanged
(`operator->` is transparent). Added `#include <memory>`.

- **Close order (Plan Review #3):** `dataset` is declared *before*
  `reprojected_dataset`, so reverse-declaration destruction closes the warped VRT
  first, then the source — matching `initExtent()`'s deliberate
  `GDALClose(reprojected)` then `GDALClose(dataset)` (the VRT references the source).
  A multi-line comment on the declarations records the constraint so a refactor
  won't reorder them.

**Regression test** (`test/test_raster_layer_gdal_cleanup.cpp` + `CMakeLists.txt`,
`8552267`) — follows the `test_gggs_render.cpp` pattern (`QApplication`,
`camp::map::Map`, `map.topLevelLayers()` as the `MapItem*` parent; links `camp_map`,
`Qt5::Core/Gui/Widgets/Concurrent/Positioning`, `${GDAL_LIBRARY}`). Writes a
synthetic 8×8 WGS84 single-band Float32 GeoTIFF with real georeferencing (so the
warp path runs), constructs a `RasterLayer`, flips the colormap twice
(Grayscale → Turbo; default is Viridis — each flip re-invokes `loadAndReprojectFile`),
destroys the layer, and asserts the open-dataset count.

- **Baseline-delta (Plan Review #2):** captures `GDALDataset::GetOpenDatasets(&n)`
  *before* constructing the layer and asserts the count returns to that baseline
  after destruction (delta == 0) — not `== 0` — since the count is process-wide.
- **No vacuous pass (Plan Review #1):** asserts each load actually reached the warp
  path and succeeded. `RasterLayer` exposes no "load succeeded" accessor, so the
  test uses the existing public `MapItem::status()`: `imageReady()` sets it to `""`
  only after the warp produced non-empty mipmaps, and `"(load failed)"` otherwise.
  A `waitForLoad()` helper spins the event loop until status leaves `"(loading...)"`
  and returns true iff it became empty. Also asserts `valid()` and a non-empty
  `boundingRect()`. No new accessor was added — `status()` already existed.

### Build / test

- Built the camp deps first (a fresh container had empty `core_ws/install`):
  `colcon build --packages-up-to marine_ais_msgs marine_interfaces marine_autonomy`
  in `core_ws` (the other deps — `marine_sensor_msgs`, `grid_map_ros`, `nav2_msgs`
  — resolve from `/opt/ros/jazzy`). Then `./ui_ws/build.sh camp` — clean.
- `./ui_ws/test.sh camp`: **102 tests, 0 failures, 2 skipped** (the 2 skips are the
  pre-existing offscreen-GL-gated `test_gggs_render` cases, unrelated). The new
  `test_raster_layer_gdal_cleanup` passes.
- **Non-vacuous-pass verified:** temporarily neutered the deleter to a no-op,
  rebuilt, and confirmed the test FAILS at the final `EXPECT_EQ(openDatasetCount(),
  baseline)` (delta == 6 = three loads × two handles) while the load-success guards
  still passed — proving the leaked handles register in `GetOpenDatasets()` and the
  test is a real regression guard. Restored the deleter and re-confirmed green.

### Deviations
None. All three folded Plan Review resolutions implemented as specified. Did not push.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-22 03:52 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-96 at `7016b91`
**Mode**: pre-push
**Depth**: Deep (reason: 429 total diff lines ≥ 200; GDAL resource lifecycle + async worker-thread load)
**Must-fix**: 0 | **Suggestions**: 2
**Round**: 1 | **Ship**: recommended — no must-fix findings; clean, well-tested, plan-adherent fix

### Findings
- [x] (suggestion) `LoadResult` double members lack default initializers — pre-existing, not touched by this diff, never read on the failure path (`imageReady` returns on empty mipmaps); optional `= 0.0` hardening — `src/camp2/raster/raster_layer.h:60`
- [x] (suggestion) Add a comment noting the deliberate raw `delete layer` (synchronous dtor join before the open-dataset count; `removeFromMap()` would defer and break the guarantee) — `test/test_raster_layer_gdal_cleanup.cpp:129`

### Notes
- Static analysis clean (cppcheck: no findings on the test; `raster_layer.cpp` hit a Qt `slots`-macro parser limitation, not a code issue). cpplint not installed on this host.
- Two fresh-context Claude Adversarial passes (Lens A logic, Lens B systemic, Deep prompt) independently confirmed: reverse-declaration destruction closes the warped VRT before its source per `[stmt.dcl]/3`; all five return paths (normal + two early + two abort) now free both handles; test is non-vacuous (asserts each load reached the warp path via `status()`) and race-free (dtor abort+`waitForFinished()` joins the worker before the process-wide `GetOpenDatasets()` baseline-delta check).
- Plan adherence: full — exactly the three planned files changed; all three Plan Review resolutions present in code. No scope creep.
- Build/test not re-run here (offline container); implementation entry records `./ui_ws/test.sh camp` green (102 tests, 0 failures, 2 pre-existing GL-gated skips) with a verified non-vacuous-pass check (neutered deleter → delta == 6 failure).

## Implementation
**Status**: complete
**When**: 2026-06-22 04:03 +00:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-96 at `11452ab`
**Addressed**: `## Local Review (Pre-Push)` (When 2026-06-22 03:52 +00:00, at `7016b91`)
**Commits**: `8121162` (finding 1), `11452ab` (finding 2)

### Actions
- [x] (suggestion) Default-init `LoadResult` double members — added `= 0.0` to `world_x`/`world_y`/`scale_x`/`scale_y` so a failure-path `LoadResult` carries deterministic values (matches the existing `is_scalar = false`) — `src/camp2/raster/raster_layer.h:60` (`8121162`)
- [x] (suggestion) Documented the deliberate raw `delete layer` — added a comment explaining that `removeFromMap()` defers via `deleteLater()` (verified at `src/camp2/map/layer.cpp:67`), which would run the open-dataset count before the dtor's join completes and flake; the direct `delete` keeps the join synchronous — `test/test_raster_layer_gdal_cleanup.cpp:129` (`11452ab`)

### Build / test
- Built core deps (`colcon build --packages-up-to marine_ais_msgs marine_interfaces marine_autonomy` in `core_ws` — empty `install` on this container) then `./ui_ws/build.sh camp` — clean.
- `./ui_ws/test.sh camp`: **102 tests, 0 failures, 2 skipped** (the pre-existing offscreen-GL-gated `test_gggs_render` cases). `test_raster_layer_gdal_cleanup` green.

### Deferred
None — both pre-push suggestions actioned.
