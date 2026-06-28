---
issue: 122
---

# Issue #122 — Fix GGGS shader: honor per-band NoData instead of hardcoded `<= 0.0`

## Issue Review
**Status**: complete
**When**: 2026-06-28 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #122
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Summary

The fragment shader in `src/camp_map/raster/gggs_tile_layer.cpp` discards any
sample with `v <= 0.0` (line 80). This was correct for depth band 1, where the
mosaicker floors real returns to `>= 1` and reserves 0 for NoData. After #108
enabled arbitrary band selection, bands with valid zero or negative samples
(uncertainty, quality, signed offsets) are mis-rendered: valid `<= 0` samples
are silently discarded and the auto-range is computed only over the positive subset.

The CPU side already handles this correctly: `GggsTile::loadPixels()` calls
`GetNoDataValue()` per band (line 87), stores `has_nodata_` / `nodata_`, and the
range loop at line 104 correctly excludes only exact-NoData and non-finite values.
The gap is purely in the shader, which still hardcodes the `<= 0.0` test.

Fix: add `u_nodata` (float) and `u_has_nodata` (bool/int) uniforms to the
fragment shader; replace `if(v <= 0.0) discard;` with
`if(u_has_nodata && v == u_nodata) discard;`. Set those uniforms per-tile from
`tile->hasNoData()` and `tile->noData()` in `renderImage()`.

### Scope Assessment

**Well-scoped?** Yes — targeted shader uniform change with CPU-side plumbing already
in place. Single PR; `GggsTile::hasNoData()` and `noData()` are already public.
**Right repo?** Yes — `rolker/camp`, `src/camp_map/raster/`.
**Dependencies?** PR#124 (band select + `camp2`→`camp_map` rename) already merged.
No open blockers. The `feature/issue-122` branch already exists.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Issue is clearly motivated; fix is explicit (discard-by-NoData replaces discard-by-sign). |
| A change includes its consequences | Watch | Existing tests use UInt16 (all-positive) tiles with NoData=0. A test with a Float32 tile carrying negative valid samples and a non-zero NoData would pin the fix. No test currently exercises `v < 0` with `has_nodata_=true`. |
| Only what's needed | OK | Minimal: add two uniforms, change the discard line, plumb per-tile in `renderImage()`. No refactor needed. |
| Test what breaks | Action needed | Add a test that writes a Float32 GeoTIFF with NoData=9999, samples `[-3.0, 0.0, -1.5, 9999.0]`, loads via `GggsTile`, and asserts: `dataMin()==-3.0`, `dataMax()==0.0`, and that the NoData sample is excluded from the range. A render-level test (confirming the discard path via pixel output) should be added if offscreen GL is available (same pattern as `test_gggs_band_select.cpp`). |
| Improve incrementally | OK | Narrow fix; follows directly from the `camp#108` comment already in the shader source. |
| Workspace vs. project separation | OK | Change is in the `camp` project repo, not the workspace. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0001 (Adopt ADRs) | No | Bug fix; no new design decision needing a record. |
| ADR-0002 (Worktree isolation) | OK | `feature/issue-122` branch already checked out in `layers/worktrees/issue-camp-122`. |
| ADR-0008 (ROS 2 conventions) | Minimal | OpenGL layer, not a ROS node; not triggered. |
| ADR-0013 (progress.md vocabulary) | Yes | This entry is `## Issue Review`; plan-task should write `## Plan Authored`. |

### Consequences

- The `kFragmentShader` string is embedded in `gggs_tile_layer.cpp` — no separate shader file to update.
- `renderImage()` must set the new uniforms in the per-tile render loop; the `u_min`/`u_max` block is the natural place.
- If the NoData API on `GggsTile` changes (currently `hasNoData()` / `noData()`), the uniform-setting code in `renderImage()` must update in the same PR.
- The `max(u_max - u_min, 1.0)` denominator guard in the shader remains valid even when the range is negative (it prevents divide-by-zero on a zero-width range, not on signed ranges).

### Actions
- [ ] Add a test: Float32 GeoTIFF with NoData=9999 and negative valid samples; assert range covers negatives and NoData is excluded.
- [ ] Add shader uniforms `u_has_nodata` (int) and `u_nodata` (float); replace `if(v <= 0.0) discard` with `if(u_has_nodata != 0 && v == u_nodata) discard`.
- [ ] Set uniforms per-tile in `renderImage()` from `tile->hasNoData()` / `tile->noData()`.
- [ ] Verify uncertainty/quality bands (with valid 0 or negative values) render correctly end-to-end.

## Plan Authored
**Status**: complete
**When**: 2026-06-28 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-122/plan.md` at `fe5c2a5`
**Branch**: feature/issue-122 at `fe5c2a5`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-06-28 15:49 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-122/plan.md` at `fe5c2a5`
**PR**: PR-less (dispatched for issue #122; `gh` unauthenticated in this env — issue/review-issue context read from this progress.md)
**Verdict**: approve-with-suggestions

### Findings
- [ ] (must-fix) Step 2's per-tile uniform placement is wrong: the `u_min`/`u_max` block is at lines 497-498, **before** the `for(auto& tile : tiles_)` loop (starts line 517), so `tile` is out of scope there. The `setUniformValue("u_has_nodata"/"u_nodata")` calls must go **inside** the loop before `glDrawArrays` (line 555 — e.g. right after `texture->bind(0)` at line 550) to be genuinely per-tile and reference `tile`. As written it would not compile. — `plan.md:35`
- [ ] (suggestion) Step 3 / Files-to-Change name the comment to remove as "`[camp#122] LIMITATION`", but the actual block (`gggs_tile_layer.cpp:63-69`) is tagged `[camp#108] LIMITATION` and *references* the camp#122 follow-up. Remove that `[camp#108]` block; there is no `[camp#122]` comment to find. — `plan.md:44`
- [ ] (suggestion) `u_min`/`u_max` are layer-global (`data_min_`/`data_max_`), set once outside the tile loop — the colormap range is shared across tiles, unlike the new per-tile NoData. Pre-existing and out of scope; worth a one-line note in the commit so the asymmetry is intentional, no plan change needed. — `plan.md:41`

## Implementation
**Status**: complete
**When**: 2026-06-28 16:30 +00:00
**By**: Claude Opus

**Branch**: feature/issue-122
**Plan**: `.agent/work-plans/issue-122/plan.md`

### What changed
All in `src/camp_map/raster/gggs_tile_layer.cpp` plus two test files. All 3 Plan
Review findings addressed.

1. **Shader (`kFragmentShader`).** Added `uniform int u_has_nodata;` and
   `uniform float u_nodata;` (int, not bool, for GLSL 1.20 portability). Replaced
   `if(v <= 0.0) discard;` with `if(u_has_nodata != 0 && v == u_nodata) discard;`.
   Tiles without NoData (`u_has_nodata == 0`) now discard nothing.

2. **Per-tile uniforms (must-fix finding).** Placed the `setUniformValue` calls
   **inside** the `for(auto& tile : tiles_)` loop, right after `texture->bind(0)`
   and before the attribute-array setup / `glDrawArrays` — NOT next to the
   `u_min`/`u_max` block (which is before the loop, where `tile` is out of scope and
   would not compile, exactly as the review flagged):
   ```cpp
   program_->setUniformValue("u_has_nodata", tile->hasNoData() ? 1 : 0);
   program_->setUniformValue("u_nodata",
                             tile->hasNoData() ? float(tile->noData()) : 0.0f);
   ```

3. **Removed stale comment (suggestion finding).** Removed the
   `[camp#108] LIMITATION` block above `kFragmentShader` (the one that *referenced*
   the camp#122 follow-up — there was no `[camp#122]` comment to find), replacing it
   with a short `[camp#122]` note describing the new NoData-sentinel discard.

4. **Asymmetry note (suggestion finding).** Captured in the commit body: `u_min`/
   `u_max` stay layer-global (set once, shared colormap range) while the NoData
   uniform is per-tile (NoData can differ per tile/band). Intentional. An inline
   comment at the per-tile call documents this too.

### Tests added
- `test/test_gggs_tile.cpp` — added a `writeFloatTile` Float32 helper and
  `GggsTileTest.NegativeValidSamplesNonZeroNoData`: Float32 GeoTIFF, NoData=9999,
  samples `[-3.0, 0.0, -1.5, 9999.0]`. Asserts `hasNoData()` true, `noData()==9999`,
  `dataMin()==-3.0`, `dataMax()==0.0` (9999 excluded). **RUNS** in-container — PASSED.
- `test/test_gggs_render.cpp` — added a `writeFloatTile` helper and
  `GggsRenderTest.NoDataDiscardHonorsUniform`: Float32 tile, NoData=9999, mostly-0.0
  valid background + two positive stripes (5.0, 10.0) + an interior 9999 block.
  After `waitForLoad()`+`renderImage()`, asserts opaque pixels exist (valid 0.0
  renders, previously discarded by `v<=0`) and an enclosed transparent hole exists
  (NoData discarded). `GTEST_SKIP()` when offscreen GL is unavailable — **SKIPS**
  in-container, as designed.

### Build + test (verbatim)
Dependency workspaces had empty installs, so I built the underlay → core chain
first: `underlay_ws` (22 packages finished), `core_ws` (35 packages finished, includes
`marine_ais_msgs`). Then:
- `./ui_ws/build.sh camp` → `Summary: 1 package finished [1min 51s]`, 0 errors (only
  pre-existing `-Wsign-compare` / `-Wunused-parameter` / `-Wdeprecated-declarations`
  warnings).
- `./ui_ws/test.sh camp` → `Summary: 124 tests, 0 errors, 0 failures, 4 skipped`.
  Direct runs confirm `NegativeValidSamplesNonZeroNoData` PASSED and
  `NoDataDiscardHonorsUniform` SKIPPED ("no offscreen GL context available").

### Findings status
All 3 Plan Review findings addressed (must-fix per-tile placement; removed
`[camp#108]` comment; commit-body asymmetry note).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-28 16:16 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-122 at `1802c93`
**Mode**: pre-push
**Depth**: Standard (reason: 173 changed code lines across 3 files; rendering-correctness change)
**Must-fix**: 1 | **Suggestions**: 3
**Round**: 1 | **Ship**: continue — one mechanical test-discrimination must-fix; shader logic itself is correct, expected to converge next round

### Findings
- [ ] (must-fix) Render test `NoDataDiscardHonorsUniform` does not discriminate the fix from the reverted `v <= 0.0` bug — both `opaque > 0` and `enclosed_transparent > 0` pass under the old code (positive stripes + the `9999 > 0` block stay opaque, the discarded 0.0 background supplies the enclosed transparency). It is the only test exercising the shader change, so the fix is effectively unpinned. Assert that a valid 0.0 background pixel away from the stripes/NoData block is opaque (alpha > 0) — only the new code produces that. — `test/test_gggs_render.cpp:197`
- [ ] (suggestion) CPU range exclusion compares in double (`double(v) == nodata_`) while the shader discards in float (`v == float(nodata)`); they diverge for a NoData sentinel not exactly representable in float32 (e.g. a Float64 source band) — GPU discards, CPU keeps it in the auto-range and pollutes `u_min`/`u_max`. Benign for GGGS Float32/UInt16 data, but aligning the CPU compare to float future-proofs the consistency this PR now relies on. (Cross-pass confirmed, Lens A + Lens B.) — `gggs_tile.cpp:104`
- [ ] (suggestion) Exact-equality discard interacts with the data texture's `Linear` min/mag filter: bilinearly interpolated boundary texels never exactly equal the sentinel, so a one-texel mis-ranged halo rings every NoData region — now more conspicuous because the sentinel is a large out-of-range value that clamps to `u_max` (bright end). Structurally pre-existing; consider `Nearest` sampling for the R32F data texture. — `gggs_tile.cpp:162`
- [ ] (suggestion) New `writeFloatTile` helpers dereference `driver`/`ds` from `GetDriverByName`/`Create` without null checks (matches the pre-existing `writeTile` convention) — a guard would fail the test cleanly instead of crashing. — `test/test_gggs_tile.cpp:99`
