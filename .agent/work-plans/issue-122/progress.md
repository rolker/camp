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
