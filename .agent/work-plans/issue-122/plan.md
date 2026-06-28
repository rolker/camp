# Plan: Fix GGGS shader — honor per-band NoData instead of hardcoded `<= 0.0`

## Issue

https://github.com/rolker/camp/issues/122

## Context

The GGGS raster fragment shader (`src/camp_map/raster/gggs_tile_layer.cpp`,
`kFragmentShader`) discards every sample with `v <= 0.0` and auto-ranges over the
rest. This was correct for depth band 1, where the mosaicker floors real returns to
`>= 1` and reserves 0 for NoData. After #108 enabled arbitrary band selection, bands
with legitimately zero or negative values (uncertainty, quality, signed offsets) are
mis-rendered: valid `<= 0` samples are silently discarded and the colormap range is
computed only over the positive subset.

The CPU side is already correct: `GggsTile::loadPixels()` calls
`GetNoDataValue()` per selected band (line 87 in `gggs_tile.cpp`), stores
`has_nodata_` / `nodata_`, and the range loop at line 104 excludes only exact-NoData
and non-finite values. The gap is purely the shader's hardcoded sign test.

The `GggsTile` API already exposes `hasNoData()` and `noData()` (public, from
`gggs_tile.h`); no new CPU-side infrastructure is needed.

## Approach

1. **Add two shader uniforms** — add `uniform int u_has_nodata` and
   `uniform float u_nodata` to `kFragmentShader` in `gggs_tile_layer.cpp`.
   Replace `if(v <= 0.0) discard;` with
   `if(u_has_nodata != 0 && v == u_nodata) discard;`.
   The `u_has_nodata` int (not bool, for GLSL 1.20 portability) guards the
   discard so tiles without NoData discard nothing.

2. **Set uniforms per-tile in `renderImage()`** — in the per-tile render loop
   (after the `u_min`/`u_max` block, around line 498), add:
   ```cpp
   program_->setUniformValue("u_has_nodata", tile->hasNoData() ? 1 : 0);
   program_->setUniformValue("u_nodata",
                             tile->hasNoData() ? float(tile->noData()) : 0.0f);
   ```
   The uniforms are per-tile because tiles in a non-uniform store could in theory
   carry different NoData values (the API allows it per band).

3. **Remove the stale comment** — the `[camp#122] LIMITATION` comment block
   above `kFragmentShader` documents the exact gap being fixed here; remove it
   once the fix lands.

4. **Add a GggsTile test for negative-valid-sample / non-zero NoData** — in
   `test/test_gggs_tile.cpp`, add a new `writeTile`-style helper (or inline a
   Float32 variant) that writes a Float32 GeoTIFF with NoData=9999 and samples
   `[-3.0f, 0.0f, -1.5f, 9999.0f]`. Assert: `hasNoData()` true, `noData()==9999`,
   `dataMin()==-3.0`, `dataMax()==0.0`, and the NoData sample is excluded from
   the range. This test exercises the CPU-side path that was already correct but
   had no negative-sample coverage.

5. **Add a render-level test for the NoData discard path** — in
   `test/test_gggs_render.cpp`, add a new test `NoDataDiscardHonorsUniform`:
   write a Float32 tile with NoData=9999 and two distinct positive values plus
   one zero sample (valid) and one 9999 sample (NoData). After `waitForLoad()` +
   `renderImage()`, assert the rendered image has both opaque and transparent
   pixels (the 9999 NoData sample is discarded → transparent) and that the zero
   sample is rendered opaque (it was previously discarded by the `<= 0.0` guard).
   Skip with `GTEST_SKIP()` if offscreen GL is unavailable (same pattern as the
   existing render test).

## Files to Change

| File | Change |
|------|--------|
| `src/camp_map/raster/gggs_tile_layer.cpp` | Replace `kFragmentShader` discard line; add `u_has_nodata`/`u_nodata` uniforms; set them per-tile in `renderImage()`; remove stale `[camp#122] LIMITATION` comment |
| `test/test_gggs_tile.cpp` | Add `NegativeValidSamplesNonZeroNoData` test with a Float32 GeoTIFF writer |
| `test/test_gggs_render.cpp` | Add `NoDataDiscardHonorsUniform` offscreen render test |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Shader change is explicit and minimal; comment explains old behaviour and new |
| A change includes its consequences | Tests added in the same PR; no external interfaces change |
| Only what's needed | Two uniforms, one discard line, two tests — no refactor |
| Improve incrementally | Narrow targeted fix; no other behaviour changed |
| Test what breaks | New tests cover the exact failure mode (negative valid samples, non-zero NoData) not covered by existing tests |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0001 (Adopt ADRs) | No | Bug fix; the design choice (per-tile NoData uniform) is documented in this plan and will appear in the commit message |
| ADR-0002 (Worktree isolation) | Yes | Already on `feature/issue-122` in `layers/worktrees/issue-camp-122` |
| ADR-0008 (ROS 2 conventions) | No | OpenGL layer in Qt, not a ROS node |
| ADR-0013 (progress.md vocabulary) | Yes | `## Plan Authored` entry will follow |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `kFragmentShader` (add uniforms) | `ensureProgram()` remains unchanged — uniform locations are resolved dynamically by `setUniformValue()` | N/A — no change needed |
| Per-tile uniform set in `renderImage()` | If `GggsTile::hasNoData()`/`noData()` API changes, the renderImage block must update | Yes — in same PR |
| Stale `[camp#122] LIMITATION` comment | No other file references this comment | Yes — removed in step 3 |

## Open Questions

- [ ] No open questions — plan is review-plan-ready.

## Estimated Scope

Single PR. All changes are in `gggs_tile_layer.cpp` and two existing test files.
