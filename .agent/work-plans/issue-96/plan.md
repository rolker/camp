# Plan: GDAL dataset leak in `loadAndReprojectFile`

## Issue

https://github.com/rolker/camp/issues/96

## Context

`loadAndReprojectFile` in `src/camp2/raster/raster_layer.cpp` (lines 125–265)
opens two GDAL dataset handles and never closes them:

- `dataset` via `GDALOpen` (line 129)
- `reprojected_dataset` via `GDALAutoCreateWarpedVRT` (line 137)

The sibling `initExtent()` (lines 77–101) already uses the correct pattern —
`GDALClose(reprojected)` (line 98) then `GDALClose(dataset)` (line 100). Every
return path in `loadAndReprojectFile` leaks one or both handles:

- Normal return (line 264): leaks both.
- `if(!reprojected_dataset) return result` (line 140): leaks `dataset`.
- Abort-flag `return {}` paths inside the scan loops: leak both.

Callers that trigger the leak on every invocation: `RasterLayer` constructor
(via `loadFile`), `setColormap()`, `readSettings()`. A colormap ramp flip leaks
two handles per flip.

## Approach

1. **Wrap both handles in RAII `unique_ptr`s** in `loadAndReprojectFile`.  
   Use a lambda deleter `[](GDALDataset* d){ if(d) GDALClose(d); }` so all
   return paths (normal, early, abort) are covered automatically — no manual
   `GDALClose` call needed.

2. **Add a regression test** `test/test_raster_layer_gdal_cleanup.cpp`:  
   Create a minimal WGS84 single-band Float32 GeoTIFF in a `QTemporaryDir`,
   construct `RasterLayer` with it, trigger `setColormap()` with two additional
   ramp types (each call re-invokes `loadAndReprojectFile`), destroy the layer
   (dtor calls `future_watcher_.waitForFinished()`), then assert
   `GDALDataset::GetOpenDatasets(&n); EXPECT_EQ(n, 0)` — zero leaked handles.
   Follow the `test_gggs_render.cpp` pattern: `QApplication` + `camp::map::Map`
   + `map.topLevelLayers()` as the `MapItem*` parent.

3. **Wire the test in `CMakeLists.txt`** — `ament_add_gtest` linking `camp_map`,
   `Qt5::Core`, `Qt5::Widgets`, `Qt5::Concurrent`, and `${GDAL_LIBRARY}`.

## Files to Change

| File | Change |
|------|--------|
| `src/camp2/raster/raster_layer.cpp` | Replace bare `auto dataset` / `auto reprojected_dataset` pointers with `unique_ptr` + lambda-`GDALClose` deleter; change `GDALAutoCreateWarpedVRT(dataset, ...)` to `GDALAutoCreateWarpedVRT(dataset.get(), ...)` |
| `test/test_raster_layer_gdal_cleanup.cpp` | New: regression test — synthetic WGS84 GeoTIFF, `setColormap` stress, `GetOpenDatasets` assertion |
| `CMakeLists.txt` | Add `ament_add_gtest(test_raster_layer_gdal_cleanup ...)` after the existing GDAL-linked tests |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Fix it completely | RAII covers all early-return and abort-flag paths. The `if(!reprojected_dataset)` early return that previously leaked `dataset` is automatically covered. No partial fix. |
| Include the test | Regression guard via `GDALDataset::GetOpenDatasets()` — makes the leak observable if RAII is ever removed or regressed. |
| No scope creep | Only `loadAndReprojectFile` body changes. No interface, no header, no callers, no other functions. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0002 — Worktree isolation | Yes | Working in worktree `issue-camp-96` on branch `feature/issue-96`. |
| ADR-0008 — ROS 2 conventions | Yes | `unique_ptr` with custom deleter is idiomatic modern C++; matches ROS 2 C++ style guide. No C-style resource management needed. |
| ADR-0013 — progress.md vocabulary | Yes | This plan is recorded under `## Plan Authored`. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `dataset` → `unique_ptr` | `GDALAutoCreateWarpedVRT(dataset, ...)` → `(...dataset.get(), ...)` | Yes — same edit pass in `loadAndReprojectFile` |
| `reprojected_dataset` → `unique_ptr` | All `reprojected_dataset->...` accesses unchanged (`operator->` on `unique_ptr` works identically); `for(auto&& band: reprojected_dataset->GetBands())` unchanged | Yes — `operator->` is transparent |
| New test file | `CMakeLists.txt` must register it and link `camp_map` + GDAL | Yes — step 3 |

## Open Questions

- [ ] No open questions — plan is review-plan-ready.

## Estimated Scope

Single PR.
