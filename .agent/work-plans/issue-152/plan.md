# Plan: Fix GDAL/OGR handle leaks in Georeferenced / VectorDataset chart loading

## Issue

https://github.com/rolker/camp/issues/152

## Context

Two classes leak GDAL/OGR handles on every chart load, confirmed by valgrind (~180 KB of OGR/PROJ state per `DepthRaster` / `VectorDataset` instance, growing monotonically across a session):

1. **`Georeferenced`** (`georeferenced.{h,cpp}`) — `extractGeoreference()` allocates `m_projectTransformation` and `m_unprojectTransformation` via `OGRCreateCoordinateTransformation` (lines 48–49). There is no destructor to free them. This base class is inherited by both `DepthRaster` and `VectorDataset`, so every instance leaks both handles.

2. **`VectorDataset::open`** (`vector/vectordataset.cpp`) — four leaks:
   - `GDALDataset*` from `GDALOpenEx` (line 22) is never `GDALClose`d.
   - Per-layer `OGRCoordinateTransformation* unprojectTransformation` (line 35) is never destroyed.
   - `OGRPointIterator* pi` is never destroyed at three sites: linestring (line 72), polygon exterior ring (line 100), polygon interior rings (line 119).
   - (`OGRFeature` is correctly freed via `DestroyFeature` — no change needed there.)

`DepthRaster` already calls `GDALClose(dataset)` (line 30 of `depth_raster.cpp`) — no dataset handle leak there. The `gdal_closer` RAII pattern is already established in `raster_layer.cpp:176–178`.

## Approach

1. **Add `virtual ~Georeferenced()`** — destroy both OGR coordinate transformations.
2. **Fix `VectorDataset::open`** — RAII-close the GDAL dataset; destroy the per-layer transformation; destroy all three `OGRPointIterator` instances.
3. **Extend `test_depth_raster.cpp`** — add a GDAL open-dataset-count baseline-delta test that proves `DepthRaster` construction and destruction leave the handle count unchanged (guards against regressions where someone removes the existing `GDALClose`).
4. **Add `test_vector_dataset_cleanup.cpp`** — synthetic GeoJSON in a temp dir; open via `VectorDataset`; assert GDAL dataset handle count returns to baseline. This catches the `GDALOpenEx` leak.

## Files to Change

| File | Change |
|------|--------|
| `src/camp/georeferenced.h` | Add `virtual ~Georeferenced();` declaration |
| `src/camp/georeferenced.cpp` | Implement destructor: call `OGRCoordinateTransformation::DestroyCT` on both members, null them |
| `src/camp/vector/vectordataset.cpp` | Wrap `GDALDataset*` in `unique_ptr` with `gdal_closer`; call `OGRCoordinateTransformation::DestroyCT(unprojectTransformation)` after each layer; call `OGRPointIterator::destroy(pi)` at all three iterator sites |
| `test/test_depth_raster.cpp` | Add GDAL open-dataset baseline-delta test for `DepthRaster` |
| `test/test_vector_dataset_cleanup.cpp` | New test: synthetic OGR file, `VectorDataset::open`, assert handle count delta == 0 |
| `CMakeLists.txt` | Register `test_vector_dataset_cleanup` under `ament_add_gtest` |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | Fix targets exactly the confirmed leak sites; no refactor beyond RAII at those sites |
| A change includes its consequences | Tests added for both fix paths; valgrind re-run required in PR |
| Test what breaks | New tests catch regressions in the GDAL dataset leak path (handle count delta); OGR transform leak covered by valgrind output in PR |
| Improve incrementally | Single PR; no structural refactor of `Georeferenced` beyond the dtor |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0002 — Worktree isolation | Yes | Worktree `issue-camp-152` / branch `feature/issue-152` already in use |
| ADR-0008 — ROS 2 conventions | Marginal | C++ source only; follow existing RAII and naming patterns in this package |
| ADR-0013 — progress.md vocabulary | Yes | progress.md maintained |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `Georeferenced` gains `virtual ~Georeferenced()` | `DepthRaster` and `VectorDataset` inherit — they will call the base dtor automatically; no extra change needed | Yes — confirmed both subclasses hold no additional OGR handles beyond those `Georeferenced` owns |
| `VectorDataset::open` RAII-closes `GDALDataset*` | Existing behaviour (close on scope exit of `open()`) is unchanged — dataset was only needed for the duration of that call | Yes |

## Open Questions

- None — fix approach is unambiguous; valgrind harness is the implementer's responsibility (run before PR).

## Estimated Scope

Single PR.
