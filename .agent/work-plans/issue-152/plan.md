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

1. **Add `virtual ~Georeferenced()`** — destroy both OGR coordinate transformations. Also `= delete` the copy ctor/assignment (rule-of-three: the class now owns handles; both subclasses are heap-only so no live call site changes). *(review-plan suggestion folded)*
2. **Fix `VectorDataset::open`** — RAII-close the GDAL dataset; destroy the per-layer transformation **at end-of-layer** (else all but the last layer leak); destroy every `OGRPointIterator` **at its site**, including the interior-ring iterator **inside the ring loop** (`:119`, reassigned per ring). *(review-plan must-fix folded)*
3. **Extract a testable seam (operator decision, post-review).** The original `open()` interleaved the GDAL/OGR resource lifecycle with construction of the project item graph (`Point`/`LineString`/`Polygon` + `connect(autonomousVehicleProject(), …)`), so any test driving `open()` would have to link ~all 52 app TUs and stand up a `QApplication`-backed `AutonomousVehicleProject` — the "bounded source list" balloons. Instead split the pure parse into a new MissionItem-free TU:
   - `src/camp/vector/vector_parse.{h,cpp}` — `camp::vector::parseVectorLayers(GDALDataset*)` returns plain WGS84 geometry (`ParsedLayer`/`ParsedGeometry`) and owns the per-layer transform + per-ring iterator lifecycle (the #152 leak sites). No Qt-item / project coupling.
   - `VectorDataset::open` RAII-opens the dataset, calls `extractGeoreference`, then `buildItems(parseVectorLayers(...))`.
   - `VectorDataset::buildItems(layers)` — the unchanged item-graph construction, now fed plain data. Coordinate handling preserved verbatim per geometry type.
4. **Extend `test_depth_raster.cpp`** — add a GDAL open-dataset-count baseline-delta test (`LoadAndDestroyLeavesNoOpenDataset`). NOTE: it guards the **pre-existing** `GDALClose`, not the new dtor — `GetOpenDatasets()` can't see the `OGRCoordinateTransformation` handles the dtor frees (valgrind-only). Stated in the test comment. *(review-plan suggestion folded)*
5. **Add `test_vector_dataset_cleanup.cpp`** — links the standalone `vector_parse` TU (no item graph). Synthetic 2-layer GeoPackage (point + linestring + polygon-with-hole per layer); calls `parseVectorLayers`; asserts every parse branch ran (non-vacuous) and the GDAL open-dataset count returns to baseline. The transform/iterator leaks are **valgrind-only** (run the binary with `--leak-check=full`; definitely-lost must be 0) — documented in the test + CMake comments.

## Files to Change

| File | Change |
|------|--------|
| `src/camp/georeferenced.h` | Add `virtual ~Georeferenced();` + `= delete` copy ctor/assignment |
| `src/camp/georeferenced.cpp` | Implement destructor: `OGRCoordinateTransformation::DestroyCT` on both members |
| `src/camp/vector/vector_parse.h` | **New** — `ParsedGeometry`/`ParsedLayer` + `parseVectorLayers(GDALDataset*)` (MissionItem-free) |
| `src/camp/vector/vector_parse.cpp` | **New** — pure parse; per-layer transform + per-ring iterator created and destroyed; `OGRFeature::DestroyFeature` retained |
| `src/camp/vector/vectordataset.h` | Forward-declare `camp::vector::ParsedLayer`; declare private `buildItems` |
| `src/camp/vector/vectordataset.cpp` | `open()` → RAII dataset + `extractGeoreference` + `buildItems(parseVectorLayers(...))`; new `buildItems` holds the item-graph construction |
| `test/test_depth_raster.cpp` | Add GDAL open-dataset baseline-delta test (guards pre-existing `GDALClose`; dtor leak is valgrind-only) |
| `test/test_vector_dataset_cleanup.cpp` | **New** — links `vector_parse` TU; synthetic 2-layer GPKG; asserts parse branches + handle-count baseline; transform/iterator leaks valgrind-only |
| `CMakeLists.txt` | Add `vector_parse.cpp` to executable sources; register `test_vector_dataset_cleanup` |

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
