# Plan: Depth-at-cursor for GGGS store layers

## Issue

https://github.com/rolker/camp/issues/180

## Context

`AutonomousVehicleProject::getDepth()` walks `m_depthRasters` — a list of
`DepthRaster` objects, one per loaded background chart that carries a Float32
depth band. GGGS store layers (`GggsTileLayer`, spawned by `GggsStoreSource`)
never register in this walk, so hovering over an open bathy store silently
omits depth from the status bar.

`GggsTile` already loads pixel data off-thread (via `loadPixels()`, driven over
**all** tiles by `GggsTileLayer::loadTilesWorker()`) and holds a
`geo_transform_[6]` + extent bounds from construction. Historically `texture()`
freed the CPU buffer (`data_`) after the first paint uploaded it to the GPU, so
a point query would have had to re-read from GDAL. **Plan Review finding #4**
(operator-approved) requires the per-cursor-move cost to be a cheap in-memory
lookup with no synchronous GDAL open on the GUI thread. We therefore **retain
`data_` resident** after the GPU upload (stop freeing it in `texture()`) and
sample the resident buffer. Trade-off: a painted tile now keeps both a CPU copy
and its GL texture (the pre-#180 "halve resident memory per tile" optimization
is dropped for painted tiles); acceptable at the store scale this lands against,
noted as revisit-if-observed.

Sampling reads `data_`/`nodata_` **only** through the tile's `pixelsLoaded()`
acquire gate. That gate is the same happens-before the paint path uses, and it
is set by `loadPixels()` *after* `nodata_`/`has_nodata_` are populated — so
`sampleAt()` can never read the deferred NoData members while they are unset
(**Plan Review finding #1**, must-fix).

GGGS store values are ellipsoidal up-positive heights, not chart-datum depths;
the two need distinct labels in the status bar.

## Plan Review findings folded in (operator-approved 2026-07-31)

1. **[must-fix]** NoData mask must not read `has_nodata_`/`nodata_` while unset.
   Resolved: `sampleAt()` early-returns NaN unless `pixelsLoaded()` (acquire) is
   true; `loadPixels()` populates the NoData members *before* the release store,
   so a true gate guarantees they are valid. A unit test samples an unloaded tile
   and asserts NaN.
2. **[suggestion]** Tests beyond `sampleAt()`. Resolved: add a `tileLevel()`
   basename-parse test, and a headless `getElevation()` test (extent filter,
   descending-level "finest covering tile wins", out-of-extent → NaN, NoData →
   NaN). `getStoreElevation()`'s enabled-layer walk is covered by a manual
   protocol note (constructing a full `AutonomousVehicleProject` in a unit test
   is disproportionate; the walk is a thin `dynamic_cast` + `isVisible()` filter).
3. **[suggestion]** Honor the Layers-tab enable/disable state. Resolved:
   `getStoreElevation()` skips layers where `!isVisible()` (the Layers-tab
   checkbox maps to `MapItem::isVisible()` — `map.cpp:89`/`:111`), per ADR-0003
   §2's enabled-layer contract. Decision recorded: a hidden store does **not**
   report elevation.
4. **[suggestion]** No synchronous GDAL open per cursor move on the GUI thread.
   Resolved by retaining `data_` (above): `sampleAt()` is a pure in-memory index
   into the resident buffer.

## Approach

1. **`GggsTile::sampleAt(lon, lat) const`** — return NaN immediately unless
   `pixelsLoaded()` (acquire) — this guards both `data_` and the deferred
   `nodata_`/`has_nodata_` (finding #1). Invert the north-up geotransform
   (`geo_transform_`, `geo[2]==geo[4]==0`) to pixel (col, row); reject
   out-of-bounds → NaN. Index the **resident** `data_` buffer (finding #4 — no
   GDAL open). Apply the same value filter as `loadPixels()`'s range loop
   (`!isfinite` or `has_nodata_ && v == float(nodata_)`) → NaN. Otherwise return
   the sample. Reads only members published by the tile's acquire gate, so it is
   safe on the GUI thread against the load worker.

   Enabling change: **`GggsTile::texture()` no longer frees `data_`** after the
   GPU upload — the resident buffer is what `sampleAt()` indexes.

2. **`camp::raster::tileLevel(basename)`** (new, in `gggs_tile_util.h`) — parse
   the LEVEL (first digit group) from a `<level>_<row>_<col>.tif` basename;
   `-1` if not a value tile. Directly unit-tested (finding #2).

3. **`GggsTileLayer::getElevation(QGeoCoordinate) const`** — collect `tiles_`
   whose lat/lon extent contains the point, pair each with its `tileLevel()`,
   sort by descending level (finest covering tile first), call `sampleAt()` on
   each until a non-NaN result. NaN if nothing covers the point. The finest-level
   tile is queried regardless of the rendered LOD (inspection > display — issue
   note, #103 follow-on).

4. **`AutonomousVehicleProject::getStoreElevation(QGeoCoordinate) const`** — walk
   the Map's `topLevelLayers()` child list dynamically on each call,
   `dynamic_cast` to `GggsTileLayer*`, **skip layers where `!isVisible()`**
   (finding #3 — Layers-tab enabled state, ADR-0003 §2), call `getElevation()`,
   return first non-NaN. No separate list: the dynamic walk tracks layer
   add/remove automatically and avoids a dangling-pointer risk on layer teardown.

5. **`ProjectView::mouseMoveEvent()`** — try `getStoreElevation()` first
   (stores have precedence; issue precedence rule); if non-NaN, append
   `" Elev: X (ellipsoid)"`. Then try `getDepth()` (chart rasters); if
   non-NaN, append `" Depth: X"`. Both can appear when a store and a chart
   overlap. The two-label design makes the datum difference explicit until
   the datum service (#288) arrives.

6. **Unit tests** — in `test_gggs_tile.cpp`: `sampleAt()` in-bound / out-of-bound
   / NoData-pixel / **unloaded-tile → NaN** (finding #1), plus a `tileLevel()`
   basename-parse test (finding #2). New `test/test_gggs_elevation.cpp` (headless,
   GL-free — mirrors `test_gggs_rescan.cpp`): a two-level overlapping tile-set
   asserts `getElevation()` returns the finest covering tile's value, out-of-extent
   → NaN, all-NoData → NaN (finding #2).

## Files to Change

| File | Change |
|------|--------|
| `src/camp_map/raster/gggs_tile.h` | Declare `float sampleAt(double lon, double lat) const` |
| `src/camp_map/raster/gggs_tile.cpp` | Implement `sampleAt()` (resident-buffer index, `pixelsLoaded()` gate); stop freeing `data_` in `texture()` |
| `src/camp_map/raster/gggs_tile_util.h` | Add `int tileLevel(const QString& filename)` |
| `src/camp_map/raster/gggs_tile_layer.h` | Declare `float getElevation(const QGeoCoordinate& location) const`; `#include <QGeoCoordinate>` |
| `src/camp_map/raster/gggs_tile_layer.cpp` | Implement `getElevation()`: extent filter, level sort, `sampleAt()` loop |
| `src/camp/autonomousvehicleproject.h` | Declare `float getStoreElevation(const QGeoCoordinate&) const` |
| `src/camp/autonomousvehicleproject.cpp` | Implement `getStoreElevation()`: dynamic LayerList walk, `isVisible()` filter |
| `src/camp/projectview.cpp` | Update `mouseMoveEvent()` to call both query methods with appropriate labels |
| `test/test_gggs_tile.cpp` | Add `sampleAt()` + `tileLevel()` unit tests |
| `test/test_gggs_elevation.cpp` (new) | Headless `getElevation()` tests (extent/level/NoData) |
| `CMakeLists.txt` | Register `test_gggs_elevation` gtest target |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | No separate registration list; datum conversion deferred; LOD complexity deferred to #103 follow-on |
| Improve incrementally | Narrow change: point-query + status-bar label; no depth-model redesign |
| A change includes its consequences | Lifecycle managed by dynamic walk (no dangling pointer); label change avoids silent datum confusion |
| Capture decisions, not just implementations | Precedence rule (stores first), finest-tile-for-query, and CPU-buffer vs. GDAL-RasterIO choice noted here |
| Human control and transparency | Distinct "Elev: (ellipsoid)" label makes datum difference visible to the operator |
| Test what breaks | Unit test for `sampleAt()` covers the new query path; existing GGGS tile tests remain valid |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| camp ADR-0003 (Depth-layer tree) | Yes | GGGS stores go in a separate `getStoreElevation()` path, not `m_depthRasters`, preserving the semantic distinction between ellipsoidal height and chart-datum depth until the full depth-tree is fronted |
| camp ADR-0005 (Catalog browser / flat layers) | Yes | Dynamic walk of `topLevelLayers()` tracks the `GggsStoreSource` add/remove lifecycle without the AVP needing to own layer pointers |
| camp ADR-0007 (RasterFieldSource) | No | `sampleAt()` indexes the tile's already-resident CPU pixel buffer outside the render path; the `RasterFieldSource` render contract is unchanged (retaining `data_` past upload only affects resident memory, not the render interface) |

## Consequences

| If we change… | Also update… | Included in plan? |
|---|---|---|
| `GggsTile` public API | `test_gggs_tile.cpp` | Yes — step 6 |
| `GggsTileLayer` public API | Any code that includes `gggs_tile_layer.h` (scan for consumers) | Yes — check includes; no other callers expected |
| `GggsTile::texture()` stops freeing `data_` | Resident memory per painted tile ~doubles (CPU + GPU copy) | Yes — noted trade-off; revisit if memory pressure observed |
| Status-bar label in `projectview.cpp` | Any screenshot-based integration notes / docs mentioning the "Depth:" label | No — follow-up if formal docs exist |
| `getStoreElevation()` queries on mouse move | Cheap in-memory index (finding #4); no per-move I/O | Yes — resolved by resident buffer |

## Open Questions (resolved by the folded-in review findings)

- ~~The 1×1 GDAL RasterIO in `sampleAt()` opens the file per call…~~ **Resolved
  (finding #4):** `sampleAt()` no longer opens GDAL — it indexes the resident
  `data_` buffer (kept alive by not freeing it in `texture()`). Per-move cost is
  a cheap in-memory lookup; the network-mount latency concern no longer applies.
- **Decided:** both labels are shown when a store and a chart overlap ("Elev: X
  (ellipsoid)" and "Depth: Y"). Store takes precedence only in query *order*, not
  by suppressing the chart depth; distinct labels keep ellipsoidal height and
  chart-datum depth visibly separate until the datum service (#288).
- **Decided (finding #3):** a hidden (Layers-tab unchecked) store does NOT report
  elevation — `getStoreElevation()` filters on `isVisible()`, matching ADR-0003
  §2's enabled-layer contract for `getDepth`.

## Estimated Scope

Single PR.
