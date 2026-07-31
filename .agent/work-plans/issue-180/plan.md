# Plan: Depth-at-cursor for GGGS store layers

## Issue

https://github.com/rolker/camp/issues/180

## Context

`AutonomousVehicleProject::getDepth()` walks `m_depthRasters` — a list of
`DepthRaster` objects, one per loaded background chart that carries a Float32
depth band. GGGS store layers (`GggsTileLayer`, spawned by `GggsStoreSource`)
never register in this walk, so hovering over an open bathy store silently
omits depth from the status bar.

`GggsTile` already loads pixel data off-thread (via `loadPixels()`) and holds
a `geo_transform_[6]` + extent bounds from construction. After `texture()` is
called (first paint), the CPU buffer is freed — so a point query must re-read
a 1×1 region from the GDAL source, not the in-memory buffer.

GGGS store values are ellipsoidal up-positive heights, not chart-datum depths;
the two need distinct labels in the status bar.

## Approach

1. **`GggsTile::sampleAt(lon, lat)`** — invert the geotransform (north-up
   `geo_transform_`, no rotation) to get pixel (col, row), reject if
   out-of-bounds, then do a 1×1 `GDALRasterBand::RasterIO` read on `band_`
   from `path_`. Apply the tile's NoData mask and return `std::nanf` on
   no-data / not-valid. Safe to call on any thread.

2. **`GggsTileLayer::getElevation(QGeoCoordinate)`** — walk `tiles_`, filter
   to those whose lat/lon extent contains the point, sort by descending tile
   level (parsed from the basename `<level>_<row>_<col>.tif`), call
   `sampleAt()` on each until a non-NaN result. Returns NaN if nothing covers
   the point. The finest-level tile is queried regardless of the rendered LOD
   (inspection > display — issue note, #103 follow-on).

3. **`AutonomousVehicleProject::getStoreElevation(QGeoCoordinate)`** — walk
   the Map's `topLevelLayers()` child list dynamically on each call, cast to
   `GggsTileLayer*`, call `getElevation()`, return first non-NaN. No separate
   list: dynamic walk tracks layer add/remove automatically and avoids a
   dangling-pointer risk on layer teardown.

4. **`ProjectView::mouseMoveEvent()`** — try `getStoreElevation()` first
   (stores have precedence; issue precedence rule); if non-NaN, append
   `" Elev: X (ellipsoid)"`. Then try `getDepth()` (chart rasters); if
   non-NaN, append `" Depth: X"`. Both can appear when a store and a chart
   overlap. The two-label design makes the datum difference explicit until
   the datum service (#288) arrives.

5. **Unit test** in `test_gggs_tile.cpp` — write a 1-band Float32 north-up
   GeoTIFF with known data using the existing GDAL test-fixture helper,
   construct a `GggsTile`, call `sampleAt()` at in-bound and out-of-bound
   coordinates, and at a NoData pixel; assert expected values.

## Files to Change

| File | Change |
|------|--------|
| `src/camp_map/raster/gggs_tile.h` | Declare `float sampleAt(double lon, double lat) const` |
| `src/camp_map/raster/gggs_tile.cpp` | Implement `sampleAt()`: geotransform inversion + 1×1 GDAL RasterIO |
| `src/camp_map/raster/gggs_tile_layer.h` | Declare `float getElevation(const QGeoCoordinate& location) const` |
| `src/camp_map/raster/gggs_tile_layer.cpp` | Implement `getElevation()`: extent filter, level sort, `sampleAt()` loop |
| `src/camp/autonomousvehicleproject.h` | Declare `float getStoreElevation(const QGeoCoordinate&) const` |
| `src/camp/autonomousvehicleproject.cpp` | Implement `getStoreElevation()`: dynamic LayerList walk |
| `src/camp/projectview.cpp` | Update `mouseMoveEvent()` to call both query methods with appropriate labels |
| `test/test_gggs_tile.cpp` | Add `sampleAt()` unit tests |

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
| camp ADR-0007 (RasterFieldSource) | No | `sampleAt()` reads raw GDAL pixels outside the render path; the `RasterFieldSource` render contract is unchanged |

## Consequences

| If we change… | Also update… | Included in plan? |
|---|---|---|
| `GggsTile` public API | `test_gggs_tile.cpp` | Yes — step 5 |
| `GggsTileLayer` public API | Any code that includes `gggs_tile_layer.h` (scan for consumers) | Yes — check includes; no other callers expected |
| Status-bar label in `projectview.cpp` | Any screenshot-based integration notes / docs mentioning the "Depth:" label | No — follow-up if formal docs exist |
| `getStoreElevation()` queries on mouse move | Performance acceptable given single-point 1×1 GDAL read; profile if operator reports latency | No — noted as acceptable, revisit if needed |

## Open Questions

- The 1×1 GDAL RasterIO in `sampleAt()` opens and closes the file on each call;
  NVMe-backed stores are fast but network-mounted stores could lag. Should a
  debounce (query only when the cursor stops) or a read-ahead cache be added
  now, or treated as a follow-on if latency is observed in the field?
- When a chart raster and a GGGS store both cover the cursor, the plan shows
  both labels ("Elev: X (ellipsoid)" and "Depth: Y"). Is that the preferred
  operator UX, or should only one value be shown (with stores taking precedence
  and suppressing the chart depth)?

## Estimated Scope

Single PR.
