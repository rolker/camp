# CAMP ↔ camp2 Parity Audit (Phase 0 of #59)

Living, committed record for the camp2 map-system port
([#59](https://github.com/rolker/camp/issues/59)). This is **Phase 0**: it
gates every later replacement. Built by reading the actual source on both
sides; every feature claim cites `file:line`.

## The parity rule (load-bearing)

> **Never delete a camp original until its replacement is confirmed ≥ parity** —
> verified by test or manual check, *not* merely "it compiles."

The audit is **bidirectional**: we keep camp2's improvements (e.g. the marker
`CUBE` type, async raster load, grid auto-range) *and* preserve camp's
behavior (e.g. depth queries, `DELETEALL` across all namespaces, the grid
"speed" colormap) — porting each camp-only feature into the shared version
*before* retiring camp's.

Each per-pair matrix below classifies every feature:

| Delta tag | Meaning | Gate |
|---|---|---|
| `parity` | both sides behave the same | safe to retire camp's once switched |
| `camp2-only (keep)` | pure gain camp2 adds | no action; inherit it |
| `camp-only (port before retiring)` | camp does it, camp2 does not | **blocks retirement** until ported + verified |
| `differs (note)` | both do it, differently | review; may be improvement or regression |

## Bucket inventory (verified against `CMakeLists.txt` SOURCES)

### A. True replacements — camp2 has a counterpart (parity-gated)

| Pair | camp | camp2 | Matrix |
|---|---|---|---|
| background-raster | `backgroundraster.*`, `georeferenced.*`, `backgrounddetails.*` | `raster/raster_layer.*` + `map_view/web_mercator.*` | [raster.md](raster.md) |
| markers | `markers/markers*.{h,cpp}`, `markers_converter.h` | `ros/markers/{marker,markers,marker_namespace}.*` | [markers.md](markers.md) |
| grids | `grids/grid.*` (one class, both types) | `ros/grids/{grid_layer,occupancy_grid,grid_map}.*` (split) | [grids.md](grids.md) |

### B. camp-only carryover — no camp2 counterpart (re-home, preserve)

camp2's `ros/` set is only `geometry`, `grids`, `markers` (+ infra). Everything
else in camp has **no camp2 counterpart** and is re-homed onto
`camp::map::Layer`, preserving all logic and swapping only the coordinate
substrate (PR4/PR5):

- **Mission items** (editable): `waypoint`, `trackline`, `surveypattern`,
  `surveyarea`, `searchpattern`, `avoid_area`, `vector/{point,linestring,polygon}`,
  `group`, `behavior`
- **Live ROS overlays**: `ais/ais_contact`, `collision_monitor`,
  `platform_manager/platform` + `ship_track`, `nav_source`
- **Tools**: `measuringtool`, `orbit`
- **Glue**: `autonomousvehicleproject` (scene/mission model), `projectview`,
  `geographicsitem`/`geographicsmissionitem`, `mainwindow`

### C. camp2-only additions — pure gains (new capability)

`map_tiles/` (OSM slippy tiles + disk cache), `wmts/`, `map_tree_view/`
(multi-layer tree + drag/drop reorder), `background/background_manager`,
`ros/geometry/`, `tools/` (`layer_manager`, `map_tool`, `tools_manager`),
`util/cached_file_loader`, and the Web-Mercator scene itself.

## Status summary (Phase 0)

| Pair | Verdict | Retirement blockers (must port/verify before deleting camp's) |
|---|---|---|
| raster | **NOT yet ≥ parity** | depth band + `getDepth(geo)` query (camp-only, → PR3c); confirm scale-accessor consumers |
| markers | **NOT yet ≥ parity** | `DELETEALL` all-namespace fan-out (camp2 regression); DELETE object cleanup (camp2 leak); empty-frame + already-expired ingest drops; text scaling |
| grids | **NOT yet ≥ parity** | lazy-subscribe; confirm OccGrid centering; camp2 warn-throttle unit bug. (GridMap "speed" colormap **dropped** — see decisions) |
| OccupancyGrid colormap | **parity ✓** | none — byte-for-byte identical |

**Open decisions — RESOLVED 2026-06-02:**
- **GridMap colormap:** drop camp's situational "speed" ramp → adopt camp2
  grayscale auto-range. A reusable selectable-colormap facility
  (`camp::map::ColorMap`, also a depth-shading consumer) is tracked out of #59
  scope in [#63](https://github.com/rolker/camp/issues/63).
- **Marker fill-alpha:** make it a config option (not hardcoded); default
  chosen at PR5.

## Caveat

`*_manager.*` windows (Grid/AIS/Markers/CollisionMonitor) are being retired
(visibility moves to inline layer-tree checkboxes), so they were not deep-read
here beyond confirming they carry no unique behavior other than topic
discovery/instantiation — verify that assumption before deleting them (PR5).
