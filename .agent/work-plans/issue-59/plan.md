# Plan: Port camp2 multi-layer + OSM/WMTS map system into deployed CAMP

## Issue

https://github.com/rolker/camp/issues/59

## Context

Deployed CAMP (`CCOMAutonomousMissionPlanner`, built from `src/camp/`) is
single-background and chart-centric: one GDAL raster *is* the `QGraphicsScene`
(chart-native WKT projection), every overlay is parented to that `BackgroundRaster` and
positioned via `geoToPixel()`, and there is no layer model. `src/camp2/` (a **second
executable** in the same CMake project, `CMakeLists.txt:237+`) contains a self-contained
map subsystem CAMP lacks — a `Map` (`QAbstractItemModel`) + `Layer`/`LayerList` tree,
OSM/WMTS tiles with disk cache, and GDAL rasters reprojected into a global
**Web-Mercator** scene as ordinary layers.

The crux is the scene coordinate system: adopting the camp2 map system means adopting
Web Mercator as CAMP's scene projection, which touches every overlay's positioning
(~25 files call `geoToPixel`). It is tractable because overlays already store canonical
position as `QGeoCoordinate` and derive scene pixels on demand — the core change is
swapping `geoToPixel(geo, bgr)` for `web_mercator::geoToMap(geo)` and dropping the
parent-to-background rule.

## Framing: adopt the framework, preserve the functionality

This is **not** "replace camp with camp2." camp2's ROS layer set is only `geometry`,
`grids`, `markers` — it has no AIS, collision monitor, platform, nav_source, mission
planning, vector datasets, measuring tool, or orbit. So most of camp is re-homed onto
the new framework, not replaced. Three buckets:

| Bucket | Members | Handling |
|---|---|---|
| **True replacements** (camp2 has a counterpart) | background-raster, grids, markers | Parity-audit each pair; port any camp-only feature into the camp2/shared version **before** deleting camp's |
| **camp-only carryover** (no camp2 counterpart) | AIS, collision_monitor, platform, nav_source, mission items (waypoint/trackline/survey/search/avoid), vector datasets, measuring tool, orbit | Re-home onto `camp::map::Layer` — preserve all logic, swap only the coordinate substrate |
| **camp2-only additions** (pure gains) | OSM tiles, WMTS, multi-layer tree, drag/drop reorder, ros/geometry | New capability |

**Parity rule (load-bearing):** never delete a camp original until its replacement is
confirmed **≥ parity** — verified by test or manual check, not merely "it compiles." The
audit is **bidirectional**: keep camp2's improvements too (e.g. marker `CUBE`).

## Resolved Decisions (from discussion, 2026-06-01)

- **End-state: shared map library.** Extract camp2's map substrate into `libcamp_map`
  (namespaced `camp::map::`) that **both** `CCOMAutonomousMissionPlanner` and the `camp2`
  executable link. **camp2 is NOT retired** — it stays a working sandbox. No code
  duplication. Lib boundary **decided (2026-06-02): split** into `libcamp_map`
  (pure Qt/GDAL core) + `libcamp_map_ros` (ROS layers, PUBLIC-links the core) —
  see Open Questions for rationale.
- **Parity audit is committed** as a living doc (`docs/parity/` or `.agents/`) — a
  reviewable, durable matrix per replacement pair, not just inline analysis.
- **Testing where practicable.** Add unit tests for pure logic / data paths (camp has
  gtest in `test/`); use manual verification for Qt rendering / UI seams that aren't
  unit-testable. Don't force tests onto framework glue.
- **Depth preserved as a first-class layer type.** Multiple depth layers may coexist,
  toggled in the tree; `getDepthRaster()` (single) → `getDepth(geo)` query over the
  **enabled** depth layers in tree order, first valid wins. Keeps depth-aware A\*,
  survey/trackline generation, and the cursor depth readout. (Raster pair's parity gate.)
- **Settings: one-time reset** — no QSettings key migration.
- **Record decisions via a new ADR system in `camp`** (`docs/decisions/`, ADR-0001
  "adopt ADRs" + architecture ADR). Lands with PR3a.
- **No radar on this boat** — radar-via-grids stays general-purpose; ROS 1 RadarSector
  still deleted (PR1). Not a deployment gate.
- **Scale-dependent rendering: preserve behavior** — `bgr->mapScale()` → `MapView`
  viewport scale; distances stay geodesic. Implementation detail.

## Approach (stacked PRs on `feature/issue-59`)

1. **PR1 — dead-code sweep** (no behavior change): delete `radar/` (ROS 1, disabled),
   `geoviz/` (ROS 1, 0 CMake refs), `sonar_manager/` (ROS 1, `.cpp` not built),
   `sound_play/` (commented out), `scaledview.{h,cpp}` (unused); clean `CMakeLists.txt`.
2. **Phase 0 — parity audit** (doc-only, gates later replacements): for each
   true-replacement pair (raster, grids, markers) commit a matrix
   `camp feature | camp2 feature | delta | resolution`; confirm the carryover inventory.
   Known deltas to capture: raster→**depth band** (camp-only) + color tables (parity ✓);
   markers→**DELETE/DELETEALL** actions (camp) vs **CUBE** + expiry timer (camp2); grids→
   one-class-both-types (camp) vs split classes (camp2), colormap/range parity.
3. **PR2 — extract `libcamp_map`**: move camp2's map substrate (+ shared ROS layer
   framework) into a shared library; **repoint the `camp2` executable to link it** and
   confirm camp2 still builds and runs unchanged (self-verifying). No camp changes yet.
4. **PR3a — camp adopts the lib**: web-mercator scene; `Map` owns the scene;
   `AutonomousVehicleProject` shrinks toward a mission model contributing a mission
   layer; charts shown via reprojected `RasterLayer`; add OSM tile layer; add a
   `geoToPixel`→`web_mercator::geoToMap` **shim** in `GeoGraphicsItem` so not-yet-migrated
   overlays still compile/run. Bootstrap `docs/decisions/` + write the architecture ADR.
   *Nothing deleted yet.*
5. **PR3b — tabbed Layers/Mission UI**: split into a tabbed dock (Layers = `MapTreeView`;
   Mission = existing `treeView`); background stops being a mission-tree node.
6. **PR3c — depth-layer type**: depth-retaining raster layer + order-resolved
   `getDepth(geo)` query → satisfies the raster pair's parity gate.
7. **PR4 — mission-item migration**: port editable overlays (`Waypoint`, `TrackLine`,
   `SurveyPattern`, `SurveyArea`, `AvoidArea`, `SearchPattern`, vector
   `Point`/`LineString`/`Polygon`) off the shim to `web_mercator::geoToMap`; repoint
   depth-aware survey/trackline + A\* at the `getDepth(geo)` query. Note: A\* iterates a
   pixel grid today — it needs a defined planning grid/resolution under the multi-depth
   model (more than a query repoint; may be its own sub-PR).
8. **PR5 — overlay migration + parity-gated replacement + manager retirement**: re-home
   camp-only overlays (`NavSource`, `Markers`-carryover bits, `Platform`/`ShipTrack`,
   `CollisionMonitor`, `AISContact`) onto the framework. For the **replacement** pairs
   (grids, markers): confirm ≥ parity first (port `DELETE`/`DELETEALL` into shared markers
   if expiry doesn't cover it; verify grid colormap/range), then switch camp to the shared
   layers and retire camp's `grids/grid.*` + markers. Delete the four overlay manager
   windows (`GridManager`, `AISManager`, `MarkersManager`, `CollisionMonitorManager`) +
   menu actions — visibility moves to inline layer-tree checkboxes.
9. **PR6 — parity-confirmed retirement + cleanup**: with depth preserved, delete
   `backgroundraster.*`, `backgrounddetails.*`, `georeferenced.*`; remove the
   `geoToPixel` shim + `updateBackground`/`updateProjectedPoints` plumbing; add the
   missing `.agents/README.md` (architecture section).

Each PR builds and runs on its own; PR1, Phase 0, and PR2 are low-risk and land first.

## Testing (where practicable)

| Test | Where | PR |
|---|---|---|
| `web_mercator` geo↔map round-trip + `metersPerUnit` at latitude | gtest (pure fn) | PR2 |
| depth-layer `getDepth(geo)` order resolution (overlap → top enabled wins) | gtest (logic) | PR3c |
| marker action coverage incl. `DELETE`/`DELETEALL` parity | gtest if converter is separable; else manual | PR5 |
| grid colormap/value-range parity | manual vs deployed bag replay | PR5 |
| scene/UI wiring, tile fetch, depth cursor readout | manual (Qt rendering, not unit-testable) | PR3a/PR3c |

## Files to Change (representative; full tables in the issue)

| File | Change |
|------|--------|
| `CMakeLists.txt` | Remove retired dirs (PR1); add `libcamp_map` + relink camp2 (PR2); link camp (PR3a); drop manager UIs (PR5) |
| `src/camp/{radar,geoviz,sonar_manager,sound_play}/`, `scaledview.*` | Delete (PR1) |
| `docs/parity/*.md` | Committed parity matrices (Phase 0) |
| `src/camp2/{map,map_view,map_tiles,wmts,map_tree_view,background,raster,util}/` (+ shared `ros/`) | Extract into `libcamp_map` (PR2) |
| `docs/decisions/0001-*.md`, `docs/decisions/000X-web-mercator-scene-and-layer-model.md` | New ADR system + architecture ADR (PR3a) |
| new depth-raster layer + depth query | Depth-as-layer, `getDepth(geo)` resolution (PR3c) |
| `autonomousvehicleproject.{h,cpp}` | Drop scene/background ownership; mission-model only; `getDepthRaster()` → depth query (PR3a/PR3c) |
| `mainwindow.{ui,cpp}` | Tabbed Layers/Mission dock; wire `Map`; remove manager `show()` actions (PR3b/PR5) |
| `geographicsitem.{h,cpp}`, `projectview.{h,cpp}` | `geoToPixel` shim → `geoToMap`; cursor depth via query (PR3a/PR4) |
| `astar.cpp`, `surveyarea.cpp`, `trackline.cpp` | Depth-aware planning/survey → `getDepth(geo)`; A\* planning grid (PR4) |
| overlay classes (`waypoint`, `survey*`, `vector/*`, `ais/*`, `markers/*`, `platform_manager/*`, `collision_monitor/*`, `nav_source`, `grids/grid`) | Migrate to `geoToMap`; re-home onto Layer framework; parity-gated retirement of replaced ones (PR4/PR5) |
| `backgroundraster.*`, `backgrounddetails.*`, `georeferenced.*`, `grids/grid.*` | Delete once replacements confirmed ≥ parity (PR5/PR6) |
| `.agents/README.md`, `test/` | Create README (PR6); add unit tests per the Testing table |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Improve incrementally | Foundational change staged into small, individually-buildable, parity-gated PRs; PR2 is self-verifying via the unchanged camp2 binary |
| A change includes its consequences | Parity rule prevents silent feature loss; depth/markers/grids deltas captured before deletion; tests added where practicable |
| Only what's needed | Shared lib avoids duplication; reuse over rewrite; one-time settings reset |
| Capture decisions, not just implementations | New camp ADR + committed parity doc record the why durably |
| Test what breaks | Unit tests on pure logic (web_mercator, depth resolution, marker actions); manual verification reserved for Qt/UI glue |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 worktree isolation | Yes | Work on `feature/issue-59` worktree; PRs to `jazzy` |
| 0008 ROS 2 conventions | Yes | Removes ROS 1 holdouts; migrated overlays stay rclcpp/tf2 |
| 0013 progress.md vocabulary | Yes | `## Plan Authored` / `## Plan Review` entries recorded |
| new camp ADR | Yes (PR3a) | Bootstrap `docs/decisions/` + architecture ADR (Web-Mercator scene, two-model split, depth-as-layer, shared lib) |

## Consequences

| If we change... | Also update... | Included? |
|---|---|---|
| Scene projection → Web Mercator | every overlay's geo→scene conversion (via shim, then migrated) | Yes (PR3a–PR5) |
| Extract `libcamp_map` | camp2 executable relinks it; camp links it | Yes (PR2/PR3a) |
| Replace camp grids/markers | parity port (DELETEALL, colormap) before deletion | Yes (Phase 0 + PR5) |
| `getDepthRaster()` → `getDepth(geo)` | `astar.cpp`, `surveyarea.cpp`, `trackline.cpp`, `projectview.cpp`, menu gating | Yes (PR3c/PR4) |
| Remove manager windows | `mainwindow` menu + action wiring | Yes (PR5) |
| Mission layer in shared scene | Hypack/mission export reads mission model, not scene | Verify in PR4 |

## Open Questions

- [x] Lib boundary: **DECIDED 2026-06-02 — split.** Two libraries: `libcamp_map`
  (pure Qt/GDAL map core — `map`, `map_view`, `map_tiles`, `wmts`,
  `map_tree_view`, `background`, `raster`, `util`) and `libcamp_map_ros`
  (ROS-dependent layers — `ros/`), where `libcamp_map_ros` PUBLIC-links
  `libcamp_map`. Verified the boundary is real and one-directional: the core has
  **0** ROS includes; `ros/` depends on the core, not vice versa. Rationale:
  enforces the layering at compile time, lets the pure-logic gtests
  (`web_mercator` round-trip, depth-order) link Qt/GDAL only (no rclcpp/tf2),
  and keeps the core reusable in a non-ROS context. Cost is modest extra CMake
  (two targets/installs/export sets).

## Estimated Scope

Multiple PRs (PR1, Phase 0, PR2, PR3a/b/c, PR4, PR5, PR6 — ~9 units), stacked on
`feature/issue-59`. PR1/Phase 0/PR2 are low-risk; PR3a is the foundational scene swap;
PR3b–PR6 are incremental, parity-gated migration and cleanup.
