# Plan: Port camp2 multi-layer + OSM/WMTS map system into deployed CAMP

## Issue

https://github.com/rolker/camp/issues/59

## Context

Deployed CAMP (`src/camp/`) is single-background and chart-centric: one GDAL raster
*is* the `QGraphicsScene` (chart-native WKT projection), every overlay is parented to
that `BackgroundRaster` and positioned via `geoToPixel()`, and there is no layer model.
`src/camp2/` contains a self-contained map subsystem CAMP lacks — a `Map`
(`QAbstractItemModel`) + `Layer`/`LayerList` tree, OSM/WMTS tiles with disk cache, and
GDAL rasters reprojected into a global **Web-Mercator** scene as ordinary layers.

The crux is the scene coordinate system: adopting the camp2 map system means adopting
Web Mercator as CAMP's scene projection, which touches every overlay's positioning
(~25 files call `geoToPixel`). It is tractable because overlays already store canonical
position as `QGeoCoordinate` and derive scene pixels on demand — the core change is
swapping `geoToPixel(geo, bgr)` for `web_mercator::geoToMap(geo)` and dropping the
parent-to-background rule.

## Approach (stacked PRs on `feature/issue-59`)

1. **PR1 — dead-code sweep** (no behavior change): delete `radar/` (ROS 1, disabled),
   `geoviz/` (ROS 1, 0 CMake refs), `sonar_manager/` (ROS 1, `.cpp` not built),
   `sound_play/` (commented out), `scaledview.{h,cpp}` (unused); clean `CMakeLists.txt`.
2. **PR2 — import map substrate**: bring `camp2/{map, map_view, map_tiles, wmts,
   map_tree_view, background, raster, util}` in as new code; build green; not yet wired
   to the live UI.
3. **PR3 — scene swap + chart display + UI split**: `Map` owns the Web-Mercator scene;
   `AutonomousVehicleProject` shrinks to a mission model contributing a mission layer;
   existing charts shown via reprojected `RasterLayer`; add OSM tile layer; split the
   UI into a **tabbed dock** (Layers tab = `MapTreeView`; Mission tab = existing
   `treeView`). Retire `backgroundraster.*`, `backgrounddetails.*`, `georeferenced.*`.
4. **PR4 — mission-item migration**: port editable overlays (`Waypoint`, `TrackLine`,
   `SurveyPattern`, `SurveyArea`, `AvoidArea`, `SearchPattern`, vector
   `Point`/`LineString`/`Polygon`) from `geoToPixel` to `web_mercator::geoToMap`.
5. **PR5 — ROS-overlay migration + manager-window retirement**: port `Grid`,
   `NavSource`, `Markers`, `Platform`/`ShipTrack`, `CollisionMonitor`, `AISContact` to
   layers; **radar arrives here** as `OccupancyGrid`/`grid_map` via the grids path;
   delete `grids/grid.*` and the four overlay manager windows (`GridManager`,
   `AISManager`, `MarkersManager`, `CollisionMonitorManager`) + their menu actions —
   visibility moves to inline layer-tree checkboxes.
6. **PR6 — cleanup**: remove `updateBackground`/`updateProjectedPoints` plumbing, final
   dead-code pass, docs (incl. a `.agents/README.md`, currently missing).

Each PR builds and runs on its own; PR1–PR2 are low risk and land first.

## Files to Change (representative; full tables in the issue)

| File | Change |
|------|--------|
| `CMakeLists.txt` | Remove retired dirs (PR1); add camp2 map modules (PR2); drop manager UIs (PR5) |
| `src/camp/{radar,geoviz,sonar_manager,sound_play}/`, `scaledview.*` | Delete (PR1) |
| `src/camp2/{map,map_view,map_tiles,wmts,map_tree_view,background,raster,util}/` | Import as `src/camp/...` (PR2) |
| `autonomousvehicleproject.{h,cpp}` | Drop scene/background ownership; mission-model only (PR3) |
| `mainwindow.{ui,cpp}` | Tabbed Layers/Mission dock; wire `Map`; remove manager `show()` actions (PR3/PR5) |
| `projectview.{h,cpp}`, `geographicsitem.{h,cpp}` | Use Web-Mercator scene + `geoToMap` (PR3/PR4) |
| overlay classes (`waypoint`, `trackline`, `survey*`, `vector/*`, `grids/grid`, `ais/*`, `markers/*`, `platform_manager/*`, `collision_monitor/*`, `nav_source`) | Migrate to `geoToMap`; reparent into layer tree (PR4/PR5) |
| `backgroundraster.*`, `backgrounddetails.*`, `georeferenced.*`, `grids/grid.*` | Delete once replacements live (PR3/PR5) |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Improve incrementally | Foundational change is staged into 6 self-contained, individually-buildable PRs; PR1 (sweep) and PR2 (import) carry no behavior change |
| A change includes its consequences | Each overlay migration carries its retirements + the `updateBackground` plumbing removal; settings-key migration tracked as an open question, not deferred silently |
| Only what's needed | Reuses the existing camp2 subsystem rather than writing new map code; deletes more than it adds |
| Capture decisions, not just implementations | UI/scene decisions recorded in the issue; an ADR for the Web-Mercator scene + two-model split is a candidate (open question) |
| Test what breaks | Coordinate round-trip (`geoToMap`/`mapToGeo` vs old `geoToPixel`) and chart reprojection are the regression-prone seams to cover |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 worktree isolation | Yes | Work proceeds on `feature/issue-59` worktree; PRs to `jazzy` |
| 0008 ROS 2 conventions | Yes | Removes ROS 1 holdouts (radar/geoviz/sonar_manager); migrated overlays stay rclcpp/tf2 |
| 0013 progress.md vocabulary | Yes | `## Plan Authored` entry recorded; later entries per PR |
| new ADR? | Maybe | Web-Mercator scene + Map/mission two-model split may warrant an ADR in `camp` (open question) |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Scene projection → Web Mercator | every overlay's geo→scene conversion | Yes (PR3–PR5) |
| Remove `BackgroundRaster` | `getDepth()` consumers (measuring/sounding readout) | Open question — confirm depth-probe path |
| Remove manager windows | `mainwindow` menu + action wiring | Yes (PR5) |
| QSettings keys (per-background → per-`itemID()`) | view/layer persistence on upgrade | Open question |
| Mission layer in shared scene | Hypack/mission export (`exportHypack`, `exportMissionPlan`) still reads mission model | Yes — export reads model, not scene; verify in PR4 |

## Open Questions

- Confirm radar/costmap currently feeds `grids/grid.cpp` as `OccupancyGrid` vs `grid_map` (verify against the deployed launch before PR5).
- Where does `BackgroundRaster::getDepth()` get consumed (measuring tool / sounding readout)? Web-Mercator tiles carry no depth band — need a replacement source or retain a depth raster.
- QSettings migration: camp2 stores per-`itemID()`, camp per-background — accept a one-time reset or migrate keys?
- Worth an ADR in `camp` for the Web-Mercator scene + two-model (layers vs mission) split?
- `MeasuringTool`/`Orbit` rendering at scale under a global projection (currently scale via `bgr->mapScale()`).

## Estimated Scope

Multiple PRs (6), stacked on `feature/issue-59`. PR1–PR2 are low-risk and mergeable
immediately; PR3 is the high-risk foundational swap; PR4–PR6 are incremental migration
and cleanup.
