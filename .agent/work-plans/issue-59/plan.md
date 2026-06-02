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

## Resolved Decisions (from issue discussion, 2026-06-01)

- **Depth = a first-class layer type, preserved.** Don't drop depth. A depth raster
  (GDAL `Float32` band) becomes a layer like any other; **multiple depth layers may
  coexist**, each toggled in the tree. The single `getDepthRaster()` accessor is
  replaced by a `getDepth(geo)` query that walks the **enabled** depth layers in tree
  order and returns the first that has valid data at that point (layer order resolves
  overlap). Depth-aware A\*, survey/trackline generation, and the cursor depth readout
  all keep working against this query.
- **Settings: one-time reset.** No QSettings key migration; first launch after upgrade
  comes up at defaults, layers/center re-established via the new tree.
- **Record decisions via a new ADR system in `camp`.** Bootstrap `docs/decisions/` in
  the camp repo (ADR-0001 "adopt ADRs", mirroring the workspace) and write an ADR for
  the Web-Mercator scene + two-model split + depth-as-layer. Lands with PR3.
- **No radar on this boat.** Radar-via-grids stays general-purpose (radar-equipped
  platforms); it is not a deployment gate here. The ROS 1 `RadarSector`/OpenGL path is
  still deleted in PR1.
- **Scale-dependent rendering: preserve behavior.** Port `bgr->mapScale()` users
  (measuring tool, orbit, marker glyphs) to the `MapView` viewport scale with no visible
  change; distances stay geodesic (`QGeoCoordinate::distanceTo`) and projection-correct.
  Implementation detail, no behavior change.

## Approach (stacked PRs on `feature/issue-59`)

1. **PR1 — dead-code sweep** (no behavior change): delete `radar/` (ROS 1, disabled),
   `geoviz/` (ROS 1, 0 CMake refs), `sonar_manager/` (ROS 1, `.cpp` not built),
   `sound_play/` (commented out), `scaledview.{h,cpp}` (unused); clean `CMakeLists.txt`.
2. **PR2 — import map substrate**: bring `camp2/{map, map_view, map_tiles, wmts,
   map_tree_view, background, raster, util}` in as new code; build green; not yet wired
   to the live UI.
3. **PR3 — scene swap + chart display + UI split + depth layers + ADR**: `Map` owns the
   Web-Mercator scene; `AutonomousVehicleProject` shrinks to a mission model contributing
   a mission layer; existing charts shown via reprojected `RasterLayer`; add OSM tile
   layer; **depth-raster layer type** with order-based `getDepth(geo)` resolution; split
   the UI into a **tabbed dock** (Layers tab = `MapTreeView`; Mission tab = existing
   `treeView`). Bootstrap `docs/decisions/` + write the architecture ADR. Retire
   `backgroundraster.*`, `backgrounddetails.*`, `georeferenced.*`.
4. **PR4 — mission-item migration**: port editable overlays (`Waypoint`, `TrackLine`,
   `SurveyPattern`, `SurveyArea`, `AvoidArea`, `SearchPattern`, vector
   `Point`/`LineString`/`Polygon`) from `geoToPixel` to `web_mercator::geoToMap`;
   repoint depth-aware survey/trackline generation + A\* at the `getDepth(geo)` query.
5. **PR5 — ROS-overlay migration + manager-window retirement**: port `Grid`,
   `NavSource`, `Markers`, `Platform`/`ShipTrack`, `CollisionMonitor`, `AISContact` to
   layers (radar, where present, arrives via the grids path); delete `grids/grid.*` and
   the four overlay manager windows (`GridManager`, `AISManager`, `MarkersManager`,
   `CollisionMonitorManager`) + their menu actions — visibility moves to inline
   layer-tree checkboxes.
6. **PR6 — cleanup**: remove `updateBackground`/`updateProjectedPoints` plumbing, final
   dead-code pass, add the missing `.agents/README.md` (with the architecture section).

Each PR builds and runs on its own; PR1–PR2 are low risk and land first.

## Files to Change (representative; full tables in the issue)

| File | Change |
|------|--------|
| `CMakeLists.txt` | Remove retired dirs (PR1); add camp2 map modules (PR2); drop manager UIs (PR5) |
| `src/camp/{radar,geoviz,sonar_manager,sound_play}/`, `scaledview.*` | Delete (PR1) |
| `src/camp2/{map,map_view,map_tiles,wmts,map_tree_view,background,raster,util}/` | Import as `src/camp/...` (PR2) |
| `docs/decisions/0001-*.md`, `docs/decisions/000X-web-mercator-scene-and-layer-model.md` | New ADR system + architecture ADR (PR3) |
| new depth-raster layer (under imported `raster/`) + `Map`/depth query | Depth-as-layer, `getDepth(geo)` resolution (PR3) |
| `autonomousvehicleproject.{h,cpp}` | Drop scene/background ownership; mission-model only; `getDepthRaster()` → depth query (PR3) |
| `mainwindow.{ui,cpp}` | Tabbed Layers/Mission dock; wire `Map`; remove manager `show()` actions (PR3/PR5) |
| `projectview.{h,cpp}`, `geographicsitem.{h,cpp}` | Web-Mercator scene + `geoToMap`; cursor depth via query (PR3/PR4) |
| `astar.cpp`, `surveyarea.cpp`, `trackline.cpp` | Depth-aware planning/survey → `getDepth(geo)` query (PR4) |
| overlay classes (`waypoint`, `survey*`, `vector/*`, `grids/grid`, `ais/*`, `markers/*`, `platform_manager/*`, `collision_monitor/*`, `nav_source`) | Migrate to `geoToMap`; reparent into layer tree (PR4/PR5) |
| `backgroundraster.*`, `backgrounddetails.*`, `georeferenced.*`, `grids/grid.*` | Delete once replacements live (PR3/PR5) |
| `.agents/README.md` | Create with architecture section (PR6) |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Improve incrementally | Foundational change staged into 6 self-contained, individually-buildable PRs; PR1 (sweep) and PR2 (import) carry no behavior change |
| A change includes its consequences | Each overlay migration carries its retirements + the `updateBackground` plumbing removal; depth capability explicitly preserved, not silently dropped |
| Only what's needed | Reuses the camp2 subsystem rather than writing new map code; one-time settings reset instead of migration machinery; deletes more than it adds |
| Capture decisions, not just implementations | New camp ADR system records the scene/two-model/depth decisions durably (PR3) |
| Test what breaks | Coordinate round-trip (`geoToMap`/`mapToGeo` vs old `geoToPixel`), chart reprojection, and the multi-layer depth-resolution query are the regression-prone seams to cover |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 worktree isolation | Yes | Work proceeds on `feature/issue-59` worktree; PRs to `jazzy` |
| 0008 ROS 2 conventions | Yes | Removes ROS 1 holdouts (radar/geoviz/sonar_manager); migrated overlays stay rclcpp/tf2 |
| 0013 progress.md vocabulary | Yes | `## Plan Authored` entry recorded; later entries per PR |
| new camp ADR | Yes (PR3) | Bootstrap `docs/decisions/` in camp + architecture ADR for the Web-Mercator scene, two-model split, depth-as-layer |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Scene projection → Web Mercator | every overlay's geo→scene conversion | Yes (PR3–PR5) |
| Remove `BackgroundRaster` | depth consumers (cursor readout, A\*, survey/trackline gen) → depth-layer query | Yes (PR3/PR4) |
| `getDepthRaster()` (single) → `getDepth(geo)` (layer-ordered) | `astar.cpp`, `surveyarea.cpp`, `trackline.cpp`, `projectview.cpp`, menu gating | Yes (PR3/PR4) |
| Remove manager windows | `mainwindow` menu + action wiring | Yes (PR5) |
| Mission layer in shared scene | Hypack/mission export reads mission model, not scene | Yes — verify in PR4 |

## Open Questions

- [ ] No open questions — all resolved above; plan is review-plan-ready.

## Estimated Scope

Multiple PRs (6), stacked on `feature/issue-59`. PR1–PR2 are low-risk and mergeable
immediately; PR3 is the high-risk foundational swap (scene + UI + depth + ADR); PR4–PR6
are incremental migration and cleanup.
