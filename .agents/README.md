# Agent Guide: camp

CAMP — the **C**COM **A**utonomous **M**ission **P**lanner: a Qt5 + ROS 2 (rclcpp)
desktop GUI for planning and monitoring autonomous marine-vehicle missions.

## Workflow

Standard worktree/PR workflow (this is a GitHub-origin repo). The default branch
is **`jazzy`**, not `main`. To check field vs dev mode:

```bash
# From the workspace root, pass the path to this repo
.agent/scripts/field_mode.sh --describe layers/main/ui_ws/src/camp
```

There is **no CI and no pre-commit config** in this repo. The build + the gtest
suite are the only automated gates — run them locally before pushing.

## Package Inventory

One ROS 2 / ament_cmake package (`camp`) that builds **two executables** and
**two shared libraries** from `CMakeLists.txt`:

| Target | Kind | Source | Role |
|--------|------|--------|------|
| `CCOMAutonomousMissionPlanner` | executable | `src/camp/` | The deployed product (mission planning + monitoring) |
| `camp2` | executable | `src/camp2/main/` | Sandbox app — exercises the framework in isolation; also the only place `QAbstractItemModelTester` validates the Map model at runtime |
| `camp_map` | shared lib | `src/camp2/{map,map_view,raster,map_tiles,wmts,tools,background,util}/` | Web-Mercator scene + layer-tree framework (ROS-free) |
| `camp_map_ros` | shared lib | `src/camp2/ros/` | ROS overlay framework (topic discovery, grids, markers, geometry) built on `camp_map` |

Both executables link the shared libs. The strategy (ADR-0002) is that the
deployed `camp` **adopts** `camp2`'s framework rather than one app replacing the
other; `camp2`-the-app is increasingly a dev/test harness as `camp` absorbs the
framework.

## Repository Layout

```
src/camp/        deployed app (CCOMAutonomousMissionPlanner): mission model,
                 overlays, ROS link, details/manager widgets
src/camp2/       the framework (camp_map / camp_map_ros) + the camp2 sandbox app
docs/decisions/  ADRs — read these before touching the scene/layer/depth model
test/            gtest suites (run via colcon test)
workspace/       sample data, incl. the 13283 KAP test charts
.agents/         this guide
```

## Architecture Overview

**Two models, one scene (ADR-0002):**

- **Scene coordinates are global Web-Mercator (EPSG:3857)**, owned by
  `camp::map::Map`, independent of any loaded chart. Overlays position via
  `web_mercator::geoToMap()` through the single `GeoGraphicsItem::geoToPixel(point)`
  shim (the old per-chart-pixel-space scene is gone).
- **Mission model** — `AutonomousVehicleProject` (a `QAbstractItemModel`) holds
  the editable mission tree (waypoints, tracklines, survey/search patterns,
  areas). Shown in the Mission tab.
- **Layer model** — `camp::map::Map` (a second `QAbstractItemModel`) holds the
  display layer tree (charts, OSM/WMTS tiles, ROS grids/markers, AIS, collision
  zones). Shown in the Layers tab with per-layer visibility checkboxes.

**Charts are layers, not a foundation (ADR-0003):** a loaded chart is a stacked,
Map-owned `camp::raster::RasterLayer` (GDAL-warped to EPSG:3857) plus a
`DepthRaster` depth provider. They persist as **app state in QSettings**
(`backgrounds/files`), independent of any mission file — not as mission-tree
nodes. (`BackgroundRaster` was the old single-foundation chart object; it has
been retired.)

**Overlays** (mission items, AIS contacts, collision zones, platform/ship-track,
nav_source) parent to the Map's persistent scene-origin anchor (`Map::rootItem()`,
via `AutonomousVehicleProject::originAnchor()`) or to a dedicated Map `Layer`, so
they render with or without a chart loaded (OSM/WMTS-only operation).

**ROS link:** `ROSLink` spins an rclcpp node on a `QThread` (`camp_ros::NodeThread`)
and re-emits node lifecycle as Qt signals. Manager objects (AIS, collision,
platform) derive from `camp_ros::ROSWidget`/`ROSObject` (= `ROSClient<QWidget|QObject>`)
to get `node_`/`transform_buffer_` + an `onNodeUpdated()` hook.

## Key Files to Read First

- `docs/decisions/0002-*.md`, `docs/decisions/0003-*.md` — the scene/layer/depth
  architecture. **Read before changing anything in the map system.**
- `src/camp/autonomousvehicleproject.{h,cpp}` — the mission model + chart
  load/persistence/depth.
- `src/camp2/map/map.{h,cpp}` + `map/layer.{h,cpp}` — the layer model.
- `src/camp/geographicsitem.{h,cpp}` — `geoToPixel` / `metresPerPixel` (the
  scene shim every overlay uses).
- `src/camp/roslink.{cpp,h}` + `src/camp/ros/ros_client.h` — ROS node lifecycle.

## Build & Test

```bash
# From the layer workspace directory (e.g., layers/main/ui_ws/)
colcon build --symlink-install --packages-select camp
# Testing requires setup.bash sourced in the same shell
colcon test --packages-select camp && colcon test-result --verbose
```

The gtest suites live in `test/` (incl. `test_map_model` — `QAbstractItemModelTester`
coverage of the Map model's insert/remove/reorder paths).

## Cross-Layer Dependencies

- Depends on `marine_interfaces`, `marine_ais_msgs`, `nav2_msgs`, `grid_map_msgs`
  for the overlay subscriptions.
- `camp_map` is ROS-free by design — the ROS node is attached to the Map's
  `ToolsManager` by the app layer, not the map core. Keep it that way.

## Common Pitfalls

- **QSettings store name:** the deployed app must call
  `setOrganizationName("UNH-CCOMJHC")` / `setApplicationName("CCOMAutonomousMissionPlanner")`
  in `main.cpp` (matching the camp2 sandbox) — otherwise it persists to
  "Unknown Organization" and splits state from camp2. The **camp2 sandbox's
  `main_window.cpp` is the reference for app-shell wiring** (org name,
  quit-on-ROS-shutdown).
- **Shutdown ordering:** `executor.spin()` only returns once `rclcpp::shutdown()`
  is called. Anything that `quit()`/`wait()`s the spin `QThread` (e.g.
  `~ROSLink`) must call `rclcpp::shutdown()` first or it deadlocks on
  window-close. Wire `NodeThread::shuttingDown → QApplication::quit` so Ctrl-C
  exits cleanly too.
- **Double-parented overlays:** items like `AISContact` carry both a QObject
  parent (their manager) and a QGraphicsItem parent (a Map `Layer`). This is the
  standard Qt pattern — each base destructor deregisters from the other — but be
  aware their manager's `std::map<…,*>` can hold dangling pointers after the
  graphics tree is torn down; don't dereference those maps in manager destructors.
- **Non-removable layers:** system overlay containers whose owner keeps a raw
  pointer (AIS, Collision Monitor) call `Layer::setRemovable(false)` so the
  Layers-tab Remove action can't delete them out from under the owner.
- **No back-compat for old project files:** ADR-0003 dropped the old on-disk
  `BackgroundRaster` chart format; legacy `.json` chart nodes are ignored on load.
- **No CI / no pre-commit** — local build + gtest are the gate.

## Instructions for Use

Read this guide and the relevant ADRs before modifying the scene, layer, depth,
or ROS-link subsystems. Verify any documentation claim against the source before
relying on it.
