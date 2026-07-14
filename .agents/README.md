# Agent Guide: camp

CAMP — the **C**COM **A**utonomous **M**ission **P**lanner: a Qt5 + ROS 2 (rclcpp)
desktop GUI for planning and monitoring autonomous marine-vehicle missions.

## Workflow

Standard worktree/PR workflow (this is a GitHub-origin repo). The default branch
is **`jazzy`**, not `main`.

CI (`.github/workflows/ci.yml`, colcon build + gtest suite) and pre-commit
(`.pre-commit-config.yaml`) are both configured — run the tests locally and
let pre-commit hooks run before pushing.

> When developed inside the `ros2_agent_workspace` (the usual case), the
> workspace's `.agent/scripts/` (field-mode detection, worktree helpers, etc.)
> apply from the workspace root — see that repo's `AGENTS.md`. They are not part
> of this standalone repo.

## Package Inventory

One ROS 2 / ament_cmake package (`camp`) that builds **one executable** and
**two shared libraries** from `CMakeLists.txt`:

| Target | Kind | Source | Role |
|--------|------|--------|------|
| `CCOMAutonomousMissionPlanner` | executable | `src/camp/` | The deployed product (mission planning + monitoring) |
| `camp_map` | shared lib | `src/camp_map/{map,map_view,raster,map_tiles,wmts,tools,background,util}/` | Web-Mercator scene + layer-tree framework (ROS-free) |
| `camp_map_ros` | shared lib | `src/camp_map/ros/` | ROS overlay framework (topic discovery, grids, markers, geometry) built on `camp_map` |

`CCOMAutonomousMissionPlanner` links the shared libs. The strategy (ADR-0002) was
that the deployed `camp` **adopts** the shared map framework rather than one app
replacing the other. That migration (#59) is now complete: the `camp2` sandbox
executable — once a dev/test harness for the framework in isolation — has been
**retired**, and its map framework lives in `src/camp_map/` as the `camp_map` /
`camp_map_ros` libraries that `CCOMAutonomousMissionPlanner` consumes. The
`QAbstractItemModelTester` check the sandbox once ran at runtime now lives in CI
as the `test_map_model` gtest.

## Repository Layout

```
src/camp/        deployed app (CCOMAutonomousMissionPlanner): mission model,
                 overlays, ROS link, details/manager widgets
src/camp_map/    the shared map framework (camp_map / camp_map_ros libraries)
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

**Browse vs. compose for GGGS stores (ADR-0005):** a GGGS tile *store* is
**browsed** through the generic catalog browser (`src/camp_map/catalog/` — a
ROS-free `CatalogModel`/`CatalogSource`/`CatalogBrowser` seam, shown as the
"Stores" tab beside the Layers tab), not mounted into the Layers tree. Selecting
a tile-set spawns an independent **flat top-level `GggsTileLayer`** the operator
composes in the Layers tree (visibility/opacity/draw-order are free). Selected
layers persist under `QSettings GggsTileLayers/dirs` (the dirs to recreate;
per-layer visibility/opacity persist via the layer's own settings).
**Two orthogonal persistence records** (ADR-0005 §5): the *browsed store root(s)*
also persist, under the generic `QSettings CatalogBrowser/seeds` key
(`sourceId<TAB>root` entries), so the Stores tab repopulates its **browse tree**
on launch (`CatalogBrowser::restoreSeeds()`, run after `addSource`, skip-missing
on a vanished root; a "Remove from browser" context action de-persists a
mis-picked root). This seed persistence rebuilds the tree **only** — it spawns no
layers and does not resurrect the retired nested store node; it is independent of
the `GggsTileLayers/dirs` flat-layer (display) persistence above. This
**replaced** camp#90's nested `GggsStoreLayer` (store folders straight into the
tree, persisted as store *roots*), which is retired. `GggsStoreSource` is the
first and only `CatalogSource`; the seam is reusable for future layer-manager
items (cf. topic discovery #44/#68/#69). The retired store node also carried a
`QFileSystemWatcher` (camp#102 live tile pickup); a flat layer instead exposes a
**manual "Rescan" context-menu action** (right-click → Rescan re-enumerates the
tile-set directory for newly-landed tiles). Live auto-pickup (a per-layer
watcher) is a follow-up.

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
- `src/camp_map/map/map.{h,cpp}` + `map/layer.{h,cpp}` — the layer model.
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
  in `main.cpp` — otherwise it persists to "Unknown Organization" and splits its
  state across stores. These names must stay stable across releases so the
  persisted chart list / layer settings survive upgrades.
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
  Likewise ADR-0005 reset GGGS store persistence: the old `GggsStores/roots` key
  is ignored and cleared once on startup (not migrated) — stores re-added through
  the Stores tab persist as flat `GggsTileLayers/dirs`.
- **CI + pre-commit are configured** — GitHub Actions runs the colcon build +
  gtest suite on PRs; pre-commit hooks guard formatting/config hygiene.

## Instructions for Use

Read this guide and the relevant ADRs before modifying the scene, layer, depth,
or ROS-link subsystems. Verify any documentation claim against the source before
relying on it.
