# ADR-0002: Web-Mercator scene, two-model split, depth-as-layer, and the shared map library

## Status

Accepted

## Context

`camp` (the deployed `CCOMAutonomousMissionPlanner`) and `camp2` (a second,
sandbox executable in the same CMake project) disagree at the most fundamental
layer of the UI: **what coordinate space the `QGraphicsScene` is in.**

- **Deployed camp**: the scene *is* the active background raster's pixel /
  chart-WKT space. `AutonomousVehicleProject` owns the scene
  (`autonomousvehicleproject.cpp:41`, `m_scene = new QGraphicsScene(this)`), a
  `BackgroundRaster` is the foundation, and every overlay positions itself by
  calling `geoToPixel(geo, bg)`, which delegates to the raster's per-file GDAL
  transform (`georeferenced.cpp:84`). Overlay items are **parented to the
  `BackgroundRaster`**, and `GeoGraphicsItem::geoToPixel` subtracts the parent's
  `scenePos()` to convert into parent-local coordinates
  (`geographicsitem.cpp:34-47`). Consequences: there is exactly one background
  at a time; the whole scene's projection changes when you load a different
  chart; there is no layer model; and ~24 source files (89 call sites) are
  wired directly to `geoToPixel`.

- **camp2**: the scene is global **Web Mercator (EPSG:3857)**, independent of
  any background. `camp::map::Map` owns the scene (`map.cpp:20`) and a tree of
  `MapItem`/`Layer` objects rooted at a single top-level item
  (`map.cpp:21-22`). Overlays position themselves with the free function
  `web_mercator::geoToMap(geo)` (`map_view/web_mercator.h:31`), whose output is
  "roughly meters relative to lat/lon 0,0". Rasters are reprojected to EPSG:3857
  on load via `GDALAutoCreateWarpedVRT(..., web_mercator::wkt, ...)`
  (`raster/raster_layer.cpp:78`) and become ordinary layers; OSM slippy tiles
  and WMTS are first-class tile layers; the multi-layer tree supports
  drag/drop reordering and per-layer visibility.

camp2 has the better framework (background-independent scene, multi-layer tree,
tiles); camp has the functionality (mission planning, depth-aware A*/survey,
AIS, collision monitor, platform/ship-track, nav-source, measuring/orbit
tools). PR2 already extracted camp2's map substrate into shared libraries
(`libcamp_map` / `libcamp_map_ros`) so both executables can link it. This ADR
records the decisions that let the deployed camp **adopt camp2's framework
while preserving camp's functionality**, rather than replace one app with the
other. It is the architecture record for issue
[#59](https://github.com/rolker/camp/issues/59) and lands with the PR3a scene
swap.

## Decision

### 1. The scene projection is Web Mercator (EPSG:3857), owned by `Map`

Deployed camp adopts camp2's `camp::map::Map` as the owner of the
`QGraphicsScene`. `AutonomousVehicleProject` stops owning a scene; its
`m_scene` is removed and the view (`ProjectView`, a `QGraphicsView`) renders
`Map::scene()` (`map.cpp:40`). Scene coordinates are Web-Mercator meters; geo→
scene conversion is `web_mercator::geoToMap` for every overlay. Backgrounds
become reprojected raster *layers* in the tree, not the scene foundation —
so multiple charts may be loaded, and switching charts no longer reprojects
the world.

### 2. Two models, not one: a Layer tree and a Mission tree

Today a single `treeView` (bound to `AutonomousVehicleProject`) mixes mission
items and the background. The migration yields two distinct models:

- **Layer tree** (`camp::map::Map` / `MapTreeView`): backgrounds, tiles, and
  ROS overlay layers, with inline per-layer visibility checkboxes.
- **Mission tree** (a slimmed `AutonomousVehicleProject`): the editable plan
  only. Background ceases to be a mission-tree node.

Both render into the same Web-Mercator scene owned by `Map`. The mission set is
*a layer inside that scene* but is edited through its own model/tree. UI layout
(decided with Roland): the two trees are **tabbed in a single dock** (not two
side-by-side panels); mission items do **not** appear in the layer tree (fully
separate — no mission-as-layer toggle); `detailsView` follows the active tab.

The four floating overlay-manager windows (Grids/AIS/Markers/Collision
Monitor), each opened from a menu action today, are retired — visibility moves
to the inline layer-tree checkboxes (PR3b/PR5).

### 3. Depth is a first-class layer type

camp2's raster path discards depth; camp's `BackgroundRaster` carries a depth
band and answers `getDepth(geo)` queries that feed depth-aware A*
(`astar.cpp`), survey/trackline generation, and the cursor depth readout. Depth
is **not** dropped. Instead of one implicit depth-bearing background, depth
becomes a first-class layer type: multiple depth layers are allowed (like
rasters) and toggled in the tree. The single `getDepthRaster()` accessor is
replaced by a `getDepth(geo)` query that walks the *enabled* depth layers in
tree order and returns the first valid sounding (layer order resolves overlap).
This lands in PR3c and is the **parity gate** that must close before camp's
`BackgroundRaster` is retired (see Phase 0 audit `docs/parity/raster.md`).

### 4. One shared map library, split at the ROS boundary

The camp2 map substrate is extracted into shared libraries that **both**
executables link (PR2, done):

- `libcamp_map` — pure Qt/GDAL core (`map`, `map_view`, `map_tiles`, `wmts`,
  `map_tree_view`, `background`, `raster`, `util`); zero ROS includes.
- `libcamp_map_ros` — the ROS-dependent layers (`ros/`), which PUBLIC-link
  `libcamp_map`.

camp2 is **not** retired; it stays as a sandbox executable that relinks the
libraries (self-verifying that the extraction preserved behavior). The
boundary is enforced at compile time: web-mercator/depth-order unit tests link
only the Qt/GDAL core. The dependency direction is one-way (core does not
depend on `ros/`); the PR2 inversion that achieved this — `Map` exposing
`toolsManager()` so the application layer (not the core) attaches the ROS node
(`map.cpp:26-30`) — is the mechanism that keeps it acyclic.

## Migration strategy: the `geoToPixel` shim

The blast radius of the projection swap is `GeoGraphicsItem::geoToPixel`, called
in ~24 files. Rather than rewrite all of them at once, PR3a introduces a
**shim**: the existing `geoToPixel` signatures are kept, but the body swaps the
coordinate source.

The key insight is that the parent-offset subtraction in `geoToPixel`
(`geographicsitem.cpp:42`, `ret - parentItem()->scenePos()`) is
**coordinate-system-agnostic** — it converts a scene-space point into
parent-local space and works in any units, provided the parent's `scenePos()`
is in the *same* space as `ret`. So once overlay items are reparented into the
Web-Mercator scene, the shim is:

```cpp
QPointF GeoGraphicsItem::geoToPixel(const QGeoCoordinate &point, BackgroundRaster *bg) const
{
    QPointF ret = web_mercator::geoToMap(point);   // was: bg->geoToPixel(point)
    if (QGraphicsItem *pi = parentItem())
        return ret - pi->scenePos();
    return ret;
}
```

The `bg` / `AutonomousVehicleProject*` parameters are retained for source
compatibility but no longer drive the result (the conversion no longer depends
on which background is active — that is the whole point of the Web-Mercator
scene). The real work of PR3a is therefore **reparenting** overlay items from
the `BackgroundRaster` into `Map`'s scene/layers, not editing the 24 call
sites. Per-overlay migration off the shim and onto `web_mercator::geoToMap`
directly happens incrementally in PR4/PR5; the shim is removed in PR6 once no
caller remains.

**Nothing is deleted in PR3a.** `BackgroundRaster`, `georeferenced.*`, and the
per-overlay code keep working through the shim until their replacements are
confirmed ≥ parity (the load-bearing rule from `docs/parity/README.md`).

## Consequences

- Every overlay's geo→scene conversion changes substrate (via the shim first,
  then per-overlay migration). Verified incrementally; parity-gated.
- `AutonomousVehicleProject` loses scene/background ownership and shrinks toward
  a mission model (`autonomousvehicleproject.{h,cpp}`).
- `ProjectView` mouse→geo conversion (`projectview.cpp`, currently
  `bg->pixelToGeo(mapToScene(pos))`) switches to `web_mercator::mapToGeo`.
- Charts load asynchronously and reproject to EPSG:3857; loading a second chart
  no longer reprojects the scene.
- Scale-dependent rendering (`bgr->mapScale()`) moves to the `MapView` viewport
  scale; distances stay geodesic.
- Settings are reset once (no QSettings key migration).
- Mission/Hypack export must read the mission model, not the scene (verify in
  PR4).
- The depth port (PR3c) gates retiring `BackgroundRaster`; markers/grids parity
  ports gate PR5 retirements.

## Alternatives considered

- **Replace deployed camp with camp2.** Rejected: camp2's ROS layer set is only
  geometry/grids/markers — it has no mission planning, AIS, collision monitor,
  platform/ship-track, nav-source, or measuring/orbit. Switching would lose the
  functionality that makes camp the operations tool.
- **Keep camp's chart-pixel scene, bolt on tiles/multi-layer.** Rejected: tiles
  and reprojected rasters are natively Web-Mercator; emulating a layer tree on
  a background-pixel scene reintroduces the single-background coupling this ADR
  removes.
- **Rewrite all 24 `geoToPixel` call sites in one PR.** Rejected in favor of the
  shim: a single mechanical edit to `geographicsitem.cpp` keeps every overlay
  compiling and running while migration proceeds incrementally and
  parity-gated.
- **One unified tree (mission + layers).** Rejected: the two models have
  different lifecycles and editing semantics; the tabbed two-model split keeps
  the mission plan editable in isolation while the layer tree manages display.

## References

- Issue [#59](https://github.com/rolker/camp/issues/59) — the port
- [ADR-0001](0001-topicbridge-and-executor-contract.md) — ROS/Qt threading
  contract for migrated overlays (rclcpp/tf2 stay)
- `docs/parity/README.md` + `raster.md`/`markers.md`/`grids.md` — Phase 0
  parity audit; the parity rule that gates every retirement
- `src/camp2/map/map.cpp:20` — `Map` owns the scene
- `src/camp2/map_view/web_mercator.h` — `geoToMap`/`mapToGeo`/`metersPerUnit`,
  EPSG:3857 WKT
- `src/camp/geographicsitem.cpp:34-47` — the shim target
- `src/camp/autonomousvehicleproject.cpp:41` — the scene camp currently owns
- Spin-off [#63](https://github.com/rolker/camp/issues/63) — reusable
  `camp::map::ColorMap` facility (out of #59 scope)
