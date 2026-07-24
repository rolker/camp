# ADR-0003: Backgrounds as stackable layers, an independent depth-layer tree, and split persistence

## Status

Accepted

Extends [ADR-0002](0002-web-mercator-scene-and-layer-model.md).

## Context

ADR-0002 moved the deployed `camp` onto camp2's Web-Mercator scene and the
two-model (mission tree / layer tree) split, and set the *direction* "depth is a
first-class layer type." To get there incrementally it kept the
`BackgroundRaster` alive as a transitional object: through PR3a it remained in
the scene as a non-painting origin anchor that overlays parented to, and as the
depth/georeference oracle.

The PR6 increments then hollowed it out:

- **Increment 1** extracted depth into a standalone `DepthRaster` provider;
  `AutonomousVehicleProject::getDepth(geo)` no longer reads the `BackgroundRaster`.
- **Increment 2** re-homed *every* overlay (mission items, AIS, platform,
  nav_source, collision monitor) onto the Map's persistent scene-root anchor
  (`Map::rootItem()`), so nothing parents to the `BackgroundRaster` anymore.
- **Increment 3** replaced `BackgroundRaster::scaledPixelSize()` icon scaling
  with a chart-independent, latitude-corrected `GeoGraphicsItem::metresPerPixel()`.

After those, the `BackgroundRaster` does almost nothing: the chart *image* is a
reprojected `camp::raster::RasterLayer`, depth is a `DepthRaster`, and overlay
scale comes from the view. What remains is vestigial — it is still a node in the
**mission** tree (so the chart wrongly appears in the Mission tab and stale
entries accumulate when charts are swapped), it is still added to the scene
though nothing reads it there, and a single hard dependency remains: the
view's fit-to-extent reads the chart georeference.

Two facts shape the decision:

1. **Multiple backgrounds is a real, wanted capability.** Deployed camp could
   always load several backgrounds and display one at a time. The Web-Mercator
   layer model makes the "one at a time" restriction unnecessary.
2. **We are not preserving the old on-disk project format.** Backgrounds may be
   redesigned freely, without compatibility shims for existing `.json` files.

This ADR records how backgrounds fully dissolve into the layer model.

## Decision

### 1. Backgrounds are ordinary, stackable layers

Each loaded chart becomes a `camp::raster::RasterLayer` in the **Layers** tree.
Several may be loaded at once; **visibility, stacking order, and opacity** govern
what is displayed. The "one displayed background at a time" restriction is
removed — it was a limitation of the old single-`m_currentRasterLayer`,
background-is-the-scene world, not a desired behavior. The
`AutonomousVehicleProject` single-current-background notion
(`m_currentBackground`, `setCurrentBackground`) is retired in favor of the set
of raster layers the Map already manages.

### 2. Visual and depth are independent layer trees; depth is a subtree in Layers

Depth is decoupled from imagery. Opening a chart contributes, **independently**:

- a `RasterLayer` (imagery) to the visual layer tree, when the file has a
  displayable image, and
- a depth layer (wrapping `DepthRaster`) to a **Depth subtree** within the same
  Layers tab, when the file carries a depth band.

The two are **fully independent even when co-sourced from the same file**:
independent ordering, independent enable/disable, and independent lifetime —
deleting the imagery layer does not remove the depth layer, and vice-versa. This
lets the operator, e.g., display one chart's imagery while planning against a
different chart's bathymetry.

The depth tree is a **subtree inside the Layers tab**, not a third top-level tab
— it keeps all display/derived data in one place while still giving depth its
own orderable, toggleable list. `getDepth(geo)` walks the **enabled** depth
layers in depth-tree order and returns the first valid sounding (order resolves
overlap) — the query ADR-0002 §3 specified, now literally driven by a tree the
operator can reorder. `hasDepth()` is true when any enabled depth layer covers
the query. This is the realization of ADR-0002's "depth is a first-class layer
type."

### 3. `BackgroundRaster` is deleted

With its three jobs reassigned — image → `RasterLayer`, depth → depth layer /
`DepthRaster`, georeference/extent → the `RasterLayer`'s own Web-Mercator scene
extent — the `BackgroundRaster` class is removed entirely, along with its
mission-tree membership and its now-unread scene presence.

The last hard dependency, `ProjectView` fit-to-extent (today
`bg->boundingRect()`/`pixelToGeo()`), is re-sourced from the `RasterLayer`'s
scene extent. `RasterLayer` currently loads fully asynchronously, so its
`boundingRect()` is empty until the warp completes (`raster_layer.cpp:40-44`,
`88`) — fitting at open time would fit to nothing. **Resolution: make
`RasterLayer` compute its extent synchronously in the constructor.**
`GDALAutoCreateWarpedVRT` + `GetGeoTransform` + `GetRasterXSize/YSize` read no
pixels and are effectively instant, so the layer can set its `scenePos()` and
return a valid `boundingRect()` immediately (painting nothing until the async
pixel/mipmap build finishes), rather than only after `imageReady()`. This fixes
the root async/extent mismatch instead of working around it, benefits camp2
(layers know their place before their pixels load), and keeps **zoom-to-chart on
open** working with no fit-intent flags or duplicated GDAL code. The abort /
async-reload path (`raster_layer.cpp:71-94`) is preserved — only the cheap
extent metadata moves to the constructor; the pixel warp stays on the worker
thread.

*Note (future):* a "Zoom to extents" action on layer-tree items would let the
operator re-frame any layer on demand; if that lands, auto-zoom-on-open could be
reconsidered (it stays, as today, for now).

The vestigial machinery goes with it:
`findParentBackgroundRaster()` (already dead), the `BackgroundRasterType` enum
entry, and the `geoToPixel(point, bg)` / `geoToPixel(point, AVP*)` overloads
collapse into the single bg-free `geoToPixel(point)`. `georeferenced.{h,cpp}`
**stays** — `DepthRaster` and `VectorDataset` still inherit it.

### 4. Split persistence: backgrounds persist app-level, missions via project files

The two models persist by different mechanisms, matching their different
lifecycles:

- **Background and depth layers persist as application/Map state** (camp2-style)
  — the set of loaded layers, their visibility, order, opacity, and colormap
  come back on restart, independent of any mission. They are not mission data.
- **Mission items keep the traditional explicit load/save** to a project file.

The mission project file therefore **no longer carries any background entry**.
Because we are not preserving the old format, there is no `BackgroundRaster`
node and no compatibility shim in the mission read/write path — the chart
reference simply lives in Map state. Chart removal moves from the Mission tree's
delete to a **Remove action on the layer** in the Layers tab (the layer's
context menu), where backgrounds now live.

#### Addendum ([#117](https://github.com/rolker/camp/issues/117), 2026-07-24): tile-layer persistence schema and seed

Tile/WMTS background layers, the last hard-coded holdouts from before this
ADR's implementation, now follow §4's app-level persistence. The QSettings
schema (`tile_layer_presets.h`):

- `BackgroundTileLayers/seeded` — first-run sentinel. The one-time seed fires
  only when this key is absent (covers fresh installs *and* upgrades from the
  hard-coded era) and writes it. Gating on the sentinel rather than the ids
  list means an operator who removes every tile layer stays at zero layers
  across restarts.
- `BackgroundTileLayers/ids` — QStringList of layer names, creation order.
- `BackgroundTileLayers/<enc(name)>/` — per-layer **construction parameters
  only** (`type` = `xyz`|`wmts`, `url`, `refresh_ms`, WMTS `layer_id` /
  `tile_matrix_set`), the name percent-encoded to one flat key (the
  `GggsTileLayer::settingsKey()` pattern).

Presentation state (opacity, visibility) is deliberately **not** duplicated
here — the existing `MapItem/<settingsKey()>` mechanism owns it; the add path
writes the preset's defaults there once so the deferred `readSettings()`
applies them. The seed is **OSM only** (operator decision, 2026-07-24);
everything else — including the former hard-coded OpenSeaMap / NOAA ENC /
NEXRAD layers and verified bathymetry sources (NOAA BlueTopo WMTS; GEBCO,
inert until [#118](https://github.com/rolker/camp/issues/118) lands WMS) — is
operator-added via the preset table in the "Add tile layer" dialog. Removal
de-persists in `MapTiles::onRemovedFromMap()` (the layer owns its Remove, per
§4), gated on membership in `ids` so non-persisted `MapTiles` uses are
untouched.

### 5. A depth-only load may offer to generate a visual layer

When a file carries depth but no displayable image (e.g. a bathymetry grid),
loading it adds a depth layer and **offers to generate a depth-shaded visual
layer** from it, using the `camp::map::ColorMap` depth-shading facility
([#63](https://github.com/rolker/camp/issues/63)). The generated visual layer is
an ordinary, independent `RasterLayer` (it can be removed or hidden without
affecting the depth layer). This keeps imagery and depth independent (§2) while
giving the operator a one-click way to *see* bathymetry-only data.

## Consequences

- `AutonomousVehicleProject` sheds background ownership entirely: `openBackground`,
  `setCurrentBackground`, `getBackgroundRaster`, `m_currentBackground`,
  `m_currentRasterLayer` (single) and the mission-tree insert/delete for
  backgrounds are removed or moved into the Map/layer model.
- Loading a chart no longer touches the mission model; the Mission tab shows only
  the plan. The "chart in both tabs" duplication and the swap-time accumulation/leak
  disappear by construction.
- `RasterLayer` computes its scene extent synchronously in the constructor
  (cheap GDAL metadata); the pixel warp stays async. `ProjectView` fit-to-extent
  re-sources from the layer extent, so zoom-to-chart-on-open keeps working
  immediately. (Touches shared `libcamp_map`, so camp2 benefits too.)
- The `geoToPixel` overloads collapse to one; ~40 `geoToPixel(., avp)` /
  `geoToPixel(., bg)` call sites become `geoToPixel(.)` (mechanical, all already
  bg-independent). The two remaining `bgr->mapScale()` glyph-scale readers
  (`geographicsmissionitem` arrow, `waypoint` shape) move to
  `AutonomousVehicleProject::mapScale()`.
- A new depth-layer type + its subtree view, and Map-state persistence for the
  layer set, are net-new code. `DepthRaster` (increment 1) is the data backend.
- Mission/Hypack export already reads the mission model (ADR-0002), so dropping
  backgrounds from the mission file does not affect it.
- No QSettings/project-file migration: existing saved projects that contain a
  `BackgroundRaster` node will simply ignore it (the node type no longer exists);
  their charts are re-loaded from Map state instead. This is acceptable per the
  "no old-format compatibility" decision.

## Alternatives considered

- **Keep `BackgroundRaster` as a headless depth/georef holder** (the original
  increment-5 framing). Rejected: it forces the chart to remain a pseudo
  mission-item for persistence, which reintroduces the mission-tree coupling and
  the duplication/accumulation, and keeps a class that does nothing the
  `RasterLayer`/`DepthRaster` pair doesn't already do.
- **One displayed background at a time** (preserve old behavior, radio-style
  active layer). Rejected: the single-background restriction was a limitation of
  the old scene model; stacking is strictly more capable and is what the layer
  model already provides.
- **Depth as a separate top-level tab.** Rejected in favor of a subtree inside
  the Layers tab: depth is derived/display-adjacent data; a third tab fragments
  the layer UI without benefit, while a subtree still gives depth its own
  orderable/toggleable list.
- **Link co-sourced visual+depth layers** (deleting one removes both).
  Rejected: independent lifetimes are simpler and more flexible (display chart A,
  plan on chart B's depth), and the "generate visual from depth" path (§5)
  already produces a deliberately-independent visual layer.
- **Preserve the old project-file format** (serialize a `BackgroundRaster` child
  entry for back-compat). Rejected: we explicitly chose not to carry the old
  format; backgrounds are app/Map state, not mission data, so a compat shim would
  be inelegant and would re-couple the mission file to backgrounds.

## References

- Issue [#59](https://github.com/rolker/camp/issues/59) — the port
- [ADR-0002](0002-web-mercator-scene-and-layer-model.md) — Web-Mercator scene,
  two-model split, depth-as-layer direction (this ADR realizes §3 and completes
  the background-as-layer migration it began)
- [#63](https://github.com/rolker/camp/issues/63) — `camp::map::ColorMap`
  depth-shading facility, used by §5's generated visual layer
- `.agent/work-plans/issue-59/progress.md` — "BackgroundRaster Retirement"
  increment plan and the increment 1–3 records this ADR builds on
- `src/camp/depth_raster.{h,cpp}` — the depth backend (increment 1)
- `src/camp2/raster/raster_layer.cpp` — the reprojecting visual layer; source of
  the chart scene extent for fit-to-extent
- `src/camp2/map/map.cpp` — the Map model that owns the layer tree and will own
  background/depth persistence
