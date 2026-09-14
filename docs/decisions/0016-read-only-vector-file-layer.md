# ADR-0016: A read-only vector-file layer family, beside the editable import

## Status

Accepted

Implements CAMP issue [#22](https://github.com/rolker/camp/issues/22). Builds on
[ADR-0002](0002-web-mercator-scene-and-layer-model.md) (the Web-Mercator scene +
`map::Layer` model, and the `libcamp_map` ROS-free / no-call-into-the-executable
boundary), [ADR-0003](0003-backgrounds-as-layers-and-depth-tree.md) (backgrounds
as layers, and §4's app-state persistence pattern this reuses for the file list),
and [ADR-0008](0008-adopt-marine-colormap-lut-bake.md) (the `marine_colormap`
registry the colour-by-field pass samples). Departs deliberately from
[ADR-0007](0007-raster-field-source-interface.md), which unified the *raster*
layer families behind one render path — see D1.

> **Numbering.** This is **camp ADR-0016**, a project ADR in this repo's series.
> The workspace repo has its own independent ADR series; numbers collide by
> coincidence.

## Context

CAMP could already open an OGR vector file — GeoJSON, shapefile, GeoPackage,
KML — through **File > Open Geometry**. `VectorDataset` (`src/camp/vector/`)
reads it and builds `MissionItem`s: a `Group` of `Point` / `LineString` /
`Polygon` nodes in the mission tree, each editable, draggable, renameable and
sendable to the robot, persisted in the mission project file.

That is the wrong shape for the job issue #22 describes. The operator wants to
*see* reference data on the map — a magnetic-anomaly candidate list, a survey
boundary, a cable route, a shoreline — coloured by one of its attribute fields,
clickable for its attributes, and back where they left it next time CAMP starts.
None of that is mission data:

- It must not be editable. A contact list dragged half a mile by an accidental
  mouse-down is a data corruption the operator cannot see happening.
- It must not reach the robot. Everything in the mission tree is a candidate for
  transmission; reference data is not.
- It must not live in the mission file. The operator expects the layer back
  whether or not a mission is loaded, which is the *chart list's* lifetime
  (ADR-0003 §4), not the mission's.
- It needs attribute-driven display — colour by field, marker size by field,
  click for attributes — which `MissionItem` has no notion of at all.

Meanwhile all three existing `map::Layer` families were **raster**: charts
(`RasterLayer`), GGGS tile sets (`GggsTileLayer`), and tile/WMTS backgrounds
(`MapTiles`). ADR-0007 unified their render paths behind `RasterFieldSource` and
one GL renderer. A vector layer is the first family that shares none of that
machinery, so the question of whether it belongs in the same layer model at all
had to be answered rather than assumed.

## Decision

1. **A fourth layer family, `camp::vector::VectorLayer`, and it is NOT routed
   through ADR-0007's raster path.** It is a `map::Layer` — so it gets the
   Layers tab, visibility, opacity, ordering, per-item QSettings and the removal
   lifecycle for free — but it renders as one child `QGraphicsItem` per feature
   rather than as a texture. ADR-0007's abstraction is over *raster field
   sources*: a texture, a geographic extent, a NoData sentinel, a colormap LUT.
   A feature has none of those. Forcing vector data through it would mean
   rasterizing on the CPU and throwing away exactly what makes the data useful —
   per-feature identity, which is what click-to-inspect and per-feature styling
   are. ADR-0007's unification is not weakened: it remains the one path for
   raster, and there are still three raster layers and one renderer.

2. **Per-feature child items, not one painted surface.** `VectorFeatureItem` is a
   plain `QGraphicsItem` (not a `map::MapItem`, not a `QGraphicsObject`): a
   feature is not a row in the Layers tree, owns no settings group, needs no
   signal/slot machinery, and a layer holds thousands. Making each feature an
   item makes hit-testing Qt's problem instead of ours — which is the whole
   mechanism behind click-to-inspect. It also means Qt's scene index, not our
   code, decides what is drawn at a given viewport.

3. **Both entry points share one parser, and it lives in `camp_map`.**
   `camp::vector::parseVectorLayers()` (`src/camp_map/vector/vector_parse.cpp`)
   is the single OGR front end for *both* `VectorDataset` (the editable import,
   in the executable) and `VectorLayer` (in the library). It moved into
   `camp_map` because a library cannot call into the executable that links it —
   the `libcamp_crash` rule from camp#217. It emits plain data
   (`ParsedGeometry`: WGS84 coordinates plus a name-keyed attribute map) with no
   `MissionItem` or project coupling, which is also what lets it be unit-tested
   headlessly — the #152 resource-lifecycle leaks were previously unreachable by
   any test.

4. **The parser is faithful; PLACEMENT policy belongs to the display layer.**
   The parser reports coordinates as the file states them and counts what it
   could not read (`ParseDiagnostics`). Deciding whether a coordinate can be
   placed on a Web-Mercator scene is `camp::vector::isPlaceable()`, in the
   display layer, because "placeable" is a scene property. What the parser
   refuses to do is *guess*: a per-point transform failure is dropped rather than
   carried at OGR's `HUGE_VAL`, and a layer that declares a spatial reference
   from which no transformation to WGS84 can be built is **failed outright**
   rather than falling through to the untransformed branch — reading projected
   metres as degrees places features ~1e17 m from where they belong, silently.

5. **Read-only means read-only, everywhere.** No editing, no dragging, no
   waypoint linking, nothing sent to the robot. The one interaction is
   click-to-inspect, and it is gated: it fires on left-button *release without
   movement*, and only when the view the event came from is in pan mode
   (`ScrollHandDrag`). ProjectView places mission items on left-press in its
   add-* modes, and a press that becomes a drag is a pan gesture. The mode is
   read from the view's own drag mode rather than by asking ProjectView, again
   because camp_map cannot call into the executable.

6. **Styling is attribute-driven, through `marine_colormap` (ADR-0008), and
   total.** Colour-by-field samples the selected palette across the field's
   extent *over the features that have a value*; size-by-field scales point
   markers only (there is no honest reading of "a polygon's size" from an
   attribute) and folds its range over points only. Every mapping function has a
   defined, documented result for a field that is missing on a feature, holds a
   non-numeric or NaN value, or is identical across every feature — and the one
   thing none of them may do is look like a legitimate low value. A missing value
   is painted in a fixed no-data grey that is not a position on any palette; a
   degenerate range maps to the palette midpoint (not an extreme, and not a NaN:
   the offset-the-low-bound guard used elsewhere is a no-op above 2^53, which
   OGR int64 ids and nanosecond timestamps reach).

7. **Persistence is APP STATE, and its schema is one key.** The layer list is
   `QSettings` `vectorLayers/files` — a `QStringList` of canonical file paths, in
   layer order, de-duplicated — the same shape as `backgrounds/files` for charts
   (ADR-0003 §4) and deliberately *not* the mission project file. Per-layer style
   persists separately under `MapItem/file:<percent-encoded path>` with keys
   `color_field`, `size_field` and `colormap`, alongside the presentation keys
   `map::Layer` already writes. The group is keyed on the **full path**, not the
   `itemID()` default (which derives from the basename), so two files named
   `candidates.geojson` in different directories do not share one style —
   camp#126's collision.

8. **`AutonomousVehicleProject::persistVectorLayers()` is the single writer of
   `vectorLayers/files`,** rebuilding the whole list from the layers it tracks on
   both the add and the remove path, so the two halves cannot disagree. The
   layer does not write the key itself. This is why `VectorLayer` overrides
   `onRemovedFromMap()` only to *emit a signal*, unlike `RasterLayer` and
   `GggsTileLayer`, which write their own keys there.

9. **Removal is observed through `Layer::onRemovedFromMap()`, never through the
   model's `rowsAboutToBeRemoved`.** `Map::setMapItemParent()` implements a
   drag-reorder as `beginRemoveRows` + `beginInsertRows`, so the model signal
   cannot distinguish "the operator moved this layer up the list" from "the
   operator removed it". Watching it — which this feature did first — meant a
   reorder silently un-persisted the layer. `onRemovedFromMap()` is called only
   by `removeFromMap()`, which is the removal gesture and nothing else. This is
   the camp#90 / camp#104 hook, used here for the property it has rather than
   for the key it writes.

10. **A persisted file that is unreachable is REMEMBERED, not dropped — in
    place.** Survey data lives on network shares and external disks; "the share
    was not mounted when CAMP started" is not the operator asking for the layer
    to be removed. Such an entry is carried forward into the rewritten key and
    restores on the next launch that can see it, keeping its POSITION in the
    list: the key is rewritten in the order it was read, so one launch with the
    share unmounted does not permanently reshuffle the operator's layer order.

11. **Two bounded resources, both reported rather than silently clipped.** The
    parse runs on a QtConcurrent worker whose abort flag is polled *per feature*,
    so the destructor's join is bounded by the abort and not by the size of the
    file (camp#213). And the file is read only as far as
    `VectorLayer::kMaxFeatureItems` (50 000) — an unbounded file dialog plus a
    national coastline shapefile is otherwise a frozen CAMP with no message.

    The cap is carried **into the parse** (`ParseOptions::max_geometries`), which
    stops there, rather than applied to the parse result. Item construction can
    only happen on the GUI thread, which is what first motivated the cap; but a
    cap applied after `parseVectorLayers()` returns bounds only that half, while
    the worker has already materialised every geometry and attribute map of the
    whole file — responsive, and out of memory. Capping the parse bounds both.

    Whenever the cap, an unplaceable feature, a ring-less polygon, or a failed
    layer applies, the Layers-tab status and the log say so: a layer showing 2 of
    5 features and reporting "(2 features)" is indistinguishable from a file that
    holds 2. The one thing not reported is *how many* features were left unread
    when the cap hit — counting them means reading the file the cap exists to
    stop reading, and an OGR feature count is not a geometry count anyway.

12. **A `/vsi` path is refused, and the driver set is pinned.** `GDALOpenEx` is
    handed an operator-supplied string, which OGR treats as a *connection*
    string, so two different things have to be said no to.

    - **Remote and archive fetches are stopped by refusing the PATH**
      (`camp::vector::isVirtualFileSystemPath`), checked in `VectorLayer`'s
      constructor before anything is opened and in
      `AutonomousVehicleProject` on both the open and the restore path. The
      driver allowlist cannot do this job: GDAL resolves `/vsicurl/`,
      `/vsizip/`, `/vsis3/` … in its virtual file system *before* a driver is
      selected, so `/vsicurl/https://host/x.geojson` is fetched and then read
      by the perfectly-allowed GeoJSON driver — confirmed against GDAL 3.8.4
      with exactly this allowlist. The restore path is the one that matters:
      `restorePersistedVectorLayers()` reopens every persisted entry at startup
      with no operator present, so a `/vsi` entry is dropped from the list
      rather than remembered.
    - **Non-file drivers are excluded by the allowlist** — `PG:`, `MySQL:`,
      `WFS:`, `OAPIF:` and the rest are not on it, so a connection string
      cannot open a database or a service from the load worker. That, and not
      the network, is what the list buys. Adding a format is a deliberate edit
      to it.

    **Known limitation:** `KML`/`LIBKML` are on the allowlist — operators are
    handed KML routinely — and a KML file can carry a `NetworkLink` that the
    driver may follow. Nothing here blocks that: the exposure is a fetch
    initiated by file *content*, after the operator chose to open that file,
    rather than by the path CAMP was given. Dropping the KML drivers, or
    disabling network access at the GDAL configuration level
    (`GDAL_HTTP_*`/`CPL_VSIL_CURL_*`), would close it and is not done here.

## Consequences

- **CAMP now has two vector-file entry points with similar names**, and picking
  the wrong one is the easy mistake: File > Open **Geometry** imports editable
  mission items, File > Open **Vector Layer** displays a read-only layer. The
  distinction is recorded in `.agents/README.md` and in `vector_layer.h`; the
  menu labels are the only thing standing between the operator and the wrong
  one, and renaming them is a follow-up worth considering.
- **`parseVectorLayers()` is now shared code with two callers**, so a change to
  its geometry or attribute handling affects the mission-tree import as well.
  Its tests cover the parser directly for that reason.
- **The persisted schema is public.** `vectorLayers/files` and
  `MapItem/file:<path>` are app state an operator's settings file carries across
  upgrades; changing either shape needs a migration or it silently loses layers.
- **An unreachable persisted entry cannot be removed through the UI** — it has
  no layer to right-click. It goes when the file returns and is removed, or by
  clearing the setting. This is the deliberate cost of D10.
- **The feature cap is a display limit an operator can hit** on genuinely large
  data (a national coastline, an OSM extract). Spatial-index-backed culling or
  level-of-detail would lift it; both are larger work than this issue, and the
  cap is honest in the meantime.
- **Removal leaves the layer's `MapItem/file:<path>` settings group behind.**
  `MapTiles` removes its group (camp#117's convention); `RasterLayer` does not.
  This layer follows `RasterLayer`, so re-adding the same file restores its
  style — which is the friendlier behaviour for a file the operator opens
  repeatedly, at the cost of an orphan group for one they never open again. The
  split precedent is recorded, not resolved.
- **A pan that starts on a feature does not pan the map** (camp#225). The item
  accepts the left press in pan mode, which is what lets the release tell a
  click from a drag, and that press therefore never reaches `QGraphicsView`'s
  ScrollHandDrag. An item cannot have both; the fix belongs in `ProjectView`,
  which sees the gesture and the items under it, and is tracked separately.
- **Cross-antimeridian geometry is a known limitation.** A line or polygon whose
  vertices straddle 180° is drawn the long way round the world and stretches the
  layer's extent with it. The parser reports what the file says; nothing splits
  geometry at the seam. It has not been made to fail loudly because a correct
  split is real work, and the drawing is visibly wrong rather than plausibly
  wrong.
- **Label-by-field and a style-editing dialog are deliberately absent.** The
  context menu offers colour-by, size-by and palette; anything richer is a
  follow-up, per the issue's own MVP scope.
