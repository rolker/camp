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
able to tell them its attributes when pointed at, and back where they left it next
time CAMP starts.
None of that is mission data:

- It must not be editable. A contact list dragged half a mile by an accidental
  mouse-down is a data corruption the operator cannot see happening.
- It must not reach the robot. Everything in the mission tree is a candidate for
  transmission; reference data is not.
- It must not live in the mission file. The operator expects the layer back
  whether or not a mission is loaded, which is the *chart list's* lifetime
  (ADR-0003 §4), not the mission's.
- It needs attribute-driven display — colour by field, marker size by field,
  hover for attributes — which `MissionItem` has no notion of at all.

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
   per-feature identity, which is what hover-to-inspect and per-feature styling
   are. ADR-0007's unification is not weakened: it remains the one path for
   raster, and there are still three raster layers and one renderer.

2. **Per-feature child items, not one painted surface.** `VectorFeatureItem` is a
   plain `QGraphicsItem` (not a `map::MapItem`, not a `QGraphicsObject`): a
   feature is not a row in the Layers tree, owns no settings group, needs no
   signal/slot machinery, and a layer holds thousands. Making each feature an
   item makes hit-testing Qt's problem instead of ours — which is the whole
   mechanism behind hover-to-inspect. It also means Qt's scene index, not our
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

5. **Read-only means read-only, everywhere, and the one interaction is HOVER.**
   No editing, no dragging, no waypoint linking, nothing sent to the robot. A
   feature answers the cursor arriving on it with its attributes, **instantly**,
   in an **in-scene label** — a child `QGraphicsSimpleTextItem` filled by
   `hoverEnterEvent()` and emptied by `hoverLeaveEvent()`. It is not a persistent
   panel.

   **The label is the vessel/AIS mechanism, replicated.** `GeoGraphicsItem`
   (`src/camp/geographicsitem.cpp:14-26`) owns a child `QGraphicsSimpleTextItem`
   with `ItemIgnoresTransformations`, a 20 pt bold font, a black brush and a
   width-0 white pen; `Platform` and `AISContact` fill it in `hoverEnterEvent()`
   and clear it in `hoverLeaveEvent()`. `VectorFeatureItem` lives in `camp_map`
   and cannot depend on `GeoGraphicsItem` in the `camp` executable (the same rule
   that put `parseVectorLayers()` in `camp_map`), so those settings are **copied,
   with the source named in the comment**. Only one label is ever on screen
   because every item clears its own on leave. The label item is created on the
   **first hover**, not at load: a layer may hold `kMaxFeatureItems` (50 000)
   features and the operator hovers a handful.

   **The hovered feature is also raised above its siblings, not just its
   label.** Feature items are siblings created in file order with no `zValue()`
   of their own, and Qt stacks a child with its parent's subtree, so a label
   parented to a point would still paint under a polygon loaded after that
   point no matter how high the label's own `zValue()` is set. `hoverEnterEvent()`
   raises the whole item instead; `hoverLeaveEvent()` puts it back into file
   order. This is operator-visible: the feature under the cursor comes to the
   front of its layer for the duration of the hover — which is also what an
   operator pointing at a feature wants.

   **A Qt tooltip was tried first, and the operator rejected it after testing
   it.** The first hover implementation was `setToolTip(attributeText())`, shown
   by `QGraphicsScene::helpEvent()`. It works, and it costs no per-item hover
   tracking — but it waits out Qt's tooltip delay, and the operator's verdict on
   the 2026-09-15 GUI test of it was "similar to what was existing, but not the
   same": every other CAMP item answers the cursor at once. The reason for
   choosing hover in the first place was consistency with the rest of the
   application, and a popup that behaves differently from every other popup does
   not deliver it. Accepting hover events across the layer's items is the price,
   and it is paid deliberately.

   **Hover, because that is CAMP's house convention for "tell me what this is."**
   `Platform` and `AISContact` show their label on hover
   (`setShowLabelFlag(true)` in `hoverEnterEvent`), `GeoGraphicsMissionItem`
   brightens on hover, and **nothing in CAMP inspects on click**. This layer
   shipped its first GUI test (2026-09-15) with click-to-inspect, and the operator
   decided from that test to align it with the rest of the application. Consistency
   here is not cosmetic: a click means "act on this" everywhere else in the map
   view — placing a waypoint, starting a pan — and a read-only layer has no action
   to offer.

   **Hover also costs nothing else, which click did.** A `VectorFeatureItem`
   accepts **no mouse button at all** (`setAcceptedMouseButtons(Qt::NoButton)`;
   `QGraphicsItem` accepts the left button by default, so this has to be said), so
   every press over a feature falls straight through to `QGraphicsView`. That
   deletes a whole apparatus the click version needed, two pieces of which had
   already cost a review round each to get right:

   - There is no mode gate. The click version had to read the view's drag mode to
     tell "the operator is inspecting" from "the operator is placing a waypoint",
     and it had to read it from the *event's* view rather than by asking
     ProjectView, because `camp_map` cannot call into the executable that links it
     (#217).
   - There is no coupling to `ProjectView`. The gate depended on
     `ProjectView::mousePressEvent()` deferring its switch back to pan mode until
     after the press had been forwarded — otherwise the item read "pan" during the
     very click that placed a mission item and answered it with a tooltip on top
     of the item just placed. That deferral (commit `e2a56cc`) was introduced for
     this gate alone and rewired the press path of every add-\* mode that returns
     to pan; it has been reverted. The only `ProjectView` change this branch
     carries is the pan-mode arrow cursor of **D16** — the press path itself is
     back to `jazzy`.
   - There is no click/drag slop. Telling a click from the start of a pan needed a
     screen-pixel threshold between press and release — which a first attempt
     compared against a *scene-metre* delta whenever the event carried no widget,
     a different tolerance at every zoom level.

   The one thing an item that accepts no mouse button cannot do is pin the popup
   open on click. That is a follow-on (see Consequences), and it would have to be
   built without taking the press away from the view's gesture.

6. **Styling is attribute-driven, through `marine_colormap` (ADR-0008), and
   total.** Colour-by-field samples the selected palette across the field's
   extent *over the features that have a value*; size-by-field scales point
   markers only (there is no honest reading of "a polygon's size" from an
   attribute) and folds its range over points only. Every mapping function has a
   defined, documented result for a field that is missing on a feature, holds a
   non-numeric or NaN value, or is identical across every feature — and the one
   thing none of them may do is look like a legitimate low value. A missing value
   is painted in a fixed no-data grey; a degenerate range maps to the palette
   midpoint (not an extreme, and not a NaN: the offset-the-low-bound guard used
   elsewhere is a no-op above 2^53, which OGR int64 ids and nanosecond timestamps
   reach).

   **Colour alone cannot carry "no data".** The grey is distinct from both ends of
   every shipped palette, but not from the MIDDLE of `grayscale` — a palette the
   operator can select, and the fallback for an unknown palette name — so under it
   a missing value would be indistinguishable from a mid-range measurement. A
   no-data feature is therefore also drawn with a **dashed outline and a hatched
   fill** (a hollow marker for a point): channels the palette does not touch,
   which hold up under grayscale, colour-blind vision and a monochrome printout
   alike. The predicate is `camp::vector::isNoData()`, so the colour and the
   outline cannot drift apart.

   The hollow point marker is stroked at **width 2**, the same weight a line or a
   polygon outline is drawn at — not the width-0 hairline the *filled* marker gets
   for contrast. The filled marker has a disc of colour behind that hairline and
   the hollow one has nothing, so one dashed device pixel of mid grey over a chart
   background is not visible at all: in the operator GUI test of 2026-09-15 a
   layer of no-data points read as the features having **disappeared**. A second
   channel that cannot be seen is not a second channel.

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
    per part of a multi-part feature, and every 1024 vertices inside a single
    ring — each coarser level bounds only the number of the next one down, and a
    file may be one feature, of one part, of millions of vertices — so the
    destructor's join is bounded by the abort and not by the size of the file
    (camp#213). And the file is read only as far as
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

    What is bounded here is abort latency and the geometry COUNT. Neither the
    vertex count of a single ring (100 M vertices is ~1.6 GB of coordinates) nor
    the size of a single attribute value is bounded, and both are deliberately
    left to one deferred decision rather than fixed mechanically: silently
    truncating a ring draws a WRONG shape, and silently truncating a property
    reports a wrong value — each worse than the honest geometry cap — so the
    options (drop and count it in `ParseDiagnostics`, or accept it and say so)
    are a decision this ADR does not yet make.

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
      with exactly this allowlist. The prefixes are asked of GDAL
      (`VSIGetFileSystemsPrefixes()`) rather than matched as the raw characters
      `/vsi`: that spelling also refused any ordinary local directory whose name
      begins with them (`/vsidata/survey.geojson`), and a prefix-*boundary* check
      does not help, since `/vsidata/` is still `/vsi<word>/`. The registered
      handler list is what GDAL itself dispatches on, so the refusal now tracks
      GDAL's real behaviour instead of approximating it. The restore path is the one that matters:
      `restorePersistedVectorLayers()` reopens every persisted entry at startup
      with no operator present, so a `/vsi` entry is dropped from the list
      rather than remembered.
    - **Non-file drivers are excluded by the allowlist** — `PG:`, `MySQL:`,
      `WFS:`, `OAPIF:` and the rest are not on it, so a connection string
      cannot open a database or a service from the load worker. That, and not
      the network, is what the list buys. Adding a format is a deliberate edit
      to it.

    **Measured, not assumed — `NetworkLink` is not followed on open:**
    `KML`/`LIBKML` are on the allowlist (operators are handed KML routinely) and
    a KML file can carry a `NetworkLink` pointing anywhere. An earlier draft of
    this ADR said the driver "may follow" one during `GDALOpenEx`; that was
    written from the format's capabilities rather than from the software, and it
    is not what either driver does. Tested against GDAL 3.8.4, the version CAMP
    builds on, with a `NetworkLink` carrying a relative local `href` and again
    with an `http://127.0.0.1:9/` one: both drivers return only the local
    placemark, immediately, with no fetch attempted. There is no content-initiated
    fetch to report here.

    The drivers stay named in this section because the claim is about a *version*,
    not about the format: a future GDAL could resolve `NetworkLink` hrefs on open,
    and whoever bumps the GDAL CAMP builds against should re-measure rather than
    re-derive. Should it ever become true, disabling network access at the GDAL
    configuration level (`GDAL_HTTP_*`/`CPL_VSIL_CURL_*`) is the lever that closes
    it without dropping the KML formats.

13. **Polar vertices are clamped to the projection's limit, not dropped.**
    `isPlaceable()` admits latitude ±90 — a perfectly valid WGS84 coordinate —
    where Web Mercator does not converge: `geoToMap()` stays finite only because
    `tan(π/2)` is 1.633e16 rather than inf in double, and yields y ≈ ±2.425e8 m,
    about twelve times the world half-extent of 2.004e7 m. One such vertex poisons
    the layer's `childrenBoundingRect()`, fit-to-extent and the scene index exactly
    as a `.prj`-less shapefile's eastings do. Every geometry conversion in
    `vector_feature_item.cpp` therefore goes through `placeableToMap()`, which
    clamps to `web_mercator::maximum_latitude` (85.0511°) first — the same
    truncation the tile schemes make. Clamping rather than dropping is deliberate:
    a polar survey line is real data this program should be able to show.

14. **Colour and size ramps are NUMERIC-ONLY in this MVP, and the styling menus
    offer only the fields a ramp can read.** `VectorLayer::numericFields()` — the
    subset of `fields()` for which at least one feature holds a finite numeric
    value, asked through the same `numericAttribute()` the ramp itself reads
    values through — is what the *Color by* and *Size by* menus list, plus two
    entries `numericFields()` never contributes: "(none)" to clear the field,
    and, when the persisted field below is stale, `<field> (no numbers)` so
    that setting is still visible and reachable from the menu rather than
    stuck.

    Offering every field made a free-text field selectable, and a ramp cannot read
    one: the field range came back invalid, *every* feature was marked no-data,
    and the whole layer went hollow grey. That is what the operator met on
    2026-09-15 colouring 7 magnetic-anomaly candidates by a free-text
    `assessment` field, and it is not a presentation defect — it is the ramp being
    asked a question it has no answer to. **Categorical styling is a different
    mapping** (a distinct colour per class, a legend, a stable class order), not a
    degenerate case of a continuous ramp, and it is a follow-on rather than
    something to approximate here.

    `fields()` is deliberately unchanged and still reports every field: the
    hover popup shows them all, and label-by-field — the other follow-on — will
    want them all too.

    A **persisted** `color_field_` can still name a field no feature has a number
    for, since the style group is keyed on the file path and restored whenever
    that path is reopened while the file on disk may have changed. `applyStyle()`
    treats an invalid colour range as **no colour field at all** for that pass —
    default colour, nothing flagged — rather than marking the entire layer
    no-data: no-data means "this feature lacks a value its neighbours have", and
    with no range there is nothing for anything to be missing from. (Size-by-field
    already behaved this way: `radiusForValue()` returns the default radius on an
    invalid range.)

15. **A point's HIT target is wider than its drawn marker**, by
    `kPointHoverSlackPixels` (4 device pixels) in `shape()`, with `boundingRect()`
    grown to match because a shape outside the bounding rect is undefined in Qt.
    The drawn marker is 5 pixels at the default size, and CAMP idles in pan mode —
    where the cursor is an **open hand whose hotspot the operator cannot see**. In
    the 2026-09-15 GUI test nobody landed the cursor inside 5 pixels, and the
    headline feature read as not working. This is the same trade a line already
    makes (its shape is stroked to `kHoverWidth`, wider than the drawn stroke): the
    target is what the cursor can be placed on, not what the renderer draws.
    Nothing about the symbol grows.

    The slack was added for click-to-inspect and carries over to hover-to-inspect
    unchanged, because `shape()` is what the scene hit-tests to dispatch hover
    events too — the constants are named for hover now, and the tolerance is the
    same. D16 attacks the same problem from the other end (the cursor now has a
    visible hotspot); the slack stays, because aim is never exact.

16. **In pan mode the cursor is an ARROW, CAMP-wide**, not `ScrollHandDrag`'s
    open hand. `ProjectView::setPanMode()` sets `Qt::ArrowCursor` on the
    **viewport** after `setDragMode(ScrollHandDrag)`, and `mouseReleaseEvent()`
    sets it again after `QGraphicsView::mouseReleaseEvent()` returns from a
    **left-button** release — the only button `ScrollHandDrag` pans with — because
    Qt installs the open hand at both of those points. The reset is scoped to the
    left button deliberately: a middle-button release is the measuring tool's and
    a right-button release opens the context menu, neither a pan, so a blanket
    reset would clobber a cursor either of those set for itself. The closed hand
    during an actual drag is left alone — there it is feedback about what is
    happening, not something being aimed. The add-\* modes keep their
    `Qt::CrossCursor`.

    This is an **operator decision** taken with D5's label in front of him
    (2026-09-15), and it is deliberately not scoped to this layer: the open hand's
    hotspot is invisible, so everything in CAMP that answers the cursor — a vessel
    or AIS hover label, a mission item's highlight, and now a vector feature — is
    aimed at blind, and CAMP idles in pan mode. It was recorded as a CAMP-wide
    follow-on in the previous revision of this ADR and then decided rather than
    deferred, because the change is four lines in one file.

    The view-vs-viewport split is load-bearing. Cross-cursor modes call
    `setCursor()` on the **view**, which a viewport with no cursor of its own
    inherits; leaving pan mode calls `setDragMode(NoDrag)`, and Qt unsets the
    viewport's cursor there, so the inheritance resumes. Setting the arrow on the
    view instead would be overridden by Qt's own viewport cursor.

    `ProjectView` is not constructible in a test harness (it needs the
    application's status bar and project — the same reason `test_mission_insertion`
    exercises the model rather than the view), so this decision carries **no
    automated test**. It is verified in the GUI.

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
- **A field CAMP cannot ramp is invisible in the styling menus** (D14), so an
  operator whose file carries only text attributes gets no *Color by* menu at all
  rather than one that does nothing useful. The hover popup still shows every
  attribute, which is where those fields are readable today. Categorical styling
  and label-by-field are the follow-ons that give them a rendering role.
- **The pan cursor changed for the whole application** (D16), which is the one
  thing in this branch an operator meets outside the vector layer: pan mode shows
  an arrow instead of an open hand. It is a deliberate CAMP-wide change, made
  because the open hand's hotspot is invisible and every hover-answering item in
  CAMP suffers for it, and it carries no automated test because `ProjectView`
  cannot be built in a harness.
- **Hover events are now accepted on every feature item** (D5), where the tooltip
  version needed none. That is per-item hover tracking across up to 50 000 items —
  the scene's ordinary dispatch, but not free. The label item itself is created
  lazily on first hover so the load path does not pay for it.
- **camp#225 — a pan that starts on a feature does not pan the map — is FIXED BY
  CONSTRUCTION.** It was the cost of the click version: the item accepted the left
  press so the release could tell a click from a drag, and that press therefore
  never reached `QGraphicsView`'s ScrollHandDrag. A feature now accepts no mouse
  button (D5), so there is no press to take and nothing to gate. Pinned by
  `VectorLayerInteraction.APressOverAFeatureFallsThroughToTheView` (no mouse
  grabber after a press through a real view) and
  `VectorFeatureItem.AcceptsNoMouseButtonSoThePressReachesTheView`.
- **Cross-antimeridian geometry is a known limitation.** A line or polygon whose
  vertices straddle 180° is drawn the long way round the world and stretches the
  layer's extent with it. The parser reports what the file says; nothing splits
  geometry at the seam. It has not been made to fail loudly because a correct
  split is real work, and the drawing is visibly wrong rather than plausibly
  wrong.
- **Label-by-field and a style-editing dialog are deliberately absent.** The
  context menu offers colour-by, size-by and palette; anything richer is a
  follow-up, per the issue's own MVP scope.
- **Named follow-ons, not filed as issues here** (the host files them; this list
  is what they are):
  - **Categorical styling** (a colour per class with a legend and a stable class
    order) and **label-by-field** — D14's two, and what a free-text attribute
    like `assessment` actually wants.
  - **Click to pin the popup open.** Hover answers "what is this"; an operator
    comparing two features, or copying a value, wants the popup to stay. It has
    to be built without taking the press away from the view's pan gesture, which
    is what D5 just bought back.
