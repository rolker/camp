# Plan: Import generic vector data

## Issue

https://github.com/rolker/camp/issues/22

## Revision history

**Rev 5** (2026-09-14) — pre-push review round 1 returned changes-requested with
ten must-fixes; these are the plan-level consequences. The code-level fixes are
in the branch's commits and the `## Implementation` entry in `progress.md`.

- **Rev 4's "no `onRemovedFromMap()` override" deferral is REVERSED** — it rested
  on an inaccurate premise and was must-fix 1. Rev 4 argued the override would
  signal "nothing the model does not already signal". The model signals something
  *different*: `Map::setMapItemParent()` implements a drag-REORDER as
  `beginRemoveRows` + `beginInsertRows` (`map.cpp:285-307`), so
  `rowsAboutToBeRemoved` fires for a reorder too and cannot be told apart from a
  removal. Dragging a vector layer up or down the Layers tab therefore erased its
  file from `vectorLayers/files`, and it did not come back on the next launch.
  `VectorLayer::onRemovedFromMap()` now exists and emits `removedFromMap()`;
  `AutonomousVehicleProject` connects to that per layer instead of to the model.
  The "one owner for the key" decision (Plan Review round 2) is UNCHANGED —
  `persistVectorLayers()` is still the single writer; the override writes nothing.
  What changed is *which signal* the owner reacts to.
- **A new ADR** (`docs/decisions/0016-read-only-vector-file-layer.md`, must-fix
  10): the layer family's design decisions and the persisted schema. Rev 4 had no
  ADR step; every predecessor layer family has one.
- **Two new bounds, both reported** (must-fixes 6 and 7):
  `parseVectorLayers()` takes a `ParseOptions::aborted` predicate polled per
  feature, so the destructor's join is bounded by the abort rather than by the
  size of the file; and item construction is capped at
  `VectorLayer::kMaxFeatureItems`, since it can only run on the GUI thread.
- **New test file** `test/test_vector_feature_item.cpp` — line hit-testing,
  coordinate placeability, and the click/drag gating.

**Rev 4** (2026-09-14) — implementation notes, edited inline as the work landed
(plan-first workflow). Scope is unchanged; these are the points where the
implementation differs from rev 3's letter, each with its reason:

- ~~**No `VectorLayer::onRemovedFromMap()` override**~~ — **reversed in rev 5
  above; the premise was wrong.** (Kept here so the reasoning that failed stays
  legible: the argument was that an override would be dead code because
  `AutonomousVehicleProject::onVectorLayerRemoved`, connected to the Map model's
  `rowsAboutToBeRemoved`, already covered removal. It did not: that signal also
  fires for a drag-reorder.)
- **No `ProjectView` change** (step 4; Open Question 2 resolved). The feature
  item reads pan mode from the view's own `dragMode()`
  (`QGraphicsView::ScrollHandDrag`), which `ProjectView::setPanMode()` sets
  (`projectview.cpp:335`) and all six add-* modes clear to `NoDrag`
  (`projectview.cpp:273-314`). No accessor is needed — and a camp_map item could
  not call into `ProjectView` (executable) anyway, the same layering rule that
  moved the parser. `src/camp/projectview.{h,cpp}` drops out of Files to Change.
- **New `src/camp_map/vector/vector_style.{h,cpp}`** — the colour/size mapping
  as free functions, which is what step 5's tests require ("extract the
  normalize+sample logic into small free functions"). Added to Files to Change.
- **The lat/lon-order fix landed in this PR** (step 1; Open Question 3 resolved).
  Confirmed only the UNTRANSFORMED branch was wrong (`readRing` built
  `QGeoCoordinate(getX(), getY())` where the Point path built `(getY(), getX())`);
  both now go through one `toWgs84()` helper. One helper in a file this step
  already rewrote, so no follow-up issue.
- **`boundingRect()` override IS needed** (step 3's "confirm during
  implementation"): `MapItem::boundingRect()` returns an empty rect
  (`map_item.cpp:34-37`), so the layer returns `childrenBoundingRect()`.
- **Persistence-test harness** (step 8): the `vectorLayers/files` mechanism
  (key, read/write, add-dedup, remove) lives in `camp_map` so the rules are
  testable — `AutonomousVehicleProject` is no more constructible in a test than
  it was for step 2's seam. The project stays the single *owner*, rebuilding the
  whole list from `m_vectorLayers` on both paths; its two call sites are verified
  by reading the source, as step 2's are.
- **Step 9 acceptance, partially automated**: both real datasets were driven
  through the shipped parse + styling path off-GUI (33 and 7 features; the
  58.07 nT/m peak normalizes to 1.0; candidate C reads back at 307792 E /
  4762878 N UTM 19N — confirmed with `gdaltransform` — carrying its `assessment`
  text). What remains genuinely manual is the GUI half: rendering, the click
  popup, and the remove-then-restart check.

**Rev 3** (2026-09-14) — responds to Plan Review round 2 (verdict
changes-requested, 1 must-fix + 2 suggestions; all 9 round-1 findings
verified resolved). Operator chose "patch the plan, then implement" — no third
review round; the pre-push code review gates the code.
- Must-fix: the step-2 regression test had no harness (no test constructs
  `AutonomousVehicleProject`). Step 2 now extracts a `resolveInsertionParent`
  seam into its own TU with a narrow gtest, and names the fallback.
- Suggestion: `vectorLayers/files` has one writer (`persistVectorLayers()`).
- Suggestion: `openGeometry`'s parent is a `nullptr` sentinel, not a default
  argument, and the `RowInserter` is routed through the same resolved parent.


**Rev 2** (2026-09-14, this revision) — responds to the Plan Review
(`.agent/work-plans/issue-22/progress.md`, verdict changes-requested, 9
must-fix + 3 suggestion findings). Every finding was re-verified against the
current source before being folded in:

- Step 2 ("fix `VectorDataset::read()`") is **replaced**. The premise was
  false: a persisted `VectorDataset` node **is** restored today —
  `MissionItem::readChildren` (`missionitem.cpp:185`) special-cases
  `object["type"] == "VectorDataset"` and dispatches to
  `AutonomousVehicleProject::openGeometry(fname)`
  (`autonomousvehicleproject.cpp:200-214`), which opens the file. `read()` is
  never called on that path, so restoring it would be dead code (and risk a
  double-load if a `VectorDataset` item were ever also constructed there).
  The real defect, confirmed by reading both functions: `openGeometry`
  always inserts the new `VectorDataset` under `m_currentGroup` (a
  project-global "current group" pointer), not under the `MissionItem` whose
  `readChildren` is doing the dispatching — so a `VectorDataset` nested
  inside a `Group` is restored at the top level (or wherever
  `m_currentGroup` happens to point), not back where the user put it. The
  operator confirmed this re-scoping (decision 1 on the issue, 2026-09-14):
  fix `openGeometry`'s insertion target instead, with a regression test, and
  drop all `read()`-stub work.
- The consequences-table row claiming "previously-empty `VectorDataset`
  nodes now populate on reload" is removed — it described the same false
  premise and must not reach the PR description.
- `vector_parse.{h,cpp}` now moves into `camp_map` (new
  `src/camp_map/vector/` directory) instead of staying in the executable's
  sources, so `VectorLayer` (which lives in `camp_map`) can call it — per
  `.agents/README.md`'s "a library cannot call into the executable that
  links it" rule (the `camp_crash` lesson, #217; verified at
  `CMakeLists.txt:135` where `vector_parse.cpp` currently compiles only into
  `CCOMAutonomousMissionPlanner`'s `SOURCES`, and `CMakeLists.txt:295-339`
  where `camp_map` is a separate `SHARED` library with no link back to the
  executable).
- Step 3 (now step 2) adds the mandatory
  `camp_crash::install_thread_alt_stack()` first statement in the load
  worker, per the `check_worker_alt_stacks` CTest guard
  (`CMakeLists.txt:554-557`, pattern confirmed at `raster_layer.cpp:189`).
- Persistence (step 6, now step 7) adds the removal half: an
  `onRemovedFromMap()` override plus `Map`-model `rowsAboutToBeRemoved`
  bookkeeping on `AutonomousVehicleProject`, mirroring
  `RasterLayer::onRemovedFromMap()` (`raster_layer.cpp:749-757`) and
  `AutonomousVehicleProject::onChartLayerRemoved`
  (`autonomousvehicleproject.cpp:322+`) — confirmed these exist and are
  wired to the Map model precisely so a Layers-tab removal survives a
  restart (camp#90/#117). Persistence mechanism is now settled (operator
  decision 2): QSettings app state, the same `persistBackgrounds()` /
  `restorePersistedBackgrounds()` pattern, not the mission project file —
  the prior Open Question is resolved, not deferred to review-plan.
- Styling (step 5) now specifies degenerate/absent-value handling: a
  same-value field, a missing field, and a non-numeric/NaN value each get a
  defined behavior, mirroring `grid_map.cpp:198-206`'s
  `min_value == max_value` guard.
- The parser (step 1) now explicitly handles multi-part geometries
  (`wkbMultiPoint`/`wkbMultiLineString`/`wkbMultiPolygon`) and every 25D/ZM
  variant via `wkbFlatten()`, instead of silently dropping them — confirmed
  `vector_parse.cpp:86-140` currently `switch`es on the raw
  `getGeometryType()` with only `wkbPoint`/`wkbLineString`/`wkbPolygon`
  cases and a `default: break` that drops everything else, including a
  GeoJSON point with elevation (`wkbPoint25D`).
- Click-to-inspect (step 4) is now specified against `ProjectView`'s actual
  mouse-mode state machine (`projectview.cpp:57-196`), confirmed to run
  waypoint/trackline/survey placement on left-press in its add-* modes and
  set `ScrollHandDrag` (`projectview.cpp:335`) in pan mode, unconditionally
  forwarding to `QGraphicsView::mousePressEvent()` afterward (so child
  `QGraphicsItem`s under the cursor still receive press events regardless
  of mode).
- Files to Change now lists `item_types.h` (new `VectorLayerType` enum
  value — confirmed every `map::Layer` descendant declares one at
  `item_types.h:22-45`), the `mainwindow.cpp:161` startup restore call
  (confirmed `restorePersistedBackgrounds()` is called there and
  `restorePersistedVectorLayers()` needs the same site), and an
  `.agents/README.md` note.
- Suggestions folded in: per-layer settings key now overrides
  `settingsKey()` rather than relying on the `itemID()` default (camp#126
  precedent, confirmed at `gggs_tile_layer.h:72` /
  `gggs_tile_layer.cpp:1829-1852`, since `itemID()` is basename-derived —
  `map_item.cpp:53-61` — and two same-named files in different directories
  would otherwise collide); the pre-existing lat/lon-order inconsistency
  between the Point path (`op->getY(), op->getX()`,
  `vector_parse.cpp:96-104`) and `readRing` (`getX(), getY()`,
  `vector_parse.cpp:46`) is flagged for a one-line fix-or-confirm while
  step 1 is already editing that file, since the acceptance test is a
  coordinate check; persistence mechanism (previously an Open Question) is
  now a decided item, not a question.

**Rev 1** (2026-09-14) — initial plan, superseded above.

## Context

`VectorDataset` (`src/camp/vector/vectordataset.{h,cpp}`) already opens any
OGR-readable file via `camp::vector::parseVectorLayers` and builds an
editable `Group`/`Point`/`LineString`/`Polygon` tree in the **mission**
model. That parser is leak-clean (#152) and unit-tested
(`test/test_vector_dataset_cleanup.cpp`), but `ParsedGeometry`/`ParsedLayer`
carry no attributes today, and — the real, verified defect —
`AutonomousVehicleProject::openGeometry()` always inserts the reopened
`VectorDataset` under `m_currentGroup` rather than under the `MissionItem`
whose `readChildren` is restoring it, so a nested `VectorDataset` moves on
reload.

This issue asks for a second, **read-only** way to view the same kind of
file: a `map::Layer` in the Layers tab (like `RasterLayer`/backgrounds,
ADR-0002/ADR-0003) that renders points/lines/polygons with attribute-driven
styling (colour-by-field via `marine_colormap`, size-by-field) and
click-to-inspect.

The issue review raised six open items; the operator answered all six in a
follow-up comment on the issue (2026-09-14), and — after Plan Review rev 1
found the `read()` premise false — made a further decision (2026-09-14,
below) re-scoping the persistence-defect fix. This plan implements both
rounds of answers directly:

1. Extend `vector_parse`'s `ParsedGeometry`/`ParsedLayer` with feature
   attributes and reuse it — do not re-derive OGR iteration, and do not
   touch `VectorDataset`'s existing editable-import behavior.
2. Attributes are in scope, carried by the extended parser.
3. **(Revised per Plan Review + operator decision, 2026-09-14)** Fix the
   real defect — `AutonomousVehicleProject::openGeometry()` inserts a
   restored `VectorDataset` under `m_currentGroup` instead of under its
   original parent node — with a regression test. The `read()` stub is
   unrelated dead code on this path and is left alone.
4. Architecture: per-feature `QGraphicsItem` children under the new layer
   (for free hit-testing / click-to-inspect), transformed to scene
   coordinates once at load — not a single painted surface like
   `RasterLayer`.
5. Colour-by-field uses `marine_colormap` (already integrated in
   `RasterLayer` and `ros/grids/grid_map.cpp`), not the deleted
   `camp::map::ColorMap` / stale #63.
6. MVP scope for this PR: OGR load (GeoJSON first), points/lines/polygons
   (including multi-part and 25D variants — see step 1) with a default
   style, colour-by-field, size-by-field, click-to-inspect attributes,
   save/restore in the project (QSettings app state, per operator decision
   2 below), unit tests for the parser attributes and the styling mapping.
   Label-by-field and a style-editing UI are follow-on (new issues, not this
   PR).

**Operator decision (2026-09-14, responding to Plan Review rev 1):**

1. Replace step 2 with the `openGeometry()` insertion-target fix described
   above; drop the `read()` work and the wrong consequences row.
2. Persistence of the new layer's file list uses the app-state QSettings
   pattern that background rasters use (ADR-0003 §4,
   `persistBackgrounds`/`restorePersistedBackgrounds`, restored from
   `mainwindow.cpp` at startup), **including the removal half**
   (`onRemovedFromMap()` + model `rowsAboutToBeRemoved` bookkeeping, camp#90
   /#117 precedent) — not the mission file.

## Approach

1. **Move `vector_parse` into `camp_map`, and extend it with attributes and
   full geometry coverage.**
   - Relocate `src/camp/vector/vector_parse.{h,cpp}` to a new
     `src/camp_map/vector/vector_parse.{h,cpp}` (pure Qt/GDAL, no ROS — fits
     `camp_map`'s existing ROS-free boundary, confirmed at
     `CMakeLists.txt:295-339`). Update `VectorDataset`'s include and the
     `CMakeLists.txt` source lists (remove from the executable's `SOURCES`,
     add to `CAMP_MAP_SOURCES`) and the existing
     `test_vector_dataset_cleanup.cpp` include path. This is the only way
     the new `VectorLayer` (which must live in `camp_map` — see step 2) can
     call `parseVectorLayers`, per `.agents/README.md`'s
     library-cannot-call-executable rule (#217).
   - Add `QMap<QString, QVariant> attributes` to `ParsedGeometry`. In
     `parseVectorLayers`, read every field from `feature`'s
     `OGRFeatureDefn` (`GetFieldCount()`/`GetFieldDefnRef(i)`/
     `GetFieldAsString`/`GetFieldAsDouble`/`GetFieldAsInteger` keyed by
     `OGRFieldType`) into that map before `OGRFeature::DestroyFeature
     (feature)`. `VectorDataset::buildItems` ignores the new field (source
     compatible; no behavior change for the editable import path).
   - Replace the `switch(geometry->getGeometryType())` in
     `parseVectorLayers` (`vector_parse.cpp:86-140`) with a switch on
     `wkbFlatten(geometry->getGeometryType())` so every 25D/ZM/M variant
     (`wkbPoint25D`, `wkbLineStringZM`, etc.) reaches the same case as its
     2D form. Add `wkbMultiPoint`/`wkbMultiLineString`/`wkbMultiPolygon`
     cases that iterate the multi-geometry's parts (`OGRGeometryCollection::
     getGeometryRef(i)`) and emit one `ParsedGeometry` per part, reusing the
     existing Point/LineString/Polygon extraction logic (factor each into a
     small free function so the multi-* cases can call it per part without
     duplicating the transform/ring logic). Anything still unhandled
     (`wkbGeometryCollection`, curves) stays a documented, logged skip — not
     a silent one — since the issue's format claims (shapefile/GeoPackage/
     KML) are then honestly met for the geometry types those formats
     actually emit.
   - While in this file: confirm and, if confirmed, fix the pre-existing
     lat/lon-order inconsistency between the Point path (`op->getY(),
     op->getX()` at `vector_parse.cpp:96-104`) and `readRing` (`getX(),
     getY()` at `vector_parse.cpp:46`) — untransformed line/polygon vertices
     currently come out lat/lon-swapped relative to points. Fix in this PR
     (small, in a file this step already touches) or file a dedicated
     follow-up issue and note the decision here; do not leave it
     unacknowledged, since the acceptance test in step 8 is a coordinate
     check.

2. **Fix the real `openGeometry` persistence defect.**
   `AutonomousVehicleProject::openGeometry(fname, label)`
   (`autonomousvehicleproject.cpp:200-214`) always does
   `vd = new VectorDataset(m_currentGroup)`, ignoring which `MissionItem`
   node originally contained it. `MissionItem::readChildren`
   (`missionitem.cpp:185`) calls `project->openGeometry(...)` for a
   `type == "VectorDataset"` child without passing `this` as the intended
   parent. Fix, in three parts so the regression test has a seam that
   builds (Plan Review round 2 must-fix — no existing test constructs
   `AutonomousVehicleProject`, whose TU pulls in the whole mission tree):
   - **Seam**: a free function
     `MissionItem* resolveInsertionParent(MissionItem* requested, MissionItem* currentGroup)`
     in a small new TU `src/camp/mission_insertion.{h,cpp}` — returns
     `requested` when non-null, else `currentGroup`. Header depends only on a
     forward-declared `MissionItem`, so a narrow gtest can compile it
     without the project class.
   - **Call sites**: `openGeometry` gains a `MissionItem* parent = nullptr`
     parameter (a `nullptr` sentinel — "default to `m_currentGroup`" is not
     expressible as a default argument since `m_currentGroup` is a member).
     It resolves `parent` through the seam **once** and routes both the
     `RowInserter` and the `new VectorDataset(...)` through that resolved
     parent. `readChildren` passes `this`; the existing "Import" menu-action
     caller passes nothing and keeps its behaviour.
   - **Regression test** `test/test_mission_insertion.cpp`: (a) unit-test the
     seam — `requested` wins when set, `currentGroup` when it is null —
     compiling only `mission_insertion.cpp` + stubs (same narrow-source-set
     pattern as the other `ament_add_gtest` blocks); (b) a parse-level check
     that `MissionItem::readChildren`'s `VectorDataset` branch is the only
     `openGeometry` call that passes a parent is covered by review, not a
     test — the project class is not constructible in the test harness.
   Fallback, if the seam proves unnecessary during implementation (e.g. the
   resolution collapses to one expression): keep the narrow test anyway; the
   operator required a regression test for this fix.
   This lands in this PR as the real defect fix (operator decision 1); no
   `read()`/`write()` change is needed since that path is dead code for the
   persisted case.

3. **Add a `camp::vector::VectorLayer` class** under
   `src/camp_map/vector/` (mirrors `src/camp_map/raster/` for
   `RasterLayer`), deriving from `map::Layer`:
   - Constructor takes `(map::MapItem* parent, const QString& filename)`,
     mirrors `RasterLayer`'s shape: open the OGR dataset off the GUI thread
     (`QFutureWatcher`, abort flag + mutex — the `RasterLayer`/#213
     join-in-destructor pattern) via `parseVectorLayers`, reproject each
     `ParsedGeometry`'s WGS84 coordinates to the Web-Mercator scene **once**
     on load completion, and build one child `QGraphicsItem` per feature.
   - The load worker's **first statement** must be
     `camp_crash::install_thread_alt_stack()`, matching
     `raster_layer.cpp:189` — required by the `check_worker_alt_stacks`
     CTest guard (`CMakeLists.txt:554-557`), which fails CI for any
     `QtConcurrent::run()` entry point that omits it.
   - `int type() const override` returns a new `VectorLayerType` value
     added to `camp::map::ItemType` (`item_types.h:22-45` — every
     `map::Layer` descendant declares one; see Files to Change).
   - `boundingRect()` covers the union of child extents (confirm during
     implementation whether an explicit override is needed given
     `MapItem::boundingRect()`'s current implementation, which already
     unions children for hit-testing/painting purposes).
   - Destructor joins the load worker before teardown (the #213 pattern
     already used by `ros/geometry/polygon.h` and `OccupancyGrid`).
   - `settingsKey()` override: do **not** rely on the `itemID()` default
     (basename-derived, `map_item.cpp:53-61`) — two vector files with the
     same basename in different directories would share one settings
     group. Follow the `GggsTileLayer::settingsKey()` precedent
     (`gggs_tile_layer.cpp:1829-1852`, camp#126) and key on the full
     filename path instead.
   - `writeSettings()`/`readSettings()` overrides (QSettings, keyed by
     `settingsKey()`) persist per-layer style (`colorField`, `sizeField`,
     `colormap` name) — see step 7 for the file-list half.
   - `onRemovedFromMap()` override (rev 5): emit `removedFromMap()` so the
     project can drop this file from the persisted list. It writes no key
     itself — `persistVectorLayers()` is the single writer — but it is the
     only reorder-SAFE removal signal: the Map model's `rowsAboutToBeRemoved`
     fires for a drag-reorder too. See step 7 and ADR-0016 D9.

4. **Add feature child-item classes** (new file,
   `src/camp_map/vector/vector_feature_item.{h,cpp}`): lightweight
   `QGraphicsItem` (not `MapItem`/`QGraphicsObject` — no need for the tree
   model, settings, or signal/slot machinery per feature) for Point,
   LineString, and Polygon geometry (including multi-part instances from
   step 1, each rendered as its own child item), each holding a
   `const ParsedGeometry*` (or a copy) and its resolved paint color/size.
   Distinct from `Point`/`LineString`/`Polygon` (`src/camp/*.h`) — those are
   `MissionItem` subclasses with editing/drag/waypoint-linking baggage a
   read-only layer must not inherit.
   - Default style: point = filled circle (fixed radius unless
     size-by-field is set), line = stroked path, polygon = filled+stroked
     path with exterior/interior rings (even-odd fill rule for holes).
   - **Click-to-inspect, specified against `ProjectView`'s actual mouse
     state machine** (`projectview.cpp:57-196`): `ProjectView::
     mousePressEvent` runs its own placement logic for left-press only in
     the add-waypoint/add-trackline/add-survey-pattern/add-survey-area/
     add-search-pattern/add-avoid-area modes, does nothing extra in pan
     mode (`ScrollHandDrag`, set at `projectview.cpp:335`), and then
     **unconditionally** forwards to `QGraphicsView::mousePressEvent(event)`
     at the end regardless of mode — so a child item under the cursor
     receives the press either way. Behavior: accept `Qt::LeftButton`
     (`setAcceptedMouseButtons`) and show the feature's attributes via
     `QToolTip::showText(event->screenPos(), text)` **only in pan mode**;
     in every add-* mode, `event->ignore()` in the feature item so the
     event is not marked accepted at the item level and `ProjectView`'s own
     placement logic (which reads `event->pos()`/`mapToScene`, not item
     acceptance) is unaffected either way, and the popup does not fire
     mid-placement. Confirm during implementation whether `ProjectView`
     needs a `mouseMode` accessor for the feature item to consult (it
     currently reads a private `mouseMode` member — check for an existing
     accessor or add a minimal read-only one, since only pan mode should
     show the popup).
   - `boundingRect()`/`shape()` from the transformed geometry, in the
     parent layer's local (scene-mercator) coordinates.

5. **Attribute-driven styling on `VectorLayer`.**
   - `setColorField(const QString& field)` / `colorField()`: when set,
     resolve each feature's numeric value for that field, compute the
     field's min/max **only over features that have a present, numeric,
     non-NaN value for it** — mirroring `grid_map.cpp:198-206`'s
     `min_value == max_value` guard — at load, and sample a
     `marine_colormap::Palette` (`find_palette(name)`, default `"viridis"`,
     `palette->sample(normalized_value)` → `Rgba8` → `to_rgba8`/`QColor`)
     per feature. Degenerate/absent-value handling, specified explicitly
     (missing from rev 1):
     - **All present values equal** (`min == max`): treat as `grid_map.cpp`
       does — offset `min` by a small epsilon so every feature normalizes
       to 1.0 (top of the ramp) instead of dividing by zero, rather than
       falling back to the default style.
     - **Field missing on a feature, or present but non-numeric/NaN**:
       paint that feature in a fixed neutral/"no data" color (documented
       constant, distinct from any position on the active palette) so it
       is visually distinguishable from a real low-end value — never
       silently reuses palette index 0.
     - No field set → the existing per-layer default style color.
   - `setSizeField(const QString& field)`: same min/max normalization and
     the same degenerate/missing/non-numeric handling as colour-by-field
     (equal-value epsilon; missing/non-numeric → a fixed default radius,
     not a computed one), linear interpolation between a fixed min/max
     marker radius (e.g. 3–15 px) for points; lines/polygons ignore
     size-by-field (documented in the header comment, not silently
     dropped).
   - `setColormap(const std::string& name)`: mirrors
     `RasterLayer::setColormap` for consistency, applies to colour-by-field
     only.
   - Recompute per-feature paint properties when a style setter changes
     (single pass over already-loaded features — no re-parse, no re-load).

6. **Menu/action wiring.** Add an `AutonomousVehicleProject::
   openVectorLayer(fname)` entry point (parallel to `addBackgroundLayer`),
   wired from a new "Open vector layer" action alongside the existing "Open
   background" action (confirm the exact wiring site in `mainwindow.cpp`
   during implementation — same file as the step-7 startup-restore call).

7. **Persistence (operator decision 2 — QSettings app state, both halves).**
   Per ADR-0003 §4 and the operator's explicit confirmation, `VectorLayer`
   persists exactly like `RasterLayer`/backgrounds — app state, not the
   mission project file:
   - **Restore half**: a new `m_vectorLayers` list +
     `persistVectorLayers()`/`restorePersistedVectorLayers()` pair on
     `AutonomousVehicleProject`, mirroring `persistBackgrounds()`/
     `restorePersistedBackgrounds()` (`autonomousvehicleproject.cpp:
     291-320`) — QSettings key `vectorLayers/files`, de-dup by filename,
     self-heal on restore (rev 5: re-persist unconditionally, which collapses
     duplicates and normalises path spellings; a file that is not reachable
     right now is REMEMBERED and carried forward rather than dropped — an
     unmounted share is not the operator asking for a removal). Add the
     `project->restorePersistedVectorLayers();` call in `mainwindow.cpp`
     immediately alongside the existing `restorePersistedBackgrounds()`
     call at `mainwindow.cpp:161`.
   - **Removal half** (missing from rev 1 — confirmed
     `RasterLayer::onRemovedFromMap()` at `raster_layer.cpp:749-757` and
     `AutonomousVehicleProject::onChartLayerRemoved` at
     `autonomousvehicleproject.cpp:322+`, wired to the `Map` model's
     `rowsAboutToBeRemoved`, exist for exactly this reason, camp#90/#117):
     add `VectorLayer::onRemovedFromMap()` (signals removal; it does **not**
     write `vectorLayers/files` itself — Plan Review round 2 suggestion:
     the raster precedent has two writers over *two different* keys, so the
     vector key gets **one owner**, `AutonomousVehicleProject::persistVectorLayers()`,
     called from both the add and the remove paths) and an
     `AutonomousVehicleProject::onVectorLayerRemoved` slot connected
     **(rev 5) to that per-layer signal, NOT to the Map model's
     `rowsAboutToBeRemoved`** — which `Map::setMapItemParent()` also fires for a
     drag-reorder, so reacting to it un-persisted a layer the operator only
     moved. (Drop the matching `m_vectorLayers` entry,
     re-persist). Without this, a vector layer removed from the Layers tab
     would silently reappear on next launch.
   - Per-layer style (`colorField`, `sizeField`, `colormap` name) persists
     via `VectorLayer`'s own `writeSettings()`/`readSettings()` override
     (step 3), keyed by the overridden `settingsKey()` — no new persistence
     plumbing needed beyond new keys under that group.

8. **Tests** (new `test/` files, wired into `CMakeLists.txt` next to the
   existing `ament_add_gtest` blocks for `test_vector_dataset_cleanup` /
   `test_raster_layer_gdal_cleanup` / `test_background_persistence`):
   - `test_vector_parse_attributes.cpp`: extend the existing
     GeoPackage-writer pattern from `test_vector_dataset_cleanup.cpp` with
     typed fields (string, int, real) on each feature; assert
     `ParsedGeometry::attributes` round trips the values and types. Also
     write a `MultiPolygon` (and one 25D `Point`) feature and assert each
     part/variant is parsed into a `ParsedGeometry`, not silently dropped.
     Keep the leak-check coverage from the existing test intact.
   - `test_vector_layer_styling.cpp`: headless unit test of the
     colour-by-field and size-by-field mapping functions (extract the
     normalize+sample logic into small free functions or static methods
     that don't need a `QApplication`/GUI thread, matching
     `test_color_map.cpp`'s headless style) — assert min/max feature values
     map to the palette's first/last LUT entries, mid-range values
     interpolate, an all-equal field maps every feature to the top of the
     ramp (not a divide-by-zero/NaN), and a feature missing the field (or
     holding a non-numeric/NaN value) gets the documented neutral color
     rather than palette index 0.
   - `test_vector_layer_teardown.cpp`: confirm the load worker is joined in
     the destructor before the dataset/parsed data it captured is freed —
     the #213 pattern, analogous to
     `test_raster_layer_gdal_cleanup.cpp`'s abort-on-destroy coverage.
   - `test_mission_insertion.cpp`: the step-2 seam test — narrow source
     set (`mission_insertion.cpp` only), asserts `resolveInsertionParent`
     returns the requested parent when given and the current group when
     not. (Rev 2's `test_open_geometry_nested_group.cpp` round-trip is
     dropped: nothing in `test/` can construct `AutonomousVehicleProject`.)
   - `test_vector_layer_persistence.cpp` (or extend
     `test_background_persistence.cpp`'s pattern in a new file scoped to
     vector layers): mirror its `RestoreExistingNoReseedAndDedup` and
     `RemoveDePersistsAndSticks` coverage for `vectorLayers/files` — add,
     restore, remove, confirm it stays removed across a second restore.
   - The `check_worker_alt_stacks` CTest guard (`CMakeLists.txt:554-557`)
     already runs against every `QtConcurrent::run()` entry point in the
     tree; no new test needed for step 3's alt-stack call, but confirm it
     passes locally before pushing.

9. **Manual acceptance** against the issue's two test datasets
   (`~/data/logs/analysis/2026-09-14_massabesic_mag/
   massabesic_mag_peaks.geojson`, `massabesic_joint_candidates.geojson`) —
   not automatable in this PR (real files outside the repo, GUI rendering),
   but recorded here so `review-code`/the PR description can check it off
   explicitly: load both, confirm colour-by-field on
   `analytic_signal_nT_per_m` puts the 58 nT/m peak at the top of the ramp,
   confirm candidate C's popup identifies the point at 307792 E /
   4762878 N (UTM 19N) with its `assessment` text, and confirm a vector
   layer removed via the Layers tab does **not** reappear after closing and
   reopening CAMP.

## Files to Change

| File | Change |
|------|--------|
| `src/camp_map/vector/vector_parse.h` (moved from `src/camp/vector/`) | Add `attributes` field to `ParsedGeometry`; document type-mapping rules |
| `src/camp_map/vector/vector_parse.cpp` (moved from `src/camp/vector/`) | Read OGR field values into `attributes`; `wkbFlatten` + multi-part geometry handling |
| `src/camp/vector/vectordataset.h`, `.cpp` | Update include path for the moved `vector_parse.h`; no behavior change |
| `src/camp/autonomousvehicleproject.h` | `openGeometry()` gains a parent-group parameter (defaulted); `m_vectorLayers`, `openVectorLayer()`, `persistVectorLayers()`/`restorePersistedVectorLayers()`, `onVectorLayerRemoved` declarations |
| `src/camp/autonomousvehicleproject.cpp` | `openGeometry()` inserts under the passed parent instead of always `m_currentGroup`; new vector-layer persistence + removal implementations mirroring `addBackgroundLayer`/`persistBackgrounds`/`restorePersistedBackgrounds`/`onChartLayerRemoved` |
| `src/camp/missionitem.cpp` | `readChildren`'s `VectorDataset` case passes `this` as the restore parent |
| `src/camp_map/map/item_types.h` | New `VectorLayerType` enum value |
| `src/camp_map/vector/vector_layer.h` (new) | `VectorLayer : public map::Layer` — async OGR load (with alt-stack install), style setters, `settingsKey()` override, `onRemovedFromMap()`, persistence hooks |
| `src/camp_map/vector/vector_layer.cpp` (new) | Load worker, scene-coordinate transform, style recompute (with degenerate/missing-value handling), readSettings/writeSettings |
| `src/camp_map/vector/vector_feature_item.h` (new) | Read-only per-feature `QGraphicsItem` (point/line/polygon), click-to-inspect gated on pan mode |
| `src/camp_map/vector/vector_feature_item.cpp` (new) | Paint + hit-test (rev 5: line shapes are STROKED, or Qt's fill-area test never picks them) + coordinate placeability + the attribute popup on release-without-drag |
| `docs/decisions/0016-read-only-vector-file-layer.md` (new, rev 5) | ADR for the layer family and the persisted schema |
| `test/test_vector_feature_item.cpp` (new, rev 5) | Line hit-testing, coordinate placeability, click/drag gating |
| `src/camp_map/vector/vector_style.h`/`.cpp` (new) | Colour/size-by-field mapping as free functions — the headless-testable seam step 5's tests need |
| ~~`src/camp/projectview.h`/`.cpp`~~ | **Not needed** (rev 4): the feature item reads pan mode from the view's `dragMode()` rather than asking ProjectView |
| `src/camp/mainwindow.cpp` | "Open vector layer" action wiring; `restorePersistedVectorLayers()` call alongside `restorePersistedBackgrounds()` at `mainwindow.cpp:161` |
| `CMakeLists.txt` | Move `vector_parse.cpp` from the executable's `SOURCES` to `CAMP_MAP_SOURCES`; add `vector_layer.cpp`/`vector_feature_item.cpp` to `CAMP_MAP_SOURCES`; 5 new `ament_add_gtest` blocks |
| `test/test_vector_parse_attributes.cpp` (new) | Attribute parse round trip + multi-part/25D geometry coverage |
| `test/test_vector_layer_styling.cpp` (new) | Colour/size-by-field mapping, including degenerate/missing-value cases |
| `test/test_vector_layer_teardown.cpp` (new) | Thread-safe teardown (#213 pattern) |
| `src/camp/mission_insertion.{h,cpp}` (new) | `resolveInsertionParent` seam (step 2) |
| `test/test_mission_insertion.cpp` (new) | step-2 seam regression test, narrow source set |
| `test/test_vector_layer_persistence.cpp` (new) | Add/restore/remove/stays-removed round trip for `vectorLayers/files` |
| `.agents/README.md` | Note `VectorLayer` (read-only display, `camp_map`) vs. `VectorDataset` (editable import, mission tree) as the two vector-file entry points |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Capture decisions, not just implementations | The architecture forks (reuse `vector_parse`; per-feature items vs. single surface; persistence mechanism) are operator-decided (issue comment + Plan Review response, 2026-09-14) and restated in Context/decision blocks above. No new ADR proposed — this plan applies existing ADR-0002/0003/0007/0008 precedent, not a new pattern. |
| A change includes its consequences | The real `openGeometry` defect (step 2) lands in this PR with a regression test. Persistence's removal half (step 7) is no longer missing. Library layering (step 1's move) is resolved before implementation, not discovered at link time. |
| Only what's needed | Label-by-field and style-editing UI stay deferred to follow-on issues (operator's MVP scope). Multi-part/25D geometry support is added because dropping it silently would misrepresent the issue's own format claims — not scope creep. |
| Test what breaks | Five new test files: attribute+geometry-coverage parsing, styling math (including degenerate cases), GDAL/thread teardown, the nested-parent persistence regression, and the layer-list persistence round trip (add/remove/stays-removed). |
| Improve incrementally | `VectorDataset` (editable import) is untouched behaviorally except the `openGeometry` parent fix; the new read-only layer is fully additive. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0002 (Web-Mercator scene/layer model) | Yes | Coordinates transformed to scene Web-Mercator once at load (step 3), matching the ADR's "transform once, let Qt scale" convention already used by `RasterLayer` and mission items. |
| ADR-0003 (backgrounds as layers/depth tree) | Yes | `VectorLayer` is an ordinary Layers-tab layer (§1 pattern); §4's split persistence (app-state for Layers-tab layers, project-file for mission items) is the confirmed basis for step 7's persistence design, now including the removal half. |
| ADR-0007 (RasterFieldSource render abstraction) | No | Not applicable — `VectorLayer` does not implement `RasterFieldSource`; per-feature `QGraphicsItem` children are the chosen shape (operator decision), not `RasterLayer`'s single-texture pattern. |
| ADR-0008 (marine_colormap LUT bake) | Yes | Colour-by-field uses `marine_colormap::find_palette`/`sample()` directly (step 5), the same facility `RasterLayer` and `grid_map.cpp` already use — not the deleted `camp::map::ColorMap`. |
| ADR-0011 (viewport-clip render convention) | No (for this PR) | The two acceptance datasets are 33 and 7 features; no viewport-scoped culling in this MVP. Noted as a follow-on if vector layers grow to survey-index-footprint scale. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `vector_parse.{h,cpp}` moves from the executable into `camp_map` | `VectorDataset`'s include path; `CMakeLists.txt` source lists (both the executable's `SOURCES` and `CAMP_MAP_SOURCES`); `test_vector_dataset_cleanup.cpp`'s include | Yes — step 1 and Files to Change |
| `ParsedGeometry`/`ParsedLayer` gain an `attributes` field and multi-part/25D handling | `VectorDataset::buildItems` (source-compatible, ignores the new field; unaffected by the geometry-type widening since it already only handles Point/LineString/Polygon) | Yes — verified no behavior change, noted in step 1 |
| A new `VectorLayer` type is added to the Layers tab | `item_types.h`'s `ItemType` enum; any place that maps a persisted layer "type" string to a class | Yes — `VectorLayer` persists via QSettings app state (step 7), not the mission JSON's type-string dispatch in `missionitem.cpp`, so no new case is needed there for the layer itself |
| `AutonomousVehicleProject::openGeometry()` gains a `MissionItem* parent = nullptr` parameter | The existing "Import" menu-action call site passes nothing and resolves to `m_currentGroup` through the seam, so its behavior is unchanged | Yes — step 2, nullptr sentinel + `resolveInsertionParent` |
| A vector layer can be removed from the Layers tab | Its entry in `vectorLayers/files` must be dropped, not just the in-memory item | Yes — step 7's removal half, previously missing |
| A new menu action opens vector layers | `.agents/README.md` if it documents the menu structure | Yes — `.agents/README.md` note added in this PR (Files to Change), since the plan itself proposes the distinction it should record |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `.agents/README.md` gains a short
  note distinguishing `VectorLayer` (read-only display, `camp_map`) from
  `VectorDataset` (editable import, mission tree) — the two now look
  confusingly similar by name, and this PR is the moment that distinction
  is introduced, so it is a stale-docs item (the change itself creates the
  gap), not a proposal. See Files to Change.
- **Agent-instruction candidates** (proposals only): None beyond the above
  — the library-layering rule (#217) and the persistence pattern (ADR-0003
  §4) are both already documented; this PR is an application of existing
  guidance, not a new pattern needing its own instruction entry.

## Open Questions

All three were settled during implementation (rev 4):

- [x] `ParsedGeometry::attributes` container type — **`QMap<QString, QVariant>`**.
      It also gives the click-to-inspect popup a stable alphabetical field order,
      so the ordered-vector alternative bought nothing.
- [x] Whether `ProjectView` needs a read-only `mouseMode` accessor — **no**.
      Pan mode is `QGraphicsView::ScrollHandDrag`, which the feature item reads
      from the view directly; every add-* mode sets `NoDrag`. ProjectView is
      untouched, which also keeps the camp_map layering rule intact.
- [x] Whether the Point-vs-`readRing` lat/lon-order inconsistency is fixed here
      or filed as a follow-up — **fixed in this PR**. Only the untransformed
      branch was wrong, and both branches now share one `toWgs84()` helper.

## Estimated Scope

Single PR (MVP per operator decision). Label-by-field and a style-editing
UI are explicitly out of scope, to be filed as follow-on issues after this
PR lands.
