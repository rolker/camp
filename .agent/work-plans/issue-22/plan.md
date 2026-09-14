# Plan: Import generic vector data

## Issue

https://github.com/rolker/camp/issues/22

## Context

`VectorDataset` (`src/camp/vector/vectordataset.{h,cpp}`) already opens any
OGR-readable file via `camp::vector::parseVectorLayers` (`src/camp/vector/
vector_parse.{h,cpp}`) and builds an editable `Group`/`Point`/`LineString`/
`Polygon` tree in the **mission** model. That parser is already leak-clean
(#152) and unit-tested (`test/test_vector_dataset_cleanup.cpp`), but
`ParsedGeometry`/`ParsedLayer` carry no attributes today, and
`VectorDataset::read()` is an empty stub — the filename `write()` persists is
never restored on project reload.

This issue asks for a second, **read-only** way to view the same kind of file:
a `map::Layer` in the Layers tab (like `RasterLayer`/backgrounds, ADR-0002/
ADR-0003) that renders points/lines/polygons with attribute-driven styling
(colour-by-field via `marine_colormap`, size-by-field) and click-to-inspect.

The issue review (progress.md, 2026-09-14) raised six open items; the operator
answered all six in a follow-up comment on the issue (2026-09-14). This plan
implements those answers directly:

1. Extend `vector_parse`'s `ParsedGeometry`/`ParsedLayer` with feature
   attributes and reuse it — do not re-derive OGR iteration, and do not touch
   `VectorDataset`'s existing editable-import behavior.
2. Attributes are in scope, carried by the extended parser.
3. Fix `VectorDataset::read()` (currently an empty stub) as a side finding in
   this PR — the new layer's own persistence must not copy that defect.
4. Architecture: per-feature `QGraphicsItem` children under the new layer (for
   free hit-testing / click-to-inspect), transformed to scene coordinates once
   at load — not a single painted surface like `RasterLayer`.
5. Colour-by-field uses `marine_colormap` (already integrated in `RasterLayer`
   and `ros/grids/grid_map.cpp`), not the deleted `camp::map::ColorMap` /
   stale #63.
6. MVP scope for this PR: OGR load (GeoJSON first), points/lines/polygons with
   a default style, colour-by-field, size-by-field, click-to-inspect
   attributes, save/restore in the project, unit tests for the parser
   attributes and the styling mapping. Label-by-field and a style-editing UI
   are follow-on (new issues, not this PR).

## Approach

1. **Extend `vector_parse` with attributes.** Add
   `QMap<QString, QVariant> attributes` (or a small `std::vector<std::pair<
   QString, QVariant>>` if map ordering for display matters — decide during
   implementation, default to `QMap` for by-name lookup) to `ParsedGeometry`.
   In `parseVectorLayers`, read every field from `feature`'s `OGRFeatureDefn`
   (`GetFieldCount()`/`GetFieldDefnRef(i)`/`GetFieldAsString`/
   `GetFieldAsDouble`/`GetFieldAsInteger` keyed by `OGRFieldType`) into that
   map before `OGRFeature::DestroyFeature(feature)`. `VectorDataset::
   buildItems` ignores the new field (source compatible; no behavior change
   for the editable import path).

2. **Fix `VectorDataset::read()`.** Restore the persisted filename and call
   `open()`, mirroring `write()`'s `json["filename"]`. Add a focused unit or
   read/write round-trip test (new small test file, or extend
   `test_vector_dataset_cleanup.cpp` if a headless `QJsonObject` round trip
   fits there without pulling in the full mission-item graph — otherwise a
   new `test_vector_dataset_persistence.cpp`). This is a one-line-cause,
   real-defect fix, landing in this PR per the operator's decision — not a
   separate issue.

3. **Add a `camp::vector::VectorLayer` class** under a new
   `src/camp_map/vector/` directory (mirrors `src/camp_map/raster/` for
   `RasterLayer`), deriving from `map::Layer`:
   - Constructor takes `(map::MapItem* parent, const QString& filename)`,
     mirrors `RasterLayer`'s shape: open the OGR dataset off the GUI thread
     (`QFutureWatcher`, abort flag + mutex — the `RasterLayer`/#213
     join-in-destructor pattern) via `parseVectorLayers`, reproject each
     `ParsedGeometry`'s WGS84 coordinates to the Web-Mercator scene **once**
     on load completion, and build one child `QGraphicsItem` per feature.
   - `boundingRect()` covers the union of child extents (Qt already unions
     child `boundingRect()`s into the parent's for hit-testing/painting
     purposes via `childrenBoundingRect()`; confirm during implementation
     whether an explicit override is needed given `MapItem::boundingRect()`'s
     current implementation).
   - Destructor joins the load worker before teardown (the #213 pattern
     already used by `ros/geometry/polygon.h` and `OccupancyGrid`, and cited
     directly in the issue).
   - `write()`/`read()` via the `MapItem::readSettings()`/`writeSettings()`
     override pair (QSettings, keyed by `itemID()`/`settingsKey()`) — see
     step 6 for why this is the app-state path, not the mission JSON.

4. **Add feature child-item classes** (new file, e.g.
   `src/camp_map/vector/vector_feature_item.{h,cpp}`): lightweight
   `QGraphicsItem` (not `MapItem`/`QGraphicsObject` — no need for the tree
   model, settings, or signal/slot machinery per feature) for Point,
   LineString, and Polygon geometry, each holding a `const ParsedGeometry*`
   (or a copy) and its resolved paint color/size. Distinct from
   `Point`/`LineString`/`Polygon` (`src/camp/*.h`) — those are `MissionItem`
   subclasses with editing/drag/waypoint-linking baggage that a read-only
   layer must not inherit (this is a deliberate, new, minimal class, not
   reuse of the editable classes — matches the "read-only" requirement in
   the issue's Out-of-scope section).
   - Default style: point = filled circle (fixed radius unless size-by-field
     is set), line = stroked path, polygon = filled+stroked path with
     exterior/interior rings (even-odd fill rule for holes).
   - `mousePressEvent()`: accept `Qt::LeftButton`
     (`setAcceptedMouseButtons`), and on press show the feature's attributes
     — `QToolTip::showText(event->screenPos(), text)` for the MVP (no
     existing feature-click infrastructure elsewhere in the codebase to
     match; a dedicated properties panel is a reasonable follow-on but out of
     scope for this MVP per item 6).
   - `boundingRect()`/`shape()` from the transformed geometry, in the parent
     layer's local (scene-mercator) coordinates, consistent with how
     `MapItem::setWebMercatorPositionAndScale` places geo-referenced items.

5. **Attribute-driven styling on `VectorLayer`.**
   - `setColorField(const QString& field)` / `colorField()`: when set, resolve
     each feature's numeric value for that field, compute the field's
     min/max across all features once at load (auto-range, matching
     `RasterLayer`'s `data_min_`/`data_max_` auto-range convention), and
     sample a `marine_colormap::Palette` (`find_palette(name)`, default
     `"viridis"`, `palette->sample(normalized_value)` → `Rgba8` →
     `to_rgba8`/`QColor`) per feature. No field set → the existing per-layer
     default style color.
   - `setSizeField(const QString& field)`: same min/max normalization, linear
     interpolation between a fixed min/max marker radius (e.g. 3–15 px) for
     points; lines/polygons ignore size-by-field (documented, not silently
     dropped — add a one-line status/log note if a size field is set on a
     layer with no point features, or simply document the point-only scope
     in the header comment).
   - `setColormap(const std::string& name)`: mirrors `RasterLayer::
     setColormap` for consistency, applies to colour-by-field only.
   - Recompute per-feature paint properties when a style setter changes
     (single pass over already-loaded features — no re-parse, no re-load).

6. **Persistence.** Per ADR-0003 §4, backgrounds/depth layers persist as
   **app/Map state** (QSettings), not the mission project file — mission
   items alone use the project JSON. `VectorLayer` is a `map::Layer` in the
   Layers tab, architecturally identical to `RasterLayer` in this respect, so
   it follows the **same** split:
   - A new `m_vectorLayers` list + `persistVectorLayers()`/
     `restorePersistedVectorLayers()` pair on `AutonomousVehicleProject`,
     mirroring `persistBackgrounds()`/`restorePersistedBackgrounds()`
     (`autonomousvehicleproject.cpp:291-320`) — QSettings key
     `vectorLayers/files`, de-dup by filename, self-heal on restore.
   - Per-layer style (`colorField`, `sizeField`, `colormap` name) persists via
     the existing `MapItem::readSettings()`/`writeSettings()` /
     `settingsKey()` mechanism `RasterLayer` already uses — no new
     persistence plumbing needed for style, just new keys under the
     item's settings group.
   - This is a **deliberate divergence from the issue text's "persist in the
     project file"** wording — recorded as an Open Question below since it
     is a judgment call the operator's six answers didn't explicitly settle
     (they addressed *which defect not to copy*, not *which persistence
     mechanism*). ADR-0003 §4 is the load-bearing precedent for this choice
     and is already Accepted, so this is presented as the plan's resolution
     rather than a blocking question — flag for review-plan to confirm.
   - Wire an `AutonomousVehicleProject::openVectorLayer(fname)` entry point
     (parallel to `addBackgroundLayer`), called from wherever the "Open
     vector file" action lands (check `mainwindow.cpp`/menu wiring during
     implementation for the existing "Import" vs "Open background" action
     pattern to match).

7. **Tests** (new `test/` files, wired into `CMakeLists.txt` next to the
   existing `ament_add_gtest` blocks for `test_vector_dataset_cleanup` /
   `test_raster_layer_gdal_cleanup`):
   - `test_vector_parse_attributes.cpp`: extend the existing GeoPackage-writer
     pattern from `test_vector_dataset_cleanup.cpp` with typed fields (string,
     int, real) on each feature; assert `ParsedGeometry::attributes` round
     trips the values and types. Keep the leak-check coverage from the
     existing test intact (attribute reads must not introduce new
     leak-prone OGR handles — `GetFieldAsString`/`GetFieldAsDouble` return
     borrowed pointers/values, no extra destroy needed, but confirm no
     `OGRFieldDefn` needs freeing).
   - `test_vector_layer_styling.cpp`: headless unit test of the
     colour-by-field and size-by-field mapping functions (extract the
     normalize+sample logic into small free functions or static methods
     that don't need a `QApplication`/GUI thread, matching
     `test_color_map.cpp`'s headless style) — assert min/max feature values
     map to the palette's first/last LUT entries and mid-range values
     interpolate; assert size-by-field maps to the documented min/max pixel
     radius.
   - `test_vector_layer_teardown.cpp` (or extend an existing thread-teardown
     test if one fits): confirm the load worker is joined in the destructor
     before the dataset/parsed data it captured is freed — the #213 pattern,
     analogous to `test_raster_layer_gdal_cleanup.cpp`'s abort-on-destroy
     coverage.
   - `test_vector_dataset_persistence.cpp` (or folded into an existing file,
     see step 2): `VectorDataset::write()`/`read()` round trip restores the
     filename and re-opens it.

8. **Manual acceptance** against the issue's two test datasets
   (`~/data/logs/analysis/2026-09-14_massabesic_mag/massabesic_mag_peaks.
   geojson`, `massabesic_joint_candidates.geojson`) — not automatable in this
   PR (real files outside the repo, GUI rendering), but recorded here so
   `review-code`/the PR description can check it off explicitly: load both,
   confirm colour-by-field on `analytic_signal_nT_per_m` puts the 58 nT/m
   peak at the top of the ramp, and confirm candidate C's popup identifies
   the point at 307792 E / 4762878 N (UTM 19N) with its `assessment` text.

## Files to Change

| File | Change |
|------|--------|
| `src/camp/vector/vector_parse.h` | Add `attributes` field to `ParsedGeometry`; document type-mapping rules |
| `src/camp/vector/vector_parse.cpp` | Read OGR field values into `attributes` per feature |
| `src/camp/vector/vectordataset.cpp` | Fix `read()` to restore filename + reopen (side finding, item 3) |
| `src/camp_map/vector/vector_layer.h` (new) | `VectorLayer : public map::Layer` — async OGR load, style setters, persistence hooks |
| `src/camp_map/vector/vector_layer.cpp` (new) | Load worker, scene-coordinate transform, style recompute, readSettings/writeSettings |
| `src/camp_map/vector/vector_feature_item.h` (new) | Read-only per-feature `QGraphicsItem` (point/line/polygon), click-to-inspect |
| `src/camp_map/vector/vector_feature_item.cpp` (new) | Paint + hit-test + `mousePressEvent` attribute popup |
| `src/camp/autonomousvehicleproject.h` | `m_vectorLayers`, `openVectorLayer()`, `persistVectorLayers()`/`restorePersistedVectorLayers()` declarations |
| `src/camp/autonomousvehicleproject.cpp` | Implementations mirroring `addBackgroundLayer`/`persistBackgrounds`/`restorePersistedBackgrounds` |
| menu/action wiring (TBD file, likely `mainwindow.cpp`) | "Open vector layer" action calling `openVectorLayer()` |
| `CMakeLists.txt` | New library sources + 4 new `ament_add_gtest` blocks |
| `test/test_vector_parse_attributes.cpp` (new) | Attribute parse round trip |
| `test/test_vector_layer_styling.cpp` (new) | Colour/size-by-field mapping |
| `test/test_vector_layer_teardown.cpp` (new) | Thread-safe teardown (#213 pattern) |
| `test/test_vector_dataset_persistence.cpp` (new, or folded into existing) | `VectorDataset::read()` fix regression test |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Capture decisions, not just implementations | The two architecture forks the issue review flagged (reuse `vector_parse`; per-feature items vs. single surface) are now operator-decided (issue comment, 2026-09-14) and restated in Context above rather than re-litigated. No new ADR is proposed — this plan treats it as an application of the existing ADR-0002/0003/0007/0008 precedents (RasterLayer's async-load shape, marker's per-item Layer precedent, marine_colormap adoption), not a new architectural pattern needing its own record. Flagged as an Open Question below in case review-plan disagrees. |
| A change includes its consequences | `VectorDataset::read()` fix (item 3) lands in this PR, not deferred. Persistence-mechanism choice (step 6) explicitly reasoned from ADR-0003 §4 rather than left implicit. |
| Only what's needed | Label-by-field and style-editing UI are explicitly deferred to follow-on issues per the operator's MVP scope (item 6) — not built speculatively. New feature-item classes are minimal `QGraphicsItem`s, not reuse of the heavier editable `Point`/`LineString`/`Polygon` MissionItem classes. |
| Test what breaks | Four new test files scoped in step 7, covering the two genuinely new failure surfaces (attribute parsing, styling math) plus the two precedented risk patterns in this codebase (GDAL/thread teardown, persistence round trip). |
| Improve incrementally | `VectorDataset` (editable import) is untouched behaviorally except the `read()` fix; the new read-only layer is fully additive. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0002 (Web-Mercator scene/layer model) | Yes | Coordinates transformed to scene Web-Mercator once at load (step 3), matching the ADR's "transform once, let Qt scale" convention already used by `RasterLayer` and mission items. |
| ADR-0003 (backgrounds as layers/depth tree) | Yes | `VectorLayer` is an ordinary Layers-tab layer (§1 pattern); §4's split persistence (app-state for Layers-tab layers, project-file for mission items) is the explicit basis for step 6's persistence design. |
| ADR-0007 (RasterFieldSource render abstraction) | No | Not applicable — `VectorLayer` does not implement `RasterFieldSource`; it is not a raster/field surface. Per-feature `QGraphicsItem` children are the chosen shape (operator decision, item 4), not RasterLayer's single-texture pattern. |
| ADR-0008 (marine_colormap LUT bake) | Yes | Colour-by-field uses `marine_colormap::find_palette`/`sample()` directly (step 5), the same facility `RasterLayer` and `grid_map.cpp` already use — not the deleted `camp::map::ColorMap`. |
| ADR-0011 (viewport-clip render convention) | No (for this PR) | The two acceptance datasets are 33 and 7 features; no viewport-scoped culling is implemented in this MVP. Noted as a follow-on if vector layers grow to survey-index-footprint scale, per the issue review's flag — not blocking here. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `ParsedGeometry`/`ParsedLayer` gain an `attributes` field | `VectorDataset::buildItems` (source-compatible, ignores new field) | Yes — verified no behavior change, noted in step 1 |
| A new `VectorLayer` type is added to the Layers tab | Any place that maps a persisted layer "type" string to a class (issue review's flag re: `missionitem.cpp`'s type-string switch) | Yes — `VectorLayer` persists via QSettings app state (step 6), not the mission JSON's type-string dispatch, so `missionitem.cpp` is unaffected. Confirmed during step 3/6 implementation that no mission-side type dispatch needs a new case. |
| `VectorDataset::read()` starts actually restoring on project load | Any project file that has a `VectorDataset` node with `filename` but previously loaded with no geometry (silently) | Yes — this is a bug fix that changes observable behavior (previously-empty `VectorDataset` nodes now populate on reload); called out explicitly in the PR description, not left as a silent side effect |
| A new menu action opens vector layers | User-facing docs / `.agents/README.md` if it documents the menu structure | Follow-up — see Documentation & Instruction Impact |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): None found — `.agents/README.md` (checked at plan time) does not document the Layers-tab menu structure or `VectorDataset`'s persistence behavior in enough detail to be made stale by this change. If implementation finds a specific doc claim invalidated (e.g. a menu-structure screenshot or a parameter table entry), it must be fixed in this PR per the "Never document from assumptions" rule.
- **Agent-instruction candidates** (proposals only): Consider adding a short note to `.agents/README.md`'s architecture overview once this lands, pointing future agents at `VectorLayer` as the read-only-display precedent (vs. `VectorDataset` as the editable-import precedent) — the two now look confusingly similar by name and this PR is exactly the moment that distinction is freshest. Operator decides whether/when.

## Open Questions

- [ ] Persistence mechanism: this plan resolves "persist in the project file" (issue wording) to ADR-0003 §4's app-state (QSettings) pattern, matching `RasterLayer`/backgrounds, since `VectorLayer` is a Layers-tab layer, not a mission item. The operator's six decisions didn't explicitly settle this. Confirm at review-plan, or override before implementation if the intent was literally the mission JSON.
- [ ] Menu/action wiring location (`mainwindow.cpp` or elsewhere) for "Open vector layer" — to be confirmed by reading the existing "Open background" action wiring during implementation; not expected to change the plan's shape.
- [ ] `ParsedGeometry::attributes` container type (`QMap<QString, QVariant>` vs. an ordered vector-of-pairs) — default to `QMap` for MVP; revisit only if attribute display order turns out to matter for the click-to-inspect popup.

## Estimated Scope

Single PR (MVP per operator decision, item 6). Label-by-field and a
style-editing UI are explicitly out of scope, to be filed as follow-on issues
after this PR lands.
