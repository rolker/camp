---
issue: 22
---

# Issue #22 — Import generic vector data

## Issue Review
**Status**: complete
**When**: 2026-09-14 10:38 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #22
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: needs-more-detail

### Actions
- [ ] Reconcile with the existing `VectorDataset` / `camp::vector::parseVectorLayers` code (`src/camp/vector/vectordataset.{h,cpp}`, `src/camp/vector/vector_parse.{h,cpp}`): this already opens arbitrary OGR vector files (GeoJSON, shapefile, etc.), RAII-closes the GDAL handle (issue #152), and builds Point/LineString/Polygon items — but into the editable MissionItem/Group tree, not a `map::Layer`. The issue's Related section omits it entirely. State explicitly whether #22 (a) reuses/extends `parseVectorLayers` for the new read-only layer (recommended — avoids re-deriving OGR iteration; "only what's needed"), (b) supersedes `VectorDataset`, or (c) is a deliberately separate concept (editable-import vs. read-only-display) — and why.
- [ ] `ParsedGeometry`/`ParsedLayer` (`vector_parse.h`) currently carry **no attribute fields at all** — attributes are dropped during parsing. The issue's "Must have" list (attribute-driven styling, click-for-attributes) requires extending this struct/parse path; scope that work explicitly in the plan rather than assuming "just wire up styling to existing data."
- [ ] `VectorDataset::write()` persists the source filename but `VectorDataset::read()` is an empty stub — the persisted file is never restored on project reload today. The new layer's "Must have: persist by path + style" is exactly the feature quietly broken in the sibling code; note this so a reviewer doesn't assume the existing persistence path can be copied as-is.
- [ ] The issue asks to both "mirror RasterLayer" (a single async-loaded Layer that paints one composited surface, per ADR-0007) and support "click a feature → its attributes." RasterLayer has no per-feature hit-testing; `VectorDataset`'s approach (one QGraphicsItem child per feature) gets Qt's native item-picking for free but doesn't fit a single async field-source layer. Pick and state the architecture (custom hit-testing in one Layer vs. per-feature child items) during plan-task — the issue is silent on this fork and it drives the whole implementation shape.
- [ ] The "reusable colormap facility (#63)" reference is stale: `camp::map::ColorMap` (the facility #63 originally scoped) was deleted in camp#141/PR#144 (ADR-0008) and replaced by the external `marine_colormap` library, already integrated in `RasterLayer` and `ros/grids/grid_map.cpp` (`marine_colormap::find_palette` + `palette->sample()`). This is good news, not a blocker — the real facility already exists and is in active use — but #22 should cite `marine_colormap` / ADR-0008 directly, and #63 (still open) looks like a stale tracking issue worth closing as superseded in a follow-up.
- [ ] No tests are mentioned in the issue's scope, despite this area's strong precedent (`test_raster_layer_gdal_cleanup.cpp`, `test_color_map.cpp`, `test_catalog_source.cpp`) and the "Test what breaks" principle. Plan-task should scope: attribute-parse extension tests, the thread-safe-teardown test mirroring `ros/geometry/polygon.h`'s #213 pattern (join-worker-in-destructor), and colour/size-by-field mapping tests.
- [ ] Consider phasing: load+render+persist+teardown as an MVP slice, attribute-driven styling + click-popup as a follow-on — the combined "Must have" list is large for one PR (six distinct capabilities). Not blocking; a recommendation for plan-task.
- [ ] ADR-0003's depth-tree / Z-order model and ADR-0011's viewport-clip convention should be checked against the new layer's paint path once the architecture fork above is resolved — flag if per-feature rendering needs viewport clipping for larger vector sets (the two cited test datasets are tiny — 33 and 7 features — but the issue frames this as general-purpose, and future consumers like survey-index footprints could be much larger).


## Plan Authored
**Status**: complete
**When**: 2026-09-14 10:48 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-22/plan.md` at `75182fc`
**Branch**: feature/issue-22 at `75182fc`
**Phases**: single

### Open questions
- [ ] Persistence mechanism: plan resolves "persist in the project file" (issue wording) to ADR-0003 §4's app-state (QSettings) pattern like RasterLayer/backgrounds, since VectorLayer is a Layers-tab layer, not a mission item — the operator's six decisions didn't explicitly settle this; confirm at review-plan.
- [ ] Menu/action wiring location ("Open vector layer") to be confirmed against the existing "Open background" action during implementation.
- [ ] `ParsedGeometry::attributes` container type (QMap<QString, QVariant> vs. ordered vector-of-pairs) — defaulted to QMap; revisit only if display order matters.

## Plan Review
**Status**: complete
**When**: 2026-09-14 10:53 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-22/plan.md` at `75182fc`
**PR**: PR-less (dispatched `--issue 22`, branch `feature/issue-22`)
**Verdict**: changes-requested

Dimension verdicts — Scope: good. Issue alignment: good (all six operator
decisions honoured). File targeting: concern (library-layering link error;
three missing files). Consequences: concern (removal/de-persist path absent;
one consequence row rests on a false premise). Documentation & instruction
impact: good (present and non-silent). Principle alignment: needs work
("verify against source, never document from assumptions" — step 2's premise
is wrong; "a change includes its consequences" — de-persist missing). ADR
compliance: good (ADR-0002/0003/0008 correctly applied; 0007/0011 correctly
excluded). ROS conventions: N/A (pure Qt/GDAL, ADR-0002 ROS-free boundary).

Verified against source: `marine_colormap` API (`find_palette` /
`palette->sample(t)` clamped to [0,1] / `to_rgba8` → `Rgba8`) as used in
`grid_map.cpp:152-219` — plan's usage is correct. `RasterLayer`'s async shape
(`QFutureWatcher` + `abort_flag_`/mutex, `waitForFinished()` in the dtor,
synchronous `initExtent`) — correct. The camp#213 join-in-destructor pattern in
`ros/geometry/polygon.h` — correct. `ParsedGeometry`/`ParsedLayer` carry no
attributes — correct. `VectorDataset::read()` is an empty stub — correct, but
see finding 1 for what that actually means.

### Findings
- [ ] (must-fix) Step 2's premise is false: a persisted `VectorDataset` IS restored on project load — `missionitem.cpp:185` dispatches `object["type"] == "VectorDataset"` to `AutonomousVehicleProject::openGeometry(filename)` (`autonomousvehicleproject.cpp:200-214`), which constructs and `open()`s the dataset. `readChildren` creates no `VectorDataset` item, so `read()` is never called and the planned "restore filename + call open()" fix would be dead code (and a double-load if an item were also created). Re-scope item 3 to the defect that is actually there — `openGeometry` inserts into `m_currentGroup`, not into the node's own parent, so a nested `VectorDataset` is restored at the wrong place — or close it as not-a-defect-as-described with the evidence. The operator's decision 3 was made on this false premise and needs one line of re-confirmation before implementation — `plan.md:58-65`
- [ ] (must-fix) The consequences row "previously-empty `VectorDataset` nodes now populate on reload" is wrong for the same reason and must not reach the PR description as a behavior-change claim — `plan.md:243`
- [ ] (must-fix) Library layering: `src/camp_map/vector/vector_layer.cpp` would live in the `camp_map` SHARED library, but `camp::vector::parseVectorLayers` is compiled only into the `CCOMAutonomousMissionPlanner` executable (`CMakeLists.txt:135`). `.agents/README.md` states the rule explicitly — "code in a library cannot call into the executable that links it" (the libcamp_crash lesson, #217). Either move `vector_parse.{h,cpp}` into `camp_map` (it is pure Qt/GDAL and ROS-free, so it belongs there; `VectorDataset` then includes it from there) or keep `VectorLayer` in the executable's sources. Decide in the plan, not at link time — `plan.md:67-69, 204-205`
- [ ] (must-fix) The async load worker must call `camp_crash::install_thread_alt_stack()` as its first statement — enforced by the `check_worker_alt_stacks` CTest guard (`CMakeLists.txt:554`), which fails CI for any `QtConcurrent::run()` entry point that omits it (`raster_layer.cpp:189` is the pattern). Not mentioned anywhere in the plan — `plan.md:70-75`
- [ ] (must-fix) Persistence is missing its removal half. `RasterLayer::onRemovedFromMap()` (`raster_layer.cpp:749-757`) and `AutonomousVehicleProject::onChartLayerRemoved` (`autonomousvehicleproject.cpp:322+`, wired to the Map model's `rowsAboutToBeRemoved`) exist precisely so a layer removed from the Layers tab stays removed across restarts (camp#90/#117). Step 6 plans only persist/restore, so a removed vector layer would come back on next launch. Add the `onRemovedFromMap()` override plus the model-removal bookkeeping, and mirror `test_background_persistence.cpp`'s `RemoveDePersistsAndSticks` coverage — `plan.md:132-158, 160-186`
- [ ] (must-fix) Styling math has no degenerate/absent-value handling: a field whose values are all equal divides by zero, and a feature missing the field, or holding a non-numeric or NaN value, has no defined colour/size. `grid_map.cpp:200-206` handles exactly this case for grids. Specify the behaviour and test it — `plan.md:112-126, 171-178`
- [ ] (must-fix) The reused parser drops geometry it does not name: `vector_parse.cpp:86-140` switches on raw `getGeometryType()` and handles only `wkbPoint`/`wkbLineString`/`wkbPolygon`. Multi-part geometries (`wkbMultiPolygon` etc., routine in shapefile and KML) and every 25D variant (a GeoJSON point with an elevation is `wkbPoint25D`) fall through `default: break` and vanish silently. The issue claims shapefile/GeoPackage/KML "come free from the driver"; as written they do not. Either add `wkbFlatten` + multi-part handling in step 1, or state the limitation in the plan and PR and file the follow-on — `plan.md:47-56`
- [ ] (must-fix) Click-to-inspect is unspecified against the view that owns the mouse. `ProjectView::mousePressEvent` (`projectview.cpp:57-196`) runs the add-waypoint/trackline/survey modes on left-press and then forwards to `QGraphicsView::mousePressEvent` unconditionally, with `setDragMode(ScrollHandDrag)` in pan mode. Define what a left-click on a feature does in pan mode and in each add-* mode, so the attribute popup neither fights the hand-drag nor fires while the operator is placing waypoints — `plan.md:102-107`
- [ ] (must-fix) Three files missing from "Files to Change": `src/camp_map/map/item_types.h` (a new `VectorLayerType` enum entry — every `Layer` subclass declares one, `item_types.h:22-48`), `src/camp/mainwindow.cpp:161` (the startup `restorePersistedVectorLayers()` call, alongside `restorePersistedBackgrounds()`), and the `.agents/README.md` architecture note the plan itself proposes — `plan.md:197-215`
- [ ] (suggestion) Per-layer style settings key on `itemID()`, which derives from `objectName()` = the file's basename. Two vector files with the same basename in different directories would share one settings group. `GggsTileLayer` overrides `settingsKey()` for exactly this reason (camp#126, `map_item.h:56-61`); consider the same here — `plan.md:142-146`
- [ ] (suggestion) Latent lat/lon inconsistency in the parser you are extending: the Point path builds `QGeoCoordinate(getY(), getX())` when untransformed, while `readRing` builds `(getX(), getY())` — so line and polygon vertices from a source with no spatial ref are swapped. Pre-existing and out of scope, but worth confirming while the attribute work is in that file, since the acceptance test is a coordinate check — `vector_parse.cpp:96-104` vs `:46`
- [ ] (suggestion) Open Question 1 (persistence) — recommend **keep QSettings app state** per ADR-0003 §4: a Layers-tab layer is not mission data, and the operator's intent ("the layer returns when the project reopens") is met, since `mainwindow.cpp:161` restores layers at startup. But the plan's stated reason is inaccurate — the operator's decision 6 does repeat "save/restore in the project file" verbatim, so this is not an item their answers left silent. Put it to the operator as one explicit yes/no before implementation rather than treating ADR precedent as having settled it — `plan.md:147-153, 253`


## Plan Authored
**Status**: complete
**When**: 2026-09-14 11:02 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-22/plan.md` at `b4fd5b3` (rev 2)
**Branch**: feature/issue-22 at `b4fd5b3`
**Phases**: single

Rev 2 responds to the Plan Review (changes-requested, 9 must-fix + 3
suggestion findings). Every finding was re-verified against source before
being folded in:

1. **Step 2 premise was false (persisted `VectorDataset` restores fine)** —
   replaced. The real defect, confirmed at `missionitem.cpp:185` and
   `autonomousvehicleproject.cpp:200-214`: `openGeometry()` always inserts
   under `m_currentGroup`, ignoring the node's actual parent. Plan now fixes
   `openGeometry`'s insertion target (new parent parameter, defaulted for
   the existing call site) with a nested-`Group` regression test. All
   `read()`-stub work dropped.
2. **Wrong consequences row** ("previously-empty nodes now populate") —
   removed; it rested on the same false premise.
3. **Library layering** — `vector_parse.{h,cpp}` now moves from the
   executable into `src/camp_map/vector/` so `VectorLayer` (which must live
   in `camp_map`) can call it, per `.agents/README.md`'s
   library-cannot-call-executable rule; verified current placement at
   `CMakeLists.txt:135` vs. `camp_map`'s separate `SHARED` target
   (`CMakeLists.txt:295-339`).
4. **Missing alt-stack call** — step 3 (VectorLayer) now specifies
   `camp_crash::install_thread_alt_stack()` as the load worker's first
   statement, matching `raster_layer.cpp:189`, required by the
   `check_worker_alt_stacks` CTest guard (`CMakeLists.txt:554-557`).
5. **Persistence missing its removal half** — step 7 now adds
   `VectorLayer::onRemovedFromMap()` + an `onVectorLayerRemoved` slot wired
   to the Map model's `rowsAboutToBeRemoved`, mirroring
   `RasterLayer::onRemovedFromMap()` (`raster_layer.cpp:749-757`) and
   `onChartLayerRemoved` (`autonomousvehicleproject.cpp:322+`, camp#90/
   #117), with a persistence test covering add/restore/remove/stays-removed.
6. **No degenerate/absent-value styling handling** — step 5 now specifies:
   all-equal values get an epsilon offset (mirroring `grid_map.cpp:
   198-206`'s guard) instead of dividing by zero; missing/non-numeric/NaN
   values get a fixed neutral color/radius, never palette index 0 by
   accident. Covered by new styling-test cases.
7. **Silent geometry drops** — step 1 now switches on `wkbFlatten()` (so
   every 25D/ZM variant reaches its base case) and adds
   `wkbMultiPoint`/`wkbMultiLineString`/`wkbMultiPolygon` handling instead
   of the prior `default: break` that dropped them, confirmed against
   `vector_parse.cpp:86-140`. New test coverage for multi-part + 25D
   features.
8. **Click-to-inspect unspecified against the real mouse state machine** —
   step 4 now specifies the popup fires only in pan mode, since
   `ProjectView::mousePressEvent` (`projectview.cpp:57-196`) runs
   waypoint/trackline/survey placement on left-press in its add-* modes and
   always forwards to `QGraphicsView::mousePressEvent` regardless of mode
   (confirmed `setDragMode(ScrollHandDrag)` at `projectview.cpp:335`), so a
   naive always-on popup would fire during placement.
9. **Missing Files-to-Change entries** — `item_types.h` (new
   `VectorLayerType`, confirmed pattern at `item_types.h:22-45`),
   `mainwindow.cpp:161`'s `restorePersistedVectorLayers()` startup call
   (alongside the confirmed existing `restorePersistedBackgrounds()`
   call), and an `.agents/README.md` note are now all listed.

Suggestions also addressed: `VectorLayer::settingsKey()` now overrides the
`itemID()` default (camp#126 precedent, `gggs_tile_layer.cpp:1829-1852`) to
avoid a basename collision between same-named files in different
directories; the pre-existing Point-vs-`readRing` lat/lon-order
inconsistency (`vector_parse.cpp:46` vs. `:96-104`) is flagged in step 1
for a fix-or-explicit-follow-up decision during implementation; the
persistence-mechanism question is resolved (QSettings app state, both
halves) rather than left as an Open Question.

### Open questions
- [ ] `ParsedGeometry::attributes` container type (`QMap` vs. ordered
      vector-of-pairs) — default `QMap` for MVP.
- [ ] Whether `ProjectView` needs a new read-only `mouseMode` accessor for
      the click-to-inspect pan-mode gate, or one already exists.
- [ ] Whether the lat/lon-order fix in `vector_parse.cpp` lands in this PR
      or as a separate follow-up issue — confirm during implementation.

## Plan Review
**Status**: complete
**When**: 2026-09-14 11:07 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-22/plan.md` at `b4fd5b3` (rev 2)
**PR**: PR-less (dispatched `--issue 22`, branch `feature/issue-22`)
**Verdict**: changes-requested

Round 2 — re-review of rev 2 against the round-1 review (9 must-fix + 3
suggestions) and the operator's plan-review-checkpoint decisions on the issue
(2026-09-14). Every rev-2 claim below was re-verified against the source, not
taken from the plan's own account.

Dimension verdicts — Scope: good. Issue alignment: good (both operator
decisions and the earlier six honoured; "save/restore in the project file" is
correctly recorded as superseded). File targeting: good (the three missing
files are listed; the `camp_map` move is link-clean — `vector_parse.h` includes
only `<QGeoCoordinate>` and forward-declares `GDALDataset`, and camp_map links
Qt5::Positioning PUBLIC / GDAL PRIVATE, `CMakeLists.txt:345-382`).
Consequences: good, with one ownership ambiguity (suggestion 1). Documentation
& instruction impact: good. Principle alignment: good — "test what breaks" is
the one soft spot (must-fix 1). ADR compliance: good. ROS conventions: N/A.

**Round-1 findings confirmed genuinely resolved (verified in source):**
1. `openGeometry` premise — confirmed: `missionitem.cpp:186` dispatches
   `type == "VectorDataset"` to `project->openGeometry(...)` with no parent,
   and `autonomousvehicleproject.cpp:200-214` does
   `RowInserter ri(*this, m_currentGroup); vd = new VectorDataset(m_currentGroup)`.
   Rev 2's re-scope is the real defect. Both `VectorDataset(MissionItem*)` and
   `RowInserter(…, MissionItem*, int)` already take `MissionItem*`, so
   `readChildren` passing `this` types cleanly.
2. The false consequences row is gone from the table.
3. Library layering — confirmed `vector_parse.cpp` is in the executable's
   `SOURCES` (`CMakeLists.txt:135`) and `camp_map` is a separate SHARED target
   (`CMakeLists.txt:295+`); the move resolves it, and the header's include set
   makes it a clean public camp_map header.
4. Alt-stack — `camp_crash::install_thread_alt_stack()` as first statement
   matches `raster_layer.cpp:189`; `cmake/check_worker_alt_stacks.cmake` is a
   per-file count over `src/**.cpp`, so one call per worker file satisfies it.
5. Removal half — `Layer::removeFromMap()` (`layer.cpp:54-70`) calls
   `onRemovedFromMap()` *before* detaching through the model, and
   `onChartLayerRemoved` is connected at `autonomousvehicleproject.cpp:61`.
   Both halves in step 7 are correctly placed.
6. Degenerate styling — matches `grid_map.cpp:203-206`
   (`src/camp_map/ros/grids/grid_map.cpp`; `min_value -= 1.0` → everything
   normalizes to 1.0, the outcome the plan states). Missing/NaN → neutral colour
   is an addition beyond the grid precedent and is the right call.
7. Geometry coverage — confirmed `vector_parse.cpp:86-140` switches on raw
   `getGeometryType()` with `default: break`; `wkbFlatten` + multi-part is the
   right fix.
8. Click-to-inspect — confirmed `ProjectView::mousePressEvent`
   (`projectview.cpp:57-196`) runs placement per `mouseMode` and forwards to
   `QGraphicsView::mousePressEvent` unconditionally at `:196`, with
   `setDragMode(ScrollHandDrag)` at `:335`. Qt delivers the press to the scene
   *before* starting hand-scrolling, so "accept in pan mode, `ignore()` in add-*
   modes" is correct and does not fight the drag.
9. Missing files — `item_types.h` (enum at `:23-49`, cited as 22-45),
   `mainwindow.cpp:161` (`restorePersistedBackgrounds()` confirmed there), and
   the `.agents/README.md` note are all listed. Suggestions 1 and 2 are folded
   in (`GggsTileLayer::settingsKey()` at `gggs_tile_layer.cpp:1829+` is the
   right precedent; `itemID()` is parent-path+objectName at `map_item.cpp:53-61`);
   suggestion 3 is settled by operator decision, not by ADR inference.

### Findings
- [ ] (must-fix) Step 2's regression test has no harness that exists: no test in `test/` constructs `AutonomousVehicleProject`, and its TU pulls the whole mission-item tree plus `platform_manager/platform.h` and `mission_manager/mission_manager.h` (`autonomousvehicleproject.cpp:16-34`), while every existing `ament_add_gtest` block compiles a narrow source set (`test_astar` + `astar.cpp`, `test_vector_dataset_cleanup` + `vector_parse.cpp`) or links `camp_map`. The plan's escape hatch — "or folded into an existing mission-item test file if a closer fit exists" — points at a file that does not exist. Name the seam in the plan (e.g. extract the insertion-target resolution into a small free function testable without `AutonomousVehicleProject`, or state the source/link set the new target needs) or state the fallback explicitly, so the operator-required regression test is not silently dropped at build time — `plan.md:213-219, 388-393, 437`
- [ ] (suggestion) Two writers for one settings key: `VectorLayer::onRemovedFromMap()` edits `vectorLayers/files` directly *and* `onVectorLayerRemoved` re-persists the whole key from `m_vectorLayers` moments later (`Layer::removeFromMap()` calls the former first, `layer.cpp:54-70`). The raster precedent it mirrors has two writers over two *different* keys (`GggsRasters/files` in `raster_layer.cpp:754`, `backgrounds/files` in `autonomousvehicleproject.cpp:301`), so the pattern does not carry over unchanged. Net effect is correct today but redundant; name one owner of the key — `plan.md:253-255, 346-358`
- [ ] (suggestion) "`openGeometry()` gains a parent-group parameter (defaulting to `m_currentGroup`)" is not literally expressible — a default argument cannot name a non-static member. Use a `MissionItem* parent = nullptr` sentinel resolved to `m_currentGroup` in the body, and remember `RowInserter` must take the same resolved parent or the model's insert notification targets the wrong index — `plan.md:209-212, 423-424`

## Implementation
**Status**: complete
**When**: 2026-09-14 11:45 -04:00
**By**: Claude Opus

**Plan**: `.agent/work-plans/issue-22/plan.md` (rev 4 — implementation notes
folded in inline, plan-first workflow)
**Branch**: feature/issue-22 (six commits, unpushed)

### Commits
- `7111033` vector: move the OGR parser into camp_map — `vector_parse.{h,cpp}`
  from the executable's SOURCES into `CAMP_MAP_SOURCES` so the new library-side
  layer can call it (the library-cannot-call-the-executable rule, #217). No
  behavior change.
- `9a74003` vector: carry feature attributes and stop dropping geometry —
  `ParsedGeometry::attributes` (typed, absent-when-unset), `wkbFlatten` +
  Multi* recursion so 25D and multi-part features are no longer dropped
  silently (unhandled types now log), and the latent lat/lon swap in
  `readRing`'s untransformed branch fixed. New `test_vector_parse_attributes`.
- `16f451f` mission: restore a nested VectorDataset under its own parent —
  `camp::mission::resolveInsertionParent` seam + `openGeometry`'s nullptr
  parent sentinel routing both the RowInserter and the new node;
  `readChildren` passes `this`. New `test_mission_insertion`.
- `b592118` vector: add the read-only VectorLayer with attribute-driven
  styling — `VectorLayer` (async load with the alt-stack install and the #213
  join-in-destructor), `VectorFeatureItem` per feature, `vector_style.{h,cpp}`
  for the colour/size mapping, `VectorLayerType`, path-keyed `settingsKey()`.
  New `test_vector_layer_styling`, `test_vector_layer_teardown`.
- `5851c77` project: open, persist and remove vector display layers —
  `openVectorLayer` / `persistVectorLayers` / `restorePersistedVectorLayers` /
  `onVectorLayerRemoved`, the File > Open Vector Layer action and the startup
  restore call. New `test_vector_layer_persistence`.
- `762faac` docs: `.agents/README.md` note distinguishing VectorLayer from
  VectorDataset, and the plan's rev-4 sync.

### Verified
- **Build**: `./ui_ws/build.sh camp` — Finished, no errors. (marine_colormap
  and marine_colormap_widgets had to be built in this worktree's ui_ws first;
  they were not installed there.)
- **Tests**: `./ui_ws/test.sh camp` — **352 tests, 0 errors, 0 failures, 1
  skipped** (up from 323 on the branch point; the skip is the pre-existing
  GL-dependent case). The five new suites: parse attributes 5/5, styling 7/7,
  teardown 3/3, persistence 5/5, insertion seam 4/4. The #217 guards
  (`check_worker_alt_stacks`, `check_crash_lib_deps`, `check_camp_exports`)
  pass, so the new QtConcurrent worker's alt-stack install is accounted for.
- **Acceptance datasets, off-GUI**: both real files were driven through the
  shipped parse + styling path in a scratch probe (not committed):
  `massabesic_mag_peaks.geojson` → 33 features, `analytic_signal_nT_per_m`
  range 23.16–58.07, the 58.07 peak normalizing to t = 1.000 (top of the ramp);
  `massabesic_joint_candidates.geojson` → 7 features, candidate C reading back
  at lat 42.994286 / lon -71.357873 = **307792 E / 4762878 N UTM 19N**
  (confirmed with `gdaltransform`) carrying its `assessment` text. That covers
  the coordinate and attribute halves of plan step 9.

### Deferred / not done
- **The GUI half of step 9's manual acceptance**: on-screen rendering, the
  click-to-inspect popup in pan mode, and the remove-then-restart check. Not
  automatable here (a real GUI session); the data path behind all three is
  covered by the tests and the probe above.
- **`VectorLayer::onRemovedFromMap()` override — deliberately not added.** The
  removal half is the project's `onVectorLayerRemoved`, connected to the Map
  model's `rowsAboutToBeRemoved`; with the round-2 "one owner for the key"
  decision the override would write nothing and signal nothing new, i.e. dead
  code. Recorded in plan.md rev 4.
- **No `ProjectView` change** — pan mode is read from the view's `dragMode()`
  (ScrollHandDrag), so no accessor was needed and the camp_map layering stays
  intact. Open Question 2 answered in plan.md rev 4.
- **Label-by-field and a style-editing UI** remain out of scope per the
  operator's MVP decision; they are the follow-on issues the plan names.
- **Note for review**: the pre-commit trailing-whitespace hook cleaned a few
  pre-existing whitespace-only lines in `src/camp/mainwindow.cpp` as a side
  effect of editing that file. Mechanical, no code change.

### Actions
- [ ] Run `/review-code` against the branch diff before opening the PR.
- [ ] Manual GUI acceptance (render both datasets, colour by
      `analytic_signal_nT_per_m`, click candidate C, remove a layer and
      restart) — the operator-side half of step 9.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-14 11:54 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-22 at `0cfa138`
**Mode**: pre-push
**Depth**: Deep (reason: 3200+ changed lines across 27 non-plan files; new async worker, new persisted app state, cross-module layer/model wiring)
**Must-fix**: 10 | **Suggestions**: 11
**Round**: 1 | **Ship**: continue — nine code must-fixes on a first read, including a cross-pass-confirmed persistence defect and a cluster of unguarded-coordinate correctness issues; another independent read is warranted after they land.

Specialists: Static Analysis (pre-commit clean; cppcheck clean on every new
file — its only two hits sit on untouched context lines in
`autonomousvehicleproject.cpp:1183`), Governance, Plan Drift, Claude
Adversarial Lens A + Lens B. Copilot and Local Model off (default).

Build/test evidence reproduced in this worktree: `./ui_ws/build.sh camp`
clean; `./ui_ws/test.sh camp` — **352 tests, 0 errors, 0 failures, 1
pre-existing skip**, all five new suites present and green,
`check_worker_alt_stacks` passing (the new worker's alt-stack install is the
first statement, arithmetic 1:1).

Plan adherence: no must-fix drift. Every Plan Review must-fix and all seven
operator decisions are honoured in code, and plan rev 4 was committed on-branch
before the Implementation entry. Two of the three declared deferrals are
justified; the third (no `onRemovedFromMap` override) rests on an inaccurate
premise — see must-fix 1.

### Findings
- [x] (must-fix) Dragging a vector layer to reorder it silently un-persists it: `Map::setMapItemParent` does a reorder as beginRemoveRows+beginInsertRows, so the `rowsAboutToBeRemoved` handler erases it from `vectorLayers/files`; use the reorder-safe `Layer::onRemovedFromMap()` hook as RasterLayer/GggsTileLayer do — `src/camp/autonomousvehicleproject.cpp:422`
- [x] (must-fix) `shape()` returns the raw open path for LineStrings, so Qt's fill-area hit test never picks a line and click-to-inspect is unusable on line features; stroke it as `LineString::shape()` does — `src/camp_map/vector/vector_feature_item.cpp:107`
- [x] (must-fix) `OGRCoordinateTransformation::Transform()`'s return is discarded (a failed point yields HUGE_VAL), and a null transform on a layer that HAS an SRS falls silently through to the untransformed branch — `src/camp_map/vector/vector_parse.cpp:43` and `:203`
- [x] (must-fix) No coordinate-validity guard before `geoToMap()`: a `.prj`-less shapefile or a polar KML puts NaN / 1e17-metre positions into the scene and poisons `childrenBoundingRect()` and the scene index — `src/camp_map/vector/vector_feature_item.cpp:61`
- [x] (must-fix) `wkbGeometryCollection` is warn-and-dropped although the `toGeometryCollection()` recursion directly above already handles it — the same data-loss class this commit set out to close — `src/camp_map/vector/vector_parse.cpp:136`
- [x] (must-fix) The abort flag is read once, before `GDALOpenEx`, and `parseVectorLayers` has no cancellation hook, so `~VectorLayer()`'s join blocks the GUI thread for the whole parse; RasterLayer re-checks inside its work loops — `src/camp_map/vector/vector_layer.cpp:60`
- [x] (must-fix) Unbounded, uninterruptible per-feature item construction on the GUI thread, from an unfiltered file dialog: a coastline shapefile hangs CAMP; minimal remedy is a feature cap reported in the layer status — `src/camp_map/vector/vector_layer.cpp:101`
- [x] (must-fix) Degenerate-range guard `low = range.max - 1.0` is a no-op above 2^53 (int64 ids, ns timestamps), giving NaN into `palette->sample()` and the radius arithmetic — the one hole in the header's documented totality contract — `src/camp_map/vector/vector_style.cpp:60`
- [x] (must-fix) The lat/lon-swap fix has no regression test: no case asserts a line or polygon vertex's latitude/longitude on either branch — `test/test_vector_parse_attributes.cpp:275`
- [x] (must-fix) No ADR for camp's fourth layer family (first non-raster, deliberately departing from ADR-0007) and no record of the new persisted schema; every predecessor has one. Remedy: a short ADR, or an explicit operator decision in the PR body that none is wanted — `docs/decisions/`
- [x] (suggestion) A temporarily-unreachable persisted file is permanently forgotten: skip-then-re-persist deletes the entry after one launch with the share unmounted — `src/camp/autonomousvehicleproject.cpp:405`
- [x] (suggestion) `withoutVectorLayerFile()` has no production caller, so `RemoveDePersistsAndSticks` proves a rule the app never executes; the CMake comment overclaims — `src/camp_map/vector/vector_layer.cpp:314`
- [x] (suggestion) `viewInPanMode()` reads any attached view's drag mode rather than the event's (`event->widget()`) — `src/camp_map/vector/vector_feature_item.cpp:177`
- [x] (suggestion) The popup fires on mousePress and accepts the event, so panning over a feature pops an unrequested tooltip and grabs the gesture; gate on release-without-movement — `src/camp_map/vector/vector_feature_item.cpp:187`
- [x] (suggestion) `size_range` is folded over all features although only points are sized, compressing marker range on mixed-geometry files — `src/camp_map/vector/vector_layer.cpp:183`
- [x] (suggestion) Dedup and removal key on the raw path string; `./` and symlink spellings stack duplicates — use `canonicalFilePath()` — `src/camp/autonomousvehicleproject.cpp:376`
- [x] (suggestion) `GDALOpenEx` accepts the full registered driver set on an operator-supplied string, which OGR treats as a connection string (`/vsicurl/`, `PG:`, KML NetworkLink); pin an allowed-driver list — `src/camp_map/vector/vector_layer.cpp:79`
- [x] (suggestion) Doc contract is wrong: a null result is documented as "inserts at top level" but `RowInserter` dereferences the parent — assert non-null or fix the comment — `src/camp/mission_insertion.h:29`
- [x] (suggestion) Removal leaves an orphan `MapItem/file:<path>` settings group; camp#117's convention removes it (`map_tiles.cpp:358`), RasterLayer does not — split precedent, worth a decision — `src/camp_map/vector/vector_layer.cpp:127`
- [x] (suggestion) Docs describe click-to-inspect unconditionally, but it works only in pan mode and is a `QToolTip`, not a persistent popup — `.agents/README.md:110`, `src/camp_map/vector/vector_layer.h:31`
- [x] (suggestion) Cross-antimeridian lines and polygons draw back across the world and blow out the layer extent; at minimum a documented limitation — `src/camp_map/vector/vector_feature_item.cpp:113`

## Implementation
**Status**: complete
**When**: 2026-09-14 12:29 -04:00
**By**: Claude Opus

**Branch**: feature/issue-22 at `b24b7fd`
**Addressed**: `## Local Review (Pre-Push)` (2026-09-14 11:54 -04:00, branch at `0cfa138`) — verdict changes-requested, 10 must-fix + 11 suggestions
**Commits**: `e2d6922`, `01644e5`, `d017d22`, `8025128`, `d2a6284`, `4cf7b2b`, `4555fbb`, `b24b7fd`

**Build/test**: `./ui_ws/build.sh camp` clean; `./ui_ws/test.sh camp` — **367 tests,
0 errors, 0 failures, 1 pre-existing skip** (352 before this round; +15 new). No
test skipped, disabled or relaxed; no lint rule suppressed.

### Actions

All ten must-fixes are fixed with tests. Each line is finding → commit → what was
done and how it is verified.

- [x] **1. A drag-reorder un-persisted the layer** — `e2d6922`. `VectorLayer::onRemovedFromMap()` now emits `removedFromMap()`, and the project connects to that per layer instead of to the Map model's `rowsAboutToBeRemoved` (which `Map::setMapItemParent()` also fires for a reorder). `persistVectorLayers()` remains the single writer of `vectorLayers/files`; the override writes nothing. A `QObject::destroyed` connection keeps the bookkeeping free of dangling pointers without persisting (shutdown destroys every layer). Verified by `VectorLayerPersistence.ReorderKeepsFilePersistedRemovalDropsIt`, which reproduces the project's two lines of wiring, reorders through `Map::setMapItemParent()` and asserts the file is still persisted, then removes and asserts it is gone — `src/camp/autonomousvehicleproject.cpp:64`, `src/camp_map/vector/vector_layer.cpp:376`
- [x] **2. `shape()` returned the raw open path for lines** — `01644e5`. Qt picks an item by testing the click against `shape()`'s fill area, and an open path encloses none, so click-to-inspect silently did not work for any line feature. The path is stroked into a click ribbon as the mission-tree `LineString::shape()` does, and `boundingRect()` grows by the stroker's half-width (a shape outside the bounding rect is undefined behaviour in Qt). Verified by `VectorFeatureItem.LineShapeIsClickable`: a point midway along the segment hits, a point off the line does not, and the shape lies inside the bounding rect — `src/camp_map/vector/vector_feature_item.cpp:118`
- [x] **3. `Transform()`'s return discarded; a failed SRS fell through untransformed** — `d017d22`. The per-point success flag is now checked (`Transform(1, &x, &y, nullptr, &succeeded)`) and a failed point is dropped and counted rather than carried at OGR's `HUGE_VAL`. A layer that declares a spatial reference for which `OGRCreateCoordinateTransformation` returns null is skipped entirely with a `qWarning` naming the layer and its SRS, and counted in `ParseDiagnostics::layers_failed`; the layer reports it in its Layers-tab status. Verified by `VectorParseAttributes.LayerWithUnprojectableSrsFailsRatherThanFallingThrough`, over a GeoPackage layer in a `LOCAL_CS` holding projected metres: the fixture asserts the SRS is present (so it is not accidentally exercising the untransformed branch) and the parse emits no geometry at all — `src/camp_map/vector/vector_parse.cpp:45,222`
- [x] **4. No coordinate-validity guard before `geoToMap()`** — `01644e5`. `isPlaceable()` / `hasPlaceableCoordinate()` gate placement on `QGeoCoordinate::isValid()` (finite, lat ±90, lon ±180). Unplaceable vertices are dropped from a ring, a feature with no placeable vertex is skipped, and the layer logs the count and names it in its status. Verified by `VectorFeatureItem.PlaceabilityRejectsTheCoordinatesThatPoisonTheScene` (NaN, infinity, a UTM northing read as a latitude, the ±90/±180 boundaries) and `UnplaceableVerticesDoNotStretchTheItem`, which asserts the item's position and extent stay finite and local — `src/camp_map/vector/vector_feature_item.cpp:73`, `vector_layer.cpp:271`
- [x] **5. `wkbGeometryCollection` warn-and-dropped** — `d017d22`. It now shares the `Multi*` case and goes through the same `toGeometryCollection()` recursion that was already one line above it; nested collections recurse again. Verified by `VectorParseAttributes.GeometryCollectionIsNotDropped` over a GeoPackage `GEOMETRYCOLLECTION(point, line, polygon, GEOMETRYCOLLECTION(point))`: all four parts are emitted with their types, each carries the parent feature's attributes, and `geometries_unhandled` is zero — `src/camp_map/vector/vector_parse.cpp:147`
- [x] **6. The abort flag was read once, before `GDALOpenEx`** — `8025128` (parser hook in `d017d22`). `parseVectorLayers()` takes `ParseOptions::aborted` and polls it per layer and per feature; `VectorLayer` passes its own flag through, so `~VectorLayer()`'s join is bounded by the abort rather than by the size of the file. A partial parse is discarded rather than half-built. Verified two ways: `VectorParseAttributes.AbortStopsTheParseAndIsReported` (aborting before the first layer yields nothing; aborting after the first feature yields strictly fewer geometries than the full parse) and `VectorLayerTeardown.DestroyDuringLoadDoesNotWaitOutTheWholeParse`, which measures the full parse of a 200 000-feature file on the machine it runs on and asserts an aborted teardown finishes in under half that — `src/camp_map/vector/vector_layer.cpp:250`
- [x] **7. Unbounded GUI-thread item construction** — `8025128`. `VectorLayer::kMaxFeatureItems` (50 000) caps the items built, with the shortfall reported in the Layers-tab status and the log. The cap is a constructor parameter so a test can exercise it without a million-feature fixture. Verified by `VectorLayerTeardown.FeatureCapBoundsGuiThreadWorkAndIsReported`: a cap of 2 over a 5-feature file builds 2 items and the status names the 3 not drawn, while an uncapped layer over the same file reports plainly — `src/camp_map/vector/vector_layer.h:63`, `vector_layer.cpp:264`
- [x] **8. Degenerate-range guard was a no-op above 2^53** — `d2a6284`. `low = max - 1.0` does nothing wherever 1.0 is below the value's precision (OGR int64 ids, nanosecond timestamps), leaving a zero span and a NaN into `Palette::sample()` and the radius arithmetic. An equal-valued field now maps to the palette midpoint via `!(span > 0.0)`, which is total for every finite double and for a NaN span; an overflowing span falls back to the value's order relative to the bounds. Verified by `VectorLayerStyling.DegenerateRangeIsTotalAboveTwoToTheFiftyThree`, which asserts the fixture really is past the 1.0-ulp boundary and then that no NaN reaches the palette or the radius, plus a `-DBL_MAX..DBL_MAX` case — `src/camp_map/vector/vector_style.cpp:153`
- [x] **9. No regression test for the lat/lon-swap fix** — `d017d22`. Two tests, one per branch, asserting actual latitude and longitude of a LINE and a POLYGON vertex — the geometries the swap affected, and the ones nothing pinned: `LineAndPolygonVerticesAreLatLonWithoutAnSrs` (a GeoPackage layer created with no spatial reference, so the untransformed branch) and `LineAndPolygonVerticesAreLatLonWhenTransformed` (the WGS84 GeoPackage, whose target SRS returns authority lat/lon order), including a polygon's interior ring — `test/test_vector_parse_attributes.cpp`
- [x] **10. No ADR for the layer family or the persisted schema** — `4555fbb`. `docs/decisions/0016-read-only-vector-file-layer.md`, following the existing ADRs' Status / Context / Decision / Consequences form, with ADR-0007 as the layer-family precedent it deliberately departs from and ADR-0003 §4 as the app-state pattern it reuses. Twelve decisions (why not the raster render path, per-feature items, the shared parser's location, the parser/display split on placement policy, the read-only interaction contract, styling totality, the persisted schema, the single writer, the reorder-safe removal hook, remembering unreachable files, the two bounds, the pinned driver set) and the consequences taken knowingly — including the orphan settings group (a split precedent with MapTiles, recorded rather than resolved) and the antimeridian limitation.

Suggestions — all eleven actioned, in `4cf7b2b` unless noted:

- [x] A temporarily-unreachable persisted file is now remembered and carried forward rather than dropped after one launch (`m_unavailableVectorLayerFiles`, folded back in by the single writer).
- [x] `withoutVectorLayerFile()` removed outright (no production caller — the key is rebuilt from scratch on every change), with its test and the overclaiming CMake comment.
- [x] `viewInPanMode()` reads the drag mode of the view the event came from (`event->widget()->parentWidget()`), falling back to the attached views only for a synthesized event.
- [x] The popup fires on release without movement rather than on press.
- [x] `size_range` is folded over POINTS only — the only geometry size-by-field sizes.
- [x] Dedup / removal / persistence key on `canonicalFilePath()`.
- [x] `GDALOpenEx` is pinned to a list of file-based vector drivers, so an operator-supplied string cannot become a `/vsicurl/` fetch, a `PG:` connection or a KML NetworkLink.
- [x] `mission_insertion.h`'s null-parent contract comment now says what the code does (`RowInserter` dereferences the parent; a null result is not "insert at top level").
- [x] Docs corrected: click-to-inspect is a `QToolTip`, on release without movement, only in pan mode (`.agents/README.md`, `vector_layer.h`) — `4555fbb`.
- [x] The orphan `MapItem/file:<path>` group on removal: recorded as a deliberate choice with its cost in ADR-0016's Consequences (follow `RasterLayer`, so re-adding a file restores its style) rather than silently following one side of the split precedent — `4555fbb`.
- [x] Cross-antimeridian geometry: documented as a known limitation in ADR-0016 and in `vector_layer.h` — `4555fbb`.

### Notes for the re-review

- Findings 2 and 4 landed in ONE commit (`01644e5`): both live in `vector_feature_item`, share its `boundingRect()`, and share the new `test_vector_feature_item` target, so splitting them would have produced two commits neither of which built its own test.
- The interaction change (release-gated popup) fixes the unrequested-tooltip half of that suggestion. The other half — that accepting the press at all takes the pan gesture over a feature — is NOT fixed: an item that ignores the press receives no release, so gating on release requires holding the press. The proper fix is a ProjectView-side click handler, which camp_map cannot reach (#217) and which is a UI change worth its own issue. Recorded here rather than left implicit.
- `normalizedValue()`'s documented contract CHANGED: a degenerate range now returns 0.5, not 1.0. The existing `EqualValuesMapToTheTopOfTheRamp` test was renamed and re-pointed accordingly — that is a deliberate behaviour change from must-fix 8, not a test relaxed to pass.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-14 13:48 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-22 at `c873d9a`
**Mode**: pre-push
**Depth**: Deep (reason: 5168 added lines across 31 files; async worker, persisted app state, new ADR)
**Must-fix**: 2 | **Suggestions**: 9
**Round**: 2 | **Ship**: recommended — must-fix fell 10 to 2, both are precise mechanical corrections (a security claim the code does not deliver, and a cap that protects the GUI thread but not memory) with no design question left open.

Specialists: Static Analysis (pre-commit all-passed; cppcheck clean but for
`useStlAlgorithm` style nits), Governance, Plan Drift, Claude Adversarial
Lens A + Lens B. Copilot and Local Model off (default).

Build/test reproduced in this worktree: `./ui_ws/build.sh camp` clean;
`./ui_ws/test.sh camp` — **367 tests, 0 errors, 0 failures, 1 pre-existing
skip**, matching the Implementation entry's claim.

Round-1 verification: **all ten must-fixes are genuinely fixed in code and
covered by a test that exercises the real failure**, not a restatement — the
reorder test drives an actual `Map::setMapItemParent()` reorder, the abort
test measures a real parse against an aborted teardown, and the degenerate-range
test asserts its fixture is past the 1.0-ulp boundary before testing. All
eleven suggestions were actioned. The fix commits introduced no correctness,
concurrency or lifetime defect; the two new must-fixes below are adjacent
concerns neither round-1 finding named.

Rulings on the three items the implementer flagged:

- **(a) Findings 2 and 4 in one commit (`01644e5`)** — accepted, not a finding.
  Both change `vector_feature_item`'s `boundingRect()` and both are proven by
  the `test_vector_feature_item` target that same commit creates; splitting
  would have produced a commit whose test does not yet exist.
- **(b) The press-accept still swallows the pan gesture over a feature** —
  acceptable to ship with a filed follow-up; NOT a must-fix. The behaviour is
  real (`QGraphicsView::mousePressEvent` returns early once the scene accepts
  the press, so hand-scroll pan cannot start on a feature), but it is
  pre-existing app-wide CAMP behaviour rather than something this branch
  introduces: `TaskOverlayItem::mousePressEvent`
  (`src/camp/running_tasks/task_overlay_item.cpp:147`) consumes the press the
  same way, as do the movable mission `GeoGraphicsItem`s. The implementer's
  premise also holds — an item that ignores the press never receives the
  release — so the clean fix is a ProjectView-side click handler outside
  `camp_map`'s reach. Condition: file the issue in rolker/camp before opening
  the PR and reference it in the PR body and at
  `vector_feature_item.cpp:244`, so it does not live only in progress.md.
- **(c) `normalizedValue()`'s degenerate range 1.0 to 0.5** — confirmed the
  intended behaviour change, not a relaxed test. The contract is stated
  consistently in `vector_style.h:58-67`, the implementation comment and
  ADR-0016 D6; the renamed test asserts 0.5 AND the palette colour sampled at
  0.5, and the new `DegenerateRangeIsTotalAboveTwoToTheFiftyThree` pins the
  defect the rename came from. A strengthened test, not a weakened one.

Governance: principles all Pass or N/A; ADR-0016 D1-D11 verified line-by-line
against the code with no mismatch (D12 is the exception — must-fix 1 below).
Plan adherence (rev 5): 1:1 on files, no silent drops; only `mainwindow.h` /
`mainwindow.ui` changed without being listed, which the planned
`mainwindow.cpp` wiring implies.

### Findings
- [x] (must-fix) The driver allowlist does not deliver the property its comment and ADR-0016 D12 claim: GDAL resolves `/vsicurl/`, `/vsizip/`, `/vsis3/` in the VSI layer BEFORE driver selection, so `/vsicurl/https://.../x.geojson` still fetches through the allowed GeoJSON driver — empirically confirmed on this host's GDAL 3.8.4 with exactly `kAllowedDrivers`; `KML`/`LIBKML` are on the list too, so the NetworkLink example does not hold either, and `restorePersistedVectorLayers()` re-opens every persisted path at startup with no confirmation. Reject a `/vsi` prefix and correct the claim to what holds (`PG:` and the other non-file drivers) — `src/camp_map/vector/vector_layer.cpp:39` and `:93`, `docs/decisions/0016-read-only-vector-file-layer.md:162`
- [x] (must-fix) The feature cap is applied after `parseVectorLayers()` has already materialised every geometry and attribute map of the whole file in the worker, so for the case the header and ADR D11 name — a national coastline shapefile — the GUI no longer freezes but the matching OOM is unprotected and undocumented; carry the cap into `ParseOptions` (already threaded through and polled per feature) or state the memory limitation — `src/camp_map/vector/vector_layer.cpp:104`, `vector_layer.h:60`
- [x] (suggestion) The release-gated popup still accepts the press, taking the pan gesture over a feature — ruling (b): ship it, but file the ProjectView follow-up issue and reference it here and in the PR body — `src/camp_map/vector/vector_feature_item.cpp:244`
- [x] (suggestion) A polygon with a null exterior ring is dropped with no diagnostic increment, against `vector_parse.h:82`'s "every field counts something that was deliberately dropped" — `src/camp_map/vector/vector_parse.cpp:143`
- [x] (suggestion) The widget-less release fallback compares a scene-metre delta against a device-pixel threshold; the branch is unreachable in the shipped UI and untested (the test always sets a widget) — `src/camp_map/vector/vector_feature_item.cpp:272`
- [x] (suggestion) `PopupIsGatedOnPanModeAndOnAClickNotADrag` asserts only `isAccepted()`, which is true on both the click and the drag path, so it cannot distinguish tooltip-shown from tooltip-suppressed; assert `QToolTip::text()` — `test/test_vector_feature_item.cpp:203`
- [x] (suggestion) The test comment says a null result means "the caller inserts at top level"; `mission_insertion.h:31` says explicitly that it does not — contradictory documentation of one contract — `test/test_mission_insertion.cpp:54`
- [x] (suggestion) `openGeometry()` is itself the caller the new header comment says must check for a null parent, and does not — `RowInserter` receives the unchecked pointer — `src/camp/autonomousvehicleproject.cpp:218`
- [x] (suggestion) `WaitCursor` brackets only the synchronous kickoff; the load it appears to represent runs after `unsetCursor()` — `src/camp/mainwindow.cpp:626`
- [x] (suggestion) `!range.valid` still returns 1.0 while a degenerate range now returns 0.5 — an asymmetry with no reader, since both callers guard `range.valid` first — `src/camp_map/vector/vector_style.cpp:54`
- [x] (suggestion) Unavailable persisted files are folded in at the END of the rebuilt list, so a layer's position is lost across a launch with the share unmounted — `src/camp/autonomousvehicleproject.cpp:426`

## Implementation
**Status**: complete
**When**: 2026-09-14 14:14 -04:00
**By**: Claude Opus

**Branch**: feature/issue-22 at `3036afb`
**Addressed**: `## Local Review (Pre-Push)` round 2 — 2026-09-14 13:48 -04:00, branch at `c873d9a` (ship: recommended; 2 must-fixes, 9 suggestions)
**Commits**: 5d17a0c, 1437019, 0528837, ca83f27, 57e2c2e, bc6ddbc, 6550bcb, 0ca4004, 6b18d50, 3671436, 9278d4f, 3036afb

**Build/test**: `./ui_ws/build.sh camp` clean; `./ui_ws/test.sh camp` → **370 tests, 0 errors, 0 failures, 1 skipped** (367 before — three new tests).

### Actions
- [x] (must-fix) Driver allowlist does not stop `/vsi` network fetches; the ADR D12 / code-comment claim is wrong; `restorePersistedVectorLayers()` reopens every persisted path unattended — `src/camp_map/vector/vector_layer.cpp`, `docs/decisions/0016-read-only-vector-file-layer.md` → **5d17a0c**. New `camp::vector::isVirtualFileSystemPath()` refuses any `/vsi` prefix (trimmed, case-insensitive) in `VectorLayer`'s constructor *before* anything is opened, and in `AutonomousVehicleProject` on both the open path (so a refused path never reaches `vectorLayers/files`) and the restore path (a persisted `/vsi` entry is dropped, not retried each launch). The allowlist comment and ADR D12 now claim only what holds: the list excludes non-file drivers (`PG:`, `WFS:`, `OAPIF:`, …); the `/vsi` refusal is what blocks remote fetches; a KML NetworkLink is recorded as a remaining limitation, with the two ways to close it named. *Verified*: new `VectorLayerTeardown.VirtualFileSystemPathsAreRefusedWithoutOpening` — `/vsicurl/`, `/vsizip/`, `/vsis3/` and a whitespace-padded `/vsicurl/` all report `(refused: not a local file)`, build no features, and leave `GDALDataset::GetOpenDatasets()` at its baseline (a path that reached GDAL would have gone to the network from a unit test); plus direct predicate assertions including the negative `/data/vsicurl/x.geojson`.
- [x] (must-fix) Feature cap applied after `parseVectorLayers()` had materialised the whole file, leaving the documented OOM unprotected — `src/camp_map/vector/vector_layer.cpp`, `vector_layer.h`, ADR D11 → **1437019**. `ParseOptions::max_geometries` (0 = unlimited) is checked per feature beside the abort predicate already polled there; the parse stops at the cap and the rest of the file is never read. A multi-part feature can carry the running total past the cap, so the excess is trimmed and exactly the cap comes back. `ParseDiagnostics::geometry_cap_reached` reports the stop; the layer's status and log now say the cap was hit and that the rest of the file went unread, rather than naming a shortfall count that can no longer be known (counting it means reading the file the cap exists to stop reading, and an OGR feature count is not a geometry count). The GUI-side bound stays as a backstop. Header and ADR D11 rewritten to say the cap bounds both halves and why. *Verified*: new `VectorParseAttributes.GeometryCapStopsTheParse` — a cap of 2 on a fixture holding more returns exactly 2 with `geometry_cap_reached` set and `aborted` clear, and a cap above the file's size returns everything with the flag clear; `VectorLayerTeardown.FeatureCapBoundsGuiThreadWorkAndIsReported` updated to assert the new status wording.
  - Consequence handled in the same commit: `DestroyDuringLoadDoesNotWaitOutTheWholeParse` measured its "full parse" baseline through a cap-1 layer, which no longer reads the whole file. It now times `parseVectorLayers()` directly (no cap, no GUI items) and aborts a layer whose cap is *above* the 200 000-feature fixture, so only the abort can end that parse early. Passes, 1.7 s.
- [x] (suggestion) Release-gated popup still accepts the press, taking the pan gesture — ruling (b), file the ProjectView follow-up — `src/camp_map/vector/vector_feature_item.cpp` → **9278d4f**. Filed as [camp#225](https://github.com/rolker/camp/issues/225) ("Vector feature item takes the pan gesture by accepting the press") and referenced from the code comment and ADR-0016's consequences. **Owed to the PR body**: the review asked for the reference there too — the PR does not exist yet (the host creates it), so it must carry a line pointing at camp#225.
- [x] (suggestion) Polygon with a null exterior ring dropped without a diagnostic — `src/camp_map/vector/vector_parse.cpp` → **0528837**. New `ParseDiagnostics::polygons_without_exterior_ring`, warned per layer in the parser and reported in the layer's log beside the other diagnostics. *Verified*: new `VectorParseAttributes.PolygonWithoutExteriorRingIsCounted` builds a Memory-driver layer with one empty and one ringed polygon; one geometry comes back, the counter reads 1, and `geometries_unhandled` stays 0 (an empty polygon is dropped, not unhandled).
- [x] (suggestion) Widget-less release fallback compares scene metres against a pixel threshold; unreachable and untested — `src/camp_map/vector/vector_feature_item.cpp` → **ca83f27**. The release event carries its own button-down screen position, so the pixel comparison needs no widget and the second branch is gone — one unit, screen pixels, at every zoom. `press_scene_pos_` existed only for that branch and was removed with it.
- [x] (suggestion) `PopupIsGatedOnPanModeAndOnAClickNotADrag` asserts only `isAccepted()` — `test/test_vector_feature_item.cpp` → **57e2c2e**. The feature carries an attribute so the popup has text; the click asserts `QToolTip::text() == item->attributeText()`, the drag asserts an unchanged sentinel. (Emptiness does not work: `QToolTip::hideText()` does not clear `QToolTip::text()` on the offscreen platform — verified by running it — so an emptiness assertion would test the platform, not the item. The sentinel tests the item.)
- [x] (suggestion) Test comment contradicts `mission_insertion.h` on what a null result means — `test/test_mission_insertion.cpp` → **bc6ddbc**. The comment now states the header's contract: a null result is not "insert at top level"; `RowInserter` dereferences the parent, so the caller must check.
- [x] (suggestion) `openGeometry()` is the caller the header says must null-check, and does not — `src/camp/autonomousvehicleproject.cpp` → **6550bcb**. It warns (naming the file) and declines rather than handing `RowInserter` a null pointer. Not operator-reachable — `m_currentGroup` is set in the constructor and never cleared — so this is a programming-error guard, which is what the header describes.
- [x] (suggestion) `WaitCursor` brackets only the synchronous kickoff — `src/camp/mainwindow.cpp` → **6b18d50**. Removed: the load is asynchronous and the layer reports `(loading...)` in the Layers tab, so the cursor signalled waiting during the part that does not wait and was gone for the part that does.
- [x] (suggestion) `!range.valid` returns 1.0 while a degenerate range returns 0.5 — `src/camp_map/vector/vector_style.cpp` → **3671436**. Aligned on 0.5: both mean "nothing distinguishes these features". Header updated and the contract pinned by a new assertion in `NormalizationSpansTheRange`.
- [x] (suggestion) Unavailable persisted files folded in at the END, losing layer position — `src/camp/autonomousvehicleproject.cpp` → **0ca4004**. The list as read at restore is kept (`m_restoredVectorLayerOrder`) and the key is rewritten in that order, so an unreachable entry keeps its slot; anything opened since is appended in load order, and an entry that is now neither loaded nor unavailable was removed through the Layers tab and stays removed. ADR D10 updated. No unit test: `persistVectorLayers()` lives in a TU no test can construct (the harness note at the top of `test_vector_layer_persistence.cpp` records this seam); verified by reading the source and by the ordering rules already covered there for `withVectorLayerFile()`.

### Notes
- Plan kept in sync: **3036afb** adds plan rev 6 — the `/vsi` refusal replaces the driver allowlist as the claim about network fetches, and the cap moves into the parse. ADR-0016 D10, D11, D12 and the consequences list were updated in the commits that changed the behaviour.
- Nothing deferred; all eleven findings were actioned.
- Not pushed (sub-agent contract). Next: `review-code` re-review.

## Integrated Review
**Status**: complete
**When**: 2026-09-14 14:58 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #226 at `507ee23`
**Sources**: 3 (Copilot R1 @ `507ee23` — 8 inline + 3 suppressed comments; Local Review (Pre-Push) @ `0cfa138`; Local Review (Pre-Push) @ `c873d9a`; CI rollup)
**Cross-source confirmations**: 3
**CI**: all-pass (`build-and-test` SUCCESS on `507ee23`); no human review comments, no conversation comments

### Findings
- [x] (cross-confirmed, must-fix) Add-* placement is not actually gated: `ProjectView::mousePressEvent()` places the waypoint, calls `setPanMode()` (drag mode -> `ScrollHandDrag`) and only THEN forwards to `QGraphicsView::mousePressEvent()`, so by the time the scene dispatches the press `viewInPanMode()` reads pan, the item accepts it, and the release pops an attribute tooltip on top of the just-placed waypoint. The same holds for the second click of add-survey-pattern and add-search-pattern (`projectview.cpp:71`, `:100`, `:138`). This defeats the gating `vector_feature_item.cpp:244`'s comment and ADR-0016 claim. The fix belongs in `ProjectView` (defer the mode switch until after the forward, or capture the mode first) — the same seam camp#225 already owns — `src/camp/projectview.cpp:71`, `src/camp_map/vector/vector_feature_item.cpp:260`
- [x] (cross-confirmed, must-fix) An unavailable-at-startup path is never cleared from `m_unavailableVectorLayerFiles`, so once the file comes back and the operator opens it manually, removing that layer through the Layers tab does NOT stick: `persistVectorLayers()` still finds the entry on the unavailable branch and re-writes it, and the layer returns on every launch — camp#90/#117's exact bug class, which this PR set out not to repeat. Fix: drop the entry in `openVectorLayer()` once the file has been confirmed to exist, keeping still-missing paths; add the remove-after-reopen regression test — `src/camp/autonomousvehicleproject.cpp:413`
- [x] (cross-confirmed, should-fix) The `isPlaceable()` guard added for round-1 must-fix 4 is `QGeoCoordinate::isValid()`, which admits latitude +/-90. `web_mercator::geoToMap()` is finite there (tan(pi/2) is 1.633e16 in double, not inf), but it yields y = +/-2.425e8 m — about 12x the Web-Mercator world half-extent of 2.004e7 m — so a polar feature still blows out `childrenBoundingRect()`, fit-to-extent and the scene index. Copilot's "infinite" is wrong; the extent poisoning is real. Nothing in camp clamps to `web_mercator::maximum_latitude` (85.0511 deg), so whether to clamp, drop, or document polar features is a decision, not a one-liner — `src/camp_map/vector/vector_feature_item.cpp:73`
- [x] (should-fix, Copilot) `noDataColor()` returns a fixed `(128,128,128)`; its own comment claims distinctness only from "both ends of every shipped palette". `resolvePalette()` falls back to — and the operator can select — `grayscale`, whose midpoint samples to that same grey, so a missing value becomes indistinguishable from a mid-range measurement. Resolve the no-data appearance against the active palette, or carry it in a channel the palette does not use (outline/pattern) — `src/camp_map/vector/vector_style.cpp:95`
- [x] (suggestion, Copilot) Persisted vector-layer ORDER does not follow a Layers-tab drag: `m_vectorLayers` is appended only in `openVectorLayer()` and `Map::setMapItemParent()`'s reorder deliberately does not reach `removedFromMap`, so `vectorLayers/files` keeps the load order and the next launch restores it. Round-1's fix stopped a reorder from UN-persisting a layer; it did not make the reorder persist. Note this is not a regression: `persistBackgrounds()` has the identical limitation for chart layers, so the honest remedy is one follow-up issue rebuilding both sequences from the map's current top-level child order — `src/camp/autonomousvehicleproject.cpp:474` (deferred: not the small change the operator's condition named — the live order comes from the map's top-level children while an unavailable-at-startup entry has no map item and holds its slot from the restored order only, so the two sequences have to be merged; `persistBackgrounds()` has the identical limitation for charts. Follow-up issue text recorded in the Implementation entry below for the host to file.)
- [x] (suggestion, Copilot) The cap and abort predicate are polled per FEATURE, but `appendGeometry()` recurses through every part of a multi-part feature (and every nested collection), copying the attribute map per part, before returning to either check. Memory overshoot is bounded by one feature and is trimmed by the existing excess-resize, but abort latency for a single huge `MultiPolygon` / `GeometryCollection` is unbounded — the GUI-thread join this PR's round-2 work exists to bound. Thread the remaining budget and the abort predicate into the recursion — `src/camp_map/vector/vector_parse.cpp:180`
- [x] (suggestion, Copilot) `isVirtualFileSystemPath()` matches the raw four characters `/vsi`, so an ordinary local directory such as `/vsidata/survey.geojson` is refused outright and cannot be opened or persisted. A prefix-BOUNDARY check does not fix this (`/vsidata/` still matches `/vsi<word>/`); matching GDAL's registered handler prefixes (`VSIGetFileSystemsPrefixes()`) does. The current cost is a loud refusal rather than a silent failure, which is why this is a suggestion — `src/camp_map/vector/vector_layer.cpp:74`
- [x] (suggestion, Copilot) `ASSERT_GT(full_ms, 200)` makes a correct implementation fail on fast hardware: it is a wall-clock LOWER bound on parsing the 200 000-point fixture, asserted only so the `abort_ms < full_ms/2` comparison is meaningful. Make the guard a skip/diagnostic, or bound the comparison on an observable partial-parse condition instead — `test/test_vector_layer_teardown.cpp:316`
- [x] (suggestion, Copilot) `test_vector_layer_persistence.cpp` uses `QDir` at lines 116-117 with no direct `#include <QDir>`; it compiles today only transitively (via `<QTemporaryDir>`/`<QApplication>`). Add the header — `test/test_vector_layer_persistence.cpp:26`
- [x] (suggestion, integrator) ADR-0016 D12's "Known limitation" and the `kAllowedDrivers` comment both assert that a KML `NetworkLink` may be followed during `GDALOpenEx`. Measured against this host's GDAL 3.8.4 (the version CAMP builds on), neither the `KML` nor the `LIBKML` driver follows one — with a relative local href or an `http://127.0.0.1:9/` href, the open returns only the local placemark, instantly and with no fetch attempt. AGENTS.md requires documentation claims to be verified rather than assumed; correct the wording to what holds (the drivers do not resolve NetworkLink hrefs on open in GDAL 3.8.x; a future GDAL could, which is why the drivers stay named) — `docs/decisions/0016-read-only-vector-file-layer.md:198`, `src/camp_map/vector/vector_layer.cpp:47`

### False positives
- (Copilot, `src/camp/autonomousvehicleproject.cpp:438`, flagged critical) "`destroyed` can invoke `onVectorLayerDestroyed()` during `AutonomousVehicleProject` teardown, after `m_vectorLayers` is gone." `~QObject` removes every connection in which the object is the RECEIVER before it calls `deleteChildren()`, so the child `Map`'s `VectorLayer`s are destroyed with no connection to the slot left. Verified empirically on this host's Qt 5.15.13 with a minimal parent/child/`destroyed`-slot program: the slot does not fire. The member-vs-child destruction order Copilot describes is correct; the conclusion does not follow from it.
- (Copilot, `src/camp_map/vector/vector_layer.cpp:60`, flagged critical) "A local KML can carry a `NetworkLink` which the driver may follow and fetch during `GDALOpenEx`, bypassing the `/vsi` check, including on unattended startup restore." Tested directly against GDAL 3.8.4 with both drivers and both href forms (see the ADR finding above): the NetworkLink target is not read and no connection is attempted. The failure mode does not occur with the GDAL CAMP builds against. Recorded as a documentation finding rather than dismissed, because the repo's own text makes the same unverified claim.

## Implementation
**Status**: complete
**When**: 2026-09-15 08:39 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #226 at `71e56c0`
**Branch**: feature/issue-22 at `71e56c0`
**Addressed**: `## Integrated Review` (2026-09-14 14:58 -04:00, PR #226 at `507ee23`) — all 10 findings
**Commits**: `e2a56cc`, `a3c5117`, `3e7606e`, `403d235`, `3a2d903`, `114426b`, `71b8a81`, `80f6514`, `158dab0`, `71e56c0`

Operator decision (run-issue checkpoint 3): address all 10 findings in this PR,
clamp polar latitudes rather than dropping them, carry no-data in a channel the
palette does not use, and fix the persisted layer ORDER here only if it is small.

**Build/test**: `./ui_ws/build.sh camp` clean; `./ui_ws/test.sh camp` — 376 tests,
0 errors, 0 failures, 1 skipped (up from 375: three new tests, one existing test
extended).

### Actions
- [x] (must-fix) Add-* placement gating — `ProjectView::mousePressEvent()` now
  DEFERS its switch back to pan mode until after the press has been forwarded to
  `QGraphicsView`, so the scene dispatches the press under the mode the operator
  clicked in and `viewInPanMode()` no longer reads "pan" mid-placement. The three
  left-button sites and the right-button cancel all set one local flag that is
  consumed after the forward. The coupling is recorded from both sides
  (`vector_feature_item.cpp`'s gate comment and ADR-0016 D5), because it is
  invisible from `camp_map`. No new test: `ProjectView` is not constructible in a
  test harness (the same seam limit as `AutonomousVehicleProject`), and the
  item-level contract it depends on is already asserted by
  `VectorFeatureItem.PopupIsGatedOnPanModeAndOnAClickNotADrag`; the ordering
  itself is verified by reading the source, the precedent `test_mission_insertion`
  sets — `src/camp/projectview.cpp:57`, `src/camp_map/vector/vector_feature_item.cpp:260`
- [x] (must-fix) Unavailable-at-startup path never cleared —
  `openVectorLayer()` now drops a path from `m_unavailableVectorLayerFiles` as
  soon as `QFileInfo::exists()` confirms it, so a LATER removal through the Layers
  tab sticks instead of being written back on the unavailable branch every launch.
  Still-missing paths keep their entries, which is what carries an unmounted share
  across a session. The rebuild rule itself was extracted to
  `camp::vector::rebuildPersistedVectorLayerFiles()` so it can be exercised at all
  (`AutonomousVehicleProject` is not constructible in a harness), and the
  remove-after-reopen lifecycle is now a regression test, with a counterpart
  asserting a still-missing entry is still carried forward —
  `src/camp/autonomousvehicleproject.cpp:413`,
  `src/camp_map/vector/vector_layer.cpp:449`,
  `test/test_vector_layer_persistence.cpp:218`
- [x] (should-fix) Polar latitude — clamped, per the operator's decision, not
  dropped. Every geometry conversion in `vector_feature_item.cpp` (ring vertices
  AND the item's anchor position) goes through the new `placeableToMap()`, which
  clamps latitude to `web_mercator::maximum_latitude` (85.0511°) before
  projecting. The anchor mattered as much as the vertices: clamping only the path
  would have left the item's POSITION 2.4e8 m out with an innocent-looking local
  path. Documented where the conversion happens and as ADR-0016 D13. Two tests:
  the conversion, and that an item built from polar geometry stays inside the
  Mercator world — `src/camp_map/vector/vector_feature_item.cpp:79`,
  `test/test_vector_feature_item.cpp:163`
- [x] (should-fix) No-data appearance — carried in channels the palette does not
  touch, per the operator's decision. `camp::vector::isNoData()` names the state
  (exactly the branch `colorForValue()` answers with `noDataColor()`, so the two
  cannot drift), `VectorLayer` sets it on each feature, and `VectorFeatureItem`
  draws a no-data feature with a DASHED outline and a HATCHED fill — a hollow
  marker for a point. Those survive grayscale (whose midpoint is the same grey),
  colour-blind vision and a monochrome printout. ADR-0016 D6 updated; the old
  "not a position on any palette" claim, which grayscale falsified, is gone —
  `src/camp_map/vector/vector_style.cpp:95`,
  `src/camp_map/vector/vector_feature_item.cpp:290`,
  `src/camp_map/vector/vector_layer.cpp:330`
- [x] (suggestion) Persisted layer ORDER does not follow a Layers-tab drag —
  **deferred** to a follow-up issue, which is the branch the operator's condition
  names. It is not the small change described: the live order lives in the map's
  top-level child list, while an unavailable-at-startup entry has no map item at
  all and holds its slot only from the restored order, so the two sequences have
  to be MERGED (which slot does a missing layer occupy once the live order has
  changed?) — and `persistBackgrounds()` has the identical limitation for chart
  layers, which is what makes one issue covering both the honest remedy rather
  than a vector-only patch. Proposed issue text below for the host to file —
  `src/camp/autonomousvehicleproject.cpp:474` (deferred: scoped as a follow-up per
  the operator's "otherwise file a camp follow-up issue" condition)
- [x] (suggestion) Cap/abort not polled inside a multi-part feature — a
  `ParseBudget` (remaining geometries + the abort predicate) is now threaded
  through `appendGeometry()`'s collection recursion and checked before every part
  and spent on every emission, so a huge `MultiPolygon` / nested
  `GeometryCollection` stops mid-feature. Abort latency — the thing the predicate
  exists for, since `VectorLayer`'s destructor joins this worker on the GUI thread
  — is now bounded by one geometry rather than by one feature. The post-hoc excess
  trim is kept as a belt-and-braces no-op and its comment says so. New test drives
  cap and abort against a one-feature, four-part fixture —
  `src/camp_map/vector/vector_parse.cpp:180`, `test/test_vector_parse_attributes.cpp:290`
- [x] (suggestion) `/vsi` refusal too broad — `isVirtualFileSystemPath()` now
  matches GDAL's registered handler prefixes (`VSIGetFileSystemsPrefixes()`)
  instead of the raw four characters, so an ordinary local directory such as
  `/vsidata/survey.geojson` opens while every real virtual file system is still
  refused. Confirmed on this host's GDAL 3.8.4: 26 prefixes registered, none of
  them `/` — a bare local path cannot match. The empty-list path falls back to the
  old broad refusal rather than letting one through. Tests cover both directions,
  including the archive and in-memory handlers —
  `src/camp_map/vector/vector_layer.cpp:74`, `test/test_vector_layer_teardown.cpp:234`
- [x] (suggestion) `ASSERT_GT(full_ms, 200)` fails on fast hardware — replaced as
  the load-bearing assertion by an OBSERVABLE partial-parse condition: parse the
  200 000-point fixture with an abort predicate that trips after ten polls and
  assert `diagnostics.aborted` and a geometry count below the file's. That means
  the same thing on every machine. The wall-clock ratio is kept as a weaker second
  signal (it catches an abort that is honoured but arrives late) and is now
  reported-and-skipped, not failed, when the full parse is under 200 ms —
  `test/test_vector_layer_teardown.cpp:316`
- [x] (suggestion) Missing `#include <QDir>` — added —
  `test/test_vector_layer_persistence.cpp:26`
- [x] (suggestion) ADR-0016 D12 / `kAllowedDrivers` KML `NetworkLink` claim —
  corrected to what holds, after re-measuring it here rather than taking the
  triage's word: GDAL 3.8.4, both `KML` and `LIBKML`, a `NetworkLink` with a
  relative local `href` and again with `http://127.0.0.1:9/` — each open returns
  only the local placemark, immediately, with no fetch attempted. Both texts now
  say that, say it is a measurement against a version, and tell whoever bumps GDAL
  to re-measure rather than re-derive —
  `docs/decisions/0016-read-only-vector-file-layer.md:198`,
  `src/camp_map/vector/vector_layer.cpp:47`

### Follow-up issue for the host to file (camp)

**Title**: Persisted layer order should follow a Layers-tab drag, for vector and chart layers alike

**Body**:

Both `vectorLayers/files` and `backgrounds/files` record the order layers were
LOADED in, not the order they are in now. A drag-reorder in the Layers tab is
implemented by `Map::setMapItemParent()` as a detach-and-reinsert that
deliberately never reaches `Layer::onRemovedFromMap()` — which is what keeps a
reorder from UN-persisting a layer (camp#22 must-fix 1) — so nothing tells the
project the sequence changed. The operator drags a layer, quits, relaunches, and
the old order is back.

This is not a regression from camp#22: `persistBackgrounds()` has had the same
limitation since ADR-0003 §4. Fixing one and not the other would leave two layer
families behaving differently in the same tab, so this covers both.

The remedy is to rebuild each sequence from the map's current top-level child
order rather than from the tracked-list append order. The part that needs a
decision is the interaction with unavailable-at-startup entries: a persisted file
that could not be opened has no map item to read a position from, and currently
holds its slot from `m_restoredVectorLayerOrder` (ADR-0016 D10). When the live
order has changed around it, which slot is it in? Answer that first; the
enumeration itself is small.

Raised by the 2026-09-14 Integrated Review on camp#226 (Copilot), and deferred
there by operator decision because the change is larger than the one-liner the
checkpoint allowed inline.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-15 08:48 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-22 at `8bb6469`
**Mode**: pre-push
**Depth**: Deep (reason: 32 files / +6486-236, new ADR, cross-layer camp ↔ camp_map, GDAL input handling)
**Must-fix**: 2 | **Suggestions**: 6
**Round**: 3 | **Ship**: recommended — must-fix count flat vs round 2 (2 → 2), both are one-line documentation corrections with an obvious fix; no code defect survived the fix pass.

Specialists: Static Analysis (cppcheck + repo pre-commit, both clean), Governance,
Plan Drift, Claude Adversarial Lens A + Lens B. Copilot and local review off (default).
Build and tests re-run here: `./ui_ws/build.sh camp` clean, `./ui_ws/test.sh camp`
→ **376 tests, 0 errors, 0 failures, 1 skipped**, matching the Implementation entry's claim.
All 11 fix-pass commits carry the agent identity; working tree clean.

### Findings
- [x] (must-fix) Plan is 11 commits stale: the Files-to-Change row still says `projectview.{h,cpp}` "Not needed (rev 4)" and Open Question 2 still says "ProjectView is untouched", both falsified by `e2a56cc`; step 5's no-data text still describes a colour-only answer and step 1 predates `ParseBudget`; no mention of the polar clamp (D13) or the deferred layer-order item — `.agent/work-plans/issue-22/plan.md:563`, `:625`
- [x] (must-fix) Doc comment cites the wrong code: `RowInserter`'s unconditional parent dereference is at `autonomousvehicleproject.cpp:1473-1480`, not `:1386-1393` (which is `openProject`/`readChildren`) — a reference other agents will follow to the wrong function — `src/camp/mission_insertion.h:33`
- [x] (suggestion) `openVectorLayer()` drops the path from `m_unavailableVectorLayerFiles` before the `if(!layers) return;` guard, so on that (defensive, effectively unreachable) path the entry is neither unavailable nor loaded and the next rebuild silently forgets it; move the `removeAll` to after the layer is tracked, or note the guard-order dependency — `src/camp/autonomousvehicleproject.cpp:421`
- [x] (suggestion) `applyStyle()`'s no-data wiring is untested — `isNoData()` is covered as a free function, but nothing asserts that the layer sets it, including the `color_field_.isEmpty()` branch that clears it — `src/camp_map/vector/vector_layer.cpp:360-377`
- [x] (suggestion) No per-attribute size bound: the feature cap bounds geometry count, but a single feature carrying a multi-hundred-MB string property is read whole — low severity for an operator-chosen local file, worth a line in ADR-0016 D11 — `src/camp_map/vector/vector_parse.cpp:239` (deferred: an ADR decision on what to do with an oversized property — truncate, skip the feature, or accept it — not a mechanical edit; host's scope call for this round was cheap-and-decision-free only)
- [x] (suggestion) The KML `NetworkLink` safety claim is a measurement against GDAL 3.8.4 with nothing but comment discipline to catch a regression on a version bump; a test that fails loudly if a NetworkLink is ever followed would make it self-enforcing — `src/camp_map/vector/vector_layer.cpp:47`, `docs/decisions/0016-read-only-vector-file-layer.md:219` (deferred: needs a fixture and a decision on how a NetworkLink regression should fail; beyond this round's scope)
- [x] (suggestion) Two already-drafted follow-ups are still unfiled in `rolker/camp`: the persisted layer-order issue (full text in the Implementation entry) and the "Open Geometry" vs "Open Vector Layer" menu-label near-collision the ADR names as a live operator footgun — `docs/decisions/0016-read-only-vector-file-layer.md:259` (deferred: the host files both issues, per this round's scope)
- [x] (suggestion) `e2a56cc` changes mission-placement dispatch order for **all six** add-* modes, not just the vector-layer interaction; correctly generalised and no path bypasses the deferred switch, but the PR body should say so, since it is not vector-layer-scoped — `src/camp/projectview.cpp:57-211` (deferred: the PR body is the host's to edit; the same fact is now in plan rev 7 and ADR-0016 D5)

## Implementation
**Status**: complete
**When**: 2026-09-15 09:41 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-22 at `b63ca23`
**PR**: #226
**Addressed**: `## Local Review (Pre-Push)` (2026-09-15 08:48 -04:00, round 3, branch at `8bb6469`) — both must-fixes and two of the six suggestions; the other four deferred with reasons below, per the host's scope for this round (cheap, no design decision; the host files the follow-up issues).
**Commits**: `aa98cc7`, `ef38568`, `8abaa9f`, `b63ca23`

**Build/test**: `./ui_ws/build.sh camp` clean; `./ui_ws/test.sh camp` — **377 tests,
0 errors, 0 failures, 1 skipped** (up from 376: one new test).

### Actions
- [x] (must-fix) Plan brought to **rev 7** — a new revision-history block records the
  round-3 fix pass's plan-level consequences (ProjectView is touched and why; no-data
  as a second channel rather than a colour, with rev 1's "distinct palette position"
  claim retired because grayscale falsifies it; the `ParseBudget` threaded into the
  geometry recursion; the polar clamp / ADR-0016 D13; the deferred layer-order
  follow-up). The struck-out `projectview` row in Files to Change is replaced by what
  actually changed and says it is not vector-layer-scoped; Open Question 2 keeps its
  answer (no accessor needed) but its "ProjectView is untouched" claim is retired;
  step 1 gains the per-geometry budget; step 4 gains the clamp; step 5's no-data
  bullet is rewritten — `.agent/work-plans/issue-22/plan.md:9`, `:639`, `:701`
- [x] (must-fix) Wrong line citation corrected: `RowInserter`'s unconditional parent
  dereference is at `autonomousvehicleproject.cpp:1473-1480` (verified — the
  constructor body; `:1386-1393` is `dropMimeData`/`readChildren`) —
  `src/camp/mission_insertion.h:33`
- [x] (suggestion) `m_unavailableVectorLayerFiles.removeAll()` moved to AFTER the
  `if(!layers) return;` guard, so a path leaves the unavailable list only once this
  call is certain to track the file as a loaded layer; the comment states the
  ordering dependency. The de-dup early return above it is unaffected in practice: a
  tracked layer for that filename means the removal already happened when it was
  opened — `src/camp/autonomousvehicleproject.cpp:421`
- [x] (suggestion) `applyStyle()`'s no-data wiring now has a test —
  `VectorLayerTeardown.ApplyStyleFlagsFeaturesWithNoValue` loads the mixed fixture,
  sets the colour field and asserts the one feature without a value is flagged and
  the four with values are not, then clears the colour field and asserts the flag
  clears everywhere (the `color_field_.isEmpty()` branch the review named) —
  `test/test_vector_layer_teardown.cpp:400`
- [x] (suggestion) Per-attribute size bound — **deferred**: what to do with a
  multi-hundred-MB string property (truncate, skip the feature, accept it) is an
  ADR-0016 D11 decision, not a mechanical edit (deferred: needs a design decision;
  outside this round's cheap-fixes scope)
- [x] (suggestion) Self-enforcing KML `NetworkLink` test — **deferred**: needs a
  NetworkLink fixture and a decision on how the regression should fail (deferred:
  not cheap; outside this round's scope)
- [x] (suggestion) File the two drafted follow-up issues — **deferred**: the host
  files them, per the dispatch scope (deferred: host-owned action)
- [x] (suggestion) PR body should say the dispatch-order change covers all six add-*
  modes — **deferred**: the PR body is the host's to edit; the fact is now recorded
  in plan rev 7's Files to Change row and in ADR-0016 D5 (deferred: host-owned
  action)

### Next step

Lifecycle: **Implementation** → **review-code** (re-review the fixes).
Nothing was pushed — the host performs pushes.

## Integrated Review
**Status**: complete
**When**: 2026-09-15 09:31 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #226 at `14d61c6`
**Sources**: 2 (Copilot R2 @ `14d61c6` — 2 inline comments; CI rollup @ `14d61c6`)
**Cross-source confirmations**: 0 (no local review entry at this head; the round-1 Copilot/local findings at `507ee23` were all addressed and re-verified below)
**CI**: all-pass (`build-and-test` SUCCESS, `copilot-pull-request-reviewer` SUCCESS, both on `14d61c6`); no human review comments, no conversation comments

Round 2 of triage. Only comments created after the first `## Integrated Review`
(2026-09-14 14:58) are triaged here; the round-1 findings were confirmed closed
by reading the current code at each cited site (the `ParseBudget` recursion, the
unavailable-path `removeAll` ordering, `isPlaceable`, `noDataColor`).

### Findings
- [x] (should-fix, Copilot R2) `readRing()` consumes EVERY vertex of a ring before returning: `ParseBudget` is polled at geometry/part boundaries only, so one LineString or ring with millions of points is an unbounded stretch inside a worker whose join runs on the GUI thread (`VectorLayer::~VectorLayer()` → `waitForFinished()`). This is the same failure class round 1 fixed one level up — the per-feature poll was too coarse for a 200 000-part MultiPolygon; the per-part poll is too coarse for a 200 000-vertex ring. `budget` is already in scope at all three `readRing()` call sites (`vector_parse.cpp:167`, `:189`, `:192`), so the fix is to pass it in and poll `budget.aborted` every N vertices (N ~1024 — `isAborted()` takes a mutex, so per-vertex polling is not free) and break out. A truncated ring is safe: the next feature-boundary check sets `diagnostics.aborted` and `loadFinished()` discards the whole result on that flag. The existing abort test does not cover this path — its fixture is 200 000 SEPARATE points, so every poll site it exercises is a part boundary; add a single-huge-ring fixture — `src/camp_map/vector/vector_parse.cpp:75`, `test/test_vector_layer_teardown.cpp:304`
- [x] (should-fix, Copilot R2) `normalizedValue()` mis-maps interior values when the span overflows: with `range.min = -DBL_MAX` and `range.max = DBL_MAX`, `span` is `+inf` but `(0 - range.min) / span` is the FINITE value `0`, so the `!std::isfinite(t)` fallback never fires. Verified by direct execution of the function's arithmetic: `0 → 0.0`, `-1e100 → 0.0`, `1e100 → 0.0`, while `1e307 → 0.5` and `-1e307 → 0.0` — not merely compressed, but non-monotonic, so colours and marker radii are ordered wrongly, not just squeezed. Reachable from an operator-chosen file: `numericAttribute()` admits any finite double, and a GeoJSON property may legitimately hold `1e308`. Fix: when `span` is not finite, compute on halved operands — `(value/2 - range.min/2) / (range.max/2 - range.min/2)` is finite for every finite min/max — keeping the existing order-based fallback as the last resort — `src/camp_map/vector/vector_style.cpp:87`
- [x] (suggestion, Copilot R2) Second half of the `readRing` comment: bound VERTEX count, not just geometry count, so one ring cannot materialise an arbitrarily large coordinate vector (100M vertices ≈ 1.6 GB). Unlike the cancellation half this is a DECISION, not a mechanical edit — silently truncating a ring draws a wrong shape, which is worse than the documented geometry cap, so the honest options are drop-the-geometry-and-count-it in `ParseDiagnostics` or accept it and say so. Fold into the already-deferred ADR-0016 D11 item on per-attribute size bounds (`## Local Review (Pre-Push)` 2026-09-15 08:48) rather than opening a second decision thread — `src/camp_map/vector/vector_parse.cpp:75`, `docs/decisions/0016-read-only-vector-file-layer.md` (deferred: operator decision — folded into the already-deferred ADR-0016 D11 per-attribute-bound decision; a sentence recording that the vertex count belongs to the same deferred decision was added to D11)

### False positives
- (none this round — both new Copilot comments describe failure modes that the current code genuinely has)

## Implementation
**Status**: complete
**When**: 2026-09-15 09:47 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #226 at `9336b5f`
**Addressed**: `## Integrated Review` 2026-09-15 09:31 -04:00 (round 2, PR #226 at `14d61c6`)
**Commits**: `382c05a`, `4ed30ef`, `9336b5f`

Both should-fix findings fixed, each with the test the review asked for; the
suggestion deferred per the operator's decision at the host checkpoint. Full camp
suite rebuilt and run: **379 tests, 0 failures, 1 skipped** (the pre-existing
skip). The only compiler warning is the pre-existing `-Wdangling-else` in
`test_vector_layer_teardown.cpp:539`, untouched by this pass.

Verification worth recording: the old `normalizedValue()` arithmetic was executed
directly against the new fixture's values before the fix, confirming
`-1e307 → 0.0`, `0 → 0.0`, `1e100 → 0.0`, `1e307 → 0.5` — the bottom of the ramp
swallows everything up to ~1e308, which is why a test asserting only
finite/in-range/not-NaN (the existing wide-range test) passed over it. The new
test asserts ORDER and the midpoint, and `EXPECT_NEAR(normalizedValue(0.0), 0.5)`
is the assertion that fails against the old code.

### Actions
- [x] (should-fix, Copilot R2) `readRing()` polled the budget at geometry/part
  boundaries only, so one ring of millions of vertices was an unbounded stretch
  inside the worker the destructor joins on the GUI thread — `readRing()` now
  takes the `ParseBudget` and polls every 1024 vertices, plus once per ring so a
  polygon of very many SHORT interior rings is bounded too (that loop is not
  budget-checked, and breaking it on the geometry cap would drop the holes of a
  polygon drawn in full). New fixture `writeOneHugeLineString()` and test
  `VectorLayerTeardown.AbortCutsShortASingleHugeRing`: ONE feature of ONE part
  with 400 000 vertices, where the vertex loop is the only place an abort can be
  honoured — the existing 200 000-point fixture exercises part boundaries only —
  `src/camp_map/vector/vector_parse.cpp:112`,
  `test/test_vector_layer_teardown.cpp:435`
- [x] (should-fix, Copilot R2) `normalizedValue()` now computes on halved
  operands when the span is not finite, keeping the order-based fallback last;
  new test `VectorLayerStyling.OverflowingSpanStaysMonotonic` over
  min=-DBL_MAX/max=DBL_MAX asserts monotonic ordering across -DBL_MAX, -1e307,
  -1e100, 0, 1e100, 1e307, DBL_MAX and that the midpoint is not pinned to an end
  — `src/camp_map/vector/vector_style.cpp:87`,
  `test/test_vector_layer_styling.cpp:199`
- [x] (suggestion, Copilot R2) Bound the VERTEX count, not just the geometry
  count (deferred: operator decision at the 2026-09-15 host checkpoint — folded
  into the already-deferred ADR-0016 D11 per-attribute-bound decision rather than
  opened as a second thread. D11 now names the two together and says why each is
  a decision and not a mechanical edit: truncating a ring draws a wrong shape,
  truncating a property reports a wrong value) —
  `docs/decisions/0016-read-only-vector-file-layer.md:191`

Plan kept in sync: **rev 8** records the round-4 pass and corrects the two places
that said abort latency is bounded by one geometry — one geometry can be one ring
of millions of vertices (`.agent/work-plans/issue-22/plan.md`).

### Next step
`review-code` re-review of the branch diff (fresh context).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-15 09:58 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-22 at `754509b`
**Mode**: pre-push
**Depth**: Standard (reason: small targeted diff, but worker-thread lifecycle + numeric correctness on an already deeply-reviewed branch)
**Must-fix**: 1 | **Suggestions**: 2
**Round**: 4 | **Ship**: recommended — one mechanical must-fix (a two-line ordering change plus the test that would have caught it); must-fix count is not rising and nothing from rounds 1-3 regressed.

Scope: the 4 commits since `dd3353b` (`382c05a`, `4ed30ef`, `9336b5f`, `754509b`).
Build clean; full camp suite re-run: **379 tests, 0 failures, 1 skipped** (the
pre-existing `GggsRenderTest.RealStoreRendersWhenProvided`). Both new tests pass
(`OverflowingSpanStaysMonotonic` 0 ms, `AbortCutsShortASingleHugeRing` 1599 ms).
Pre-commit hooks clean over the diff range.

The `normalizedValue()` halved-operand fix was verified by hand at the boundary
cases (-DBL_MAX -> 0.0, 0 -> 0.5, DBL_MAX -> 1.0) and is exact and monotonic; the
`readRing()` poll site, iterator lifetime on the early break, counter reset, and
the `[this]`-capture lifetime against the destructor's set-flag-then-join are all
correct. The one must-fix is an interaction the new truncation path created with
the pre-existing cap/abort ordering, found independently by the lead reviewer and
by the Lens A adversarial pass.

### Findings
- [x] (must-fix) Cap check precedes the abort check, so a feature whose ring was truncated by the abort poll AND which crosses `max_geometries` returns with `geometry_cap_reached` set and `aborted` UNSET — the truncated ring is then outside the "discard on `diagnostics.aborted`" net and would be drawn as a complete shape (unreachable through `VectorLayer` today, whose only abort source is the destructor, but reachable through the public `parseVectorLayers()` API the tests call directly, and it makes the new comment's claim "Nothing partial is ever drawn" false); check abort before the cap, or have the cap branch set `diag.aborted` when the predicate fired — `src/camp_map/vector/vector_parse.cpp:408-423` (claim at `src/camp_map/vector/vector_parse.cpp:106-109`)
- [x] (suggestion) The interior-ring loop does not break on abort the way the sibling `wkbGeometryCollection` part loop does (`vector_parse.cpp:254`), so a polygon with very many short interior rings still spends one mutex-guarded predicate call per ring after the abort; breaking specifically on `budget.aborted && budget.aborted()` (not on `exhausted()`, which would drop holes at the cap) closes the asymmetry — `src/camp_map/vector/vector_parse.cpp:225`
- [x] (suggestion) The deferred vertex-count / attribute-size bound is tracked only in ADR prose and the plan, unlike the sibling camp#225 deferral from the same round; file a tracking issue or say in the ADR that it is deliberately tracked there only — `docs/decisions/0016-read-only-vector-file-layer.md:197-203` (deferred: host-owned — the operator files the tracking issue at the publish step, as with the sibling camp#225 deferral; ADR-0016 is deliberately left untouched)

## Implementation
**Status**: complete
**When**: 2026-09-15 10:04 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #226 at `17afcbc`
**Addressed**: `## Local Review (Pre-Push)` 2026-09-15 09:58 -04:00 (round 4, branch `feature/issue-22` at `644e58c`)
**Commits**: `6874e2e`, `a6fbf69`, `17afcbc`

The round-4 must-fix and its sibling suggestion are fixed; the third finding is
checked as host-owned (the tracking issue is the operator's to file at publish,
matching the camp#225 deferral from round 2 — ADR-0016 deliberately untouched).

Full camp suite rebuilt and re-run on the committed tree: **380 tests, 0
failures, 1 skipped** (the pre-existing `GggsRenderTest.RealStoreRendersWhenProvided`),
up one test from 379. Pre-commit hooks clean on all three commits. The only new
compiler warnings are the `-Wunused-result` on `OGRLayer::CreateFeature` in the
new fixture, the same warning every existing fixture helper in that file emits.

Negative check worth recording: the new test was run against the PREVIOUS
ordering (cap branch first) before the fix was restored, and failed exactly as
predicted — `diag.aborted` false where true was expected. The fix is therefore
demonstrably the thing the test pins, not an assertion written to the code.

### Actions
- [x] (must-fix) The per-feature cap check ran before the abort check and
  returned from its own branch, so a feature that both crossed `max_geometries`
  and carried a ring truncated by the vertex poll returned `geometry_cap_reached`
  with `aborted` UNSET — outside `VectorLayer::loadFinished()`'s
  discard-on-aborted net. The abort is now checked FIRST; a parse that hits both
  reports `aborted` and is not trimmed to exactly `max_geometries` (it is partial
  by construction, and its caller discards it). The now-false `readRing()`
  comment is corrected to name the ordering its safety claim depends on, and
  `ParseDiagnostics::geometry_cap_reached` documents the precedence. New test
  `VectorParseAttributes.AbortMidRingAtTheCapIsStillReportedAsAborted`: a
  one-feature / one-LineString / 4000-vertex fixture parsed with
  `max_geometries = 1` and a predicate that fires on the in-loop vertex poll,
  asserting the ring came back truncated (neither empty nor whole — which is what
  proves the abort fired mid-ring rather than at a coarser poll site) and that
  `diagnostics.aborted` is set — `src/camp_map/vector/vector_parse.cpp:415`,
  `src/camp_map/vector/vector_parse.h:108`,
  `test/test_vector_parse_attributes.cpp:672`
- [x] (suggestion) The interior-ring loop now breaks on `budget.aborted &&
  budget.aborted()`, like the sibling geometry-collection part loop, so a polygon
  of very many short interior rings stops spending one mutex-guarded predicate
  call per ring once the abort is raised. Deliberately NOT `exhausted()`, which
  also fires at the geometry cap and would drop the holes of a polygon whose
  exterior is drawn in full; the `readRing()` comment that said the ring loop is
  unchecked is updated to state the new rule —
  `src/camp_map/vector/vector_parse.cpp:231`
- [x] (suggestion) File a tracking issue for the deferred vertex-count /
  attribute-size bound (deferred: host-owned — the operator files it at the
  publish step, as with the sibling camp#225 deferral from round 2; ADR-0016 is
  deliberately left untouched by this pass) —
  `docs/decisions/0016-read-only-vector-file-layer.md:197-203`

Plan kept in sync: **rev 9** records the ordering change and what it does and
does not alter about the rev-8 abort-latency claim
(`.agent/work-plans/issue-22/plan.md`).

### Next step
`review-code` re-review of the branch diff (fresh context). Nothing was pushed —
the host performs pushes.

## Implementation
**Status**: complete
**When**: 2026-09-15 10:53 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #226 at `85da773`
**Addressed**: operator GUI test of 2026-09-15 (Roland, CAMP built from this
branch, `~/data/logs/analysis/2026-09-14_massabesic_mag/massabesic_joint_candidates.geojson`
— 7 Point features, free-text `assessment`, numeric `tfa_nT`/`rank`)
**Commits**: `7f0cb66`, `9ef6953`, `ec78f6a`, `85da773`

The first time an operator drove this branch, two of the three "must have"
behaviours did not work in his hands, and neither was visible to any test
written so far. Both reports were reproduced and fixed.

**Finding 1 — "changing the colour-by field to `assessment` makes the features
disappear."** Two defects compounded. `numericAttribute()` returns nullopt for
every string value, so the field range was invalid, `isNoData()` was true for
every feature, and the layer went entirely no-data; the hollow no-data point
marker was then stroked with the same width-0 hairline the FILLED marker gets
for contrast — but the filled marker has a disc of colour behind that hairline
and the hollow one has nothing, so one dashed device pixel of mid grey over a
chart is not visible. Fixed at all three levels: the marker is stroked at
cosmetic width 2 (hollow and dashed unchanged — ADR-0016 D6's second channel,
only its weight changes); the styling menus offer `VectorLayer::numericFields()`
rather than `fields()`, so a field no ramp can read is not selectable; and
`applyStyle()` treats an invalid colour range as NO colour field for that pass
(default colour, nothing flagged) rather than marking the whole layer no-data,
which is the fallback a PERSISTED `color_field_` can still reach since the style
group is keyed on the file path and the file may have changed.

**Finding 2 — "I never see a tooltip for the targets... not sure where the
hotspot is on the hand icon."** The point hit `shape()` was exactly the drawn
5 px marker, aimed at under an open-hand pan cursor whose hotspot is not
visible. `shape()` now carries `kPointClickSlackPixels` (4 px) of slack, with
`boundingRect()` grown to match (a shape outside the bounding rect is undefined
in Qt) — the same trade a line already makes with `kClickWidth`. Nothing drawn
grows.

Why no test caught either: every click test in the repo sends a
`QGraphicsSceneMouseEvent` straight to the item, skipping the viewport widget,
the view's (1, -1) y-flip, ScrollHandDrag and the scene's hit test against
`shape()`; and nothing looked at what a marker actually paints. The new
`VectorLayerInteraction` test puts a real `QGraphicsView` over a real loaded
layer and sends synthesized `QMouseEvent`s to its viewport.

Full camp suite rebuilt and re-run on the committed tree: **385 tests, 0
failures, 1 skipped** (the pre-existing `GggsRenderTest.RealStoreRendersWhenProvided`),
up five from 380. Pre-commit hooks clean on all four commits; no new compiler
warnings.

Negative check recorded: all three new tests were run against the pre-fix
behaviour (slack set to 0, pen back to width 0) and fail exactly as predicted —
the no-data marker covers 42 px against a hairline's 42 (pixel-identical, so the
test really is pinning the pen weight), the 7 px off-centre click misses, and
the centre and 3 px clicks still pass, which is the boundary the slack moves.

### Actions
- [x] (finding 1) The hollow no-data point marker is stroked at cosmetic width 2
  instead of a width-0 hairline — `src/camp_map/vector/vector_feature_item.cpp`
  `paint()`. New test
  `VectorFeatureItem.NoDataPointMarkerIsDrawnHeavierThanAHairline`: paints the
  item into a QImage and compares coverage against a hairline reference the test
  draws itself (no magic pixel count), and separately asserts the interior is
  still untouched — a pixel budget cannot say "hollow", since a dashed width-2
  ring with antialiasing covers about as many pixels as a solid disc.
- [x] (finding 1) `VectorLayer::numericFields()` added and used by the Color by /
  Size by / Colormap menus; `fields()` unchanged (the attribute popup and a future
  label-by-field want every field) — `src/camp_map/vector/vector_layer.{h,cpp}`.
  New test `VectorLayerStyleFields.NumericFieldsExcludeAStringOnlyField`
  (string-only excluded, mixed numeric/text included, `fields()` still complete).
- [x] (finding 1) `applyStyle()` falls back to UNSTYLED on an invalid colour
  range rather than marking every feature no-data — new test
  `VectorLayerStyleFields.AColorFieldWithNoNumbersFallsBackToUnstyled`, which
  also checks the colour is the default and not the no-data grey.
- [x] (finding 2) Point `shape()` and `boundingRect()` carry 4 px of click slack
  — new tests `VectorFeatureItem.PointClickTargetIsWiderThanTheDrawnMarker` and
  `VectorLayerInteraction.ClickThroughARealViewShowsTheAttributes` (through a
  y-flipped ScrollHandDrag `QGraphicsView`, at centre / 3 px / 7 px off, plus a
  click clear of any feature that must not answer).
- [x] ADR-0016: D6 gains the marker-weight reason; new D14 (numeric-only ramps,
  categorical styling a follow-on, the invalid-range fallback) and D15 (the
  click slack), plus a consequence for what D14 costs. Plan rev 10 records the
  GUI test as the source.
- [ ] (operator's call) **Categorical styling is now an explicit follow-on** —
  a distinct colour per class with a legend, which is what `assessment` actually
  wants. ADR-0016 D14 names it; no issue filed, matching the camp#225 deferral
  convention on this branch.
- [ ] (operator's call) **Label-by-field** remains the other named follow-on, and
  is the reason `fields()` was kept whole rather than narrowed.

## Implementation
**Status**: complete
**When**: 2026-09-15 11:25 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #226 at `dd0c6ba`
**Addressed**: operator decision at a run-issue checkpoint, 2026-09-15, taken
after the GUI test of the rev-10 fixes succeeded ("It works!")
**Commits**: `dde15dd`, `78c75b4`, `dd0c6ba`

**The decision, and it is not a defect report.** With the layer working in the
operator's hands, the remaining oddity was that it was the only thing in CAMP
that inspects on a CLICK. The house convention is hover: `Platform` and
`AISContact` show their label with `setShowLabelFlag(true)` in
`hoverEnterEvent` (verified in `platform.cpp:170`, `ais_contact.cpp:272`),
`GeoGraphicsMissionItem` brightens on hover (`geographicsmissionitem.cpp:32`),
and nothing inspects on click. The operator decided to switch the attribute
popup to hover in this PR and to delete the click machinery that existed only
to tell a click from a pan.

**Hover is the item's ordinary Qt tooltip.** `setToolTip(attributeText())` in
the constructor; `QGraphicsScene::helpEvent()` finds the top item under the
cursor with a non-empty `toolTip()` and shows it after the usual delay, hiding
it on move-away. No event code of ours is in the path, and the fallback the
brief allowed (`hoverEnterEvent` + `QToolTip::showText`) was not needed: the
`ItemIgnoresTransformations` concern does not bite, because `helpEvent()`
hit-tests through `QGraphicsScenePrivate::itemsAtPosition()`, which passes the
view's own `viewportTransform()` as the device transform. Verified by the new
through-a-real-view test rather than by reading, which is why it was written
first. `setAcceptHoverEvents()` is deliberately NOT set — hover events are not
what drives a tooltip, and per-item hover tracking across up to
`kMaxFeatureItems` (50 000) items would cost something for nothing.

**What was deleted.** `mousePressEvent`, `mouseReleaseEvent`,
`viewInPanMode()` and `kClickSlopPixels` are gone, and the item now sets
`setAcceptedMouseButtons(Qt::NoButton)` (`QGraphicsItem` accepts the left
button by default, so it has to be said). **camp#225 — a pan gesture that starts
on a feature does not pan — is therefore fixed by construction**, not worked
around: the press always reaches `QGraphicsView`'s ScrollHandDrag. The host can
close camp#225 at merge.

**ProjectView is reverted.** `e2a56cc` ("dispatch the press under the mode it
was made in") existed only so the pan-mode gate would read the mode the operator
clicked in. With no gate it has no purpose, and it rewired the press path of
every add-\* mode that returns to pan, so `src/camp/projectview.cpp` is restored
to its `jazzy` content — **byte-identical**, checked with
`git diff jazzy -- src/camp/projectview.cpp src/camp/projectview.h` (empty).
This branch now changes no ProjectView behaviour at all.
`src/camp/mission_insertion.h` was checked and references nothing about the
deferral.

**What was kept.** The hit slack from rev 10 (ADR-0016 D15): `shape()` is what
`helpEvent()` hit-tests too, so the 4 px around a point marker and the stroked
ribbon around a line serve hover for exactly the reason they served the click.
The constants are renamed `kPointHoverSlackPixels` / `kHoverWidth`.

**Tests.** Full camp suite rebuilt and re-run on the committed tree: **387
tests, 0 failures, 1 skipped** (the pre-existing
`GggsRenderTest.RealStoreRendersWhenProvided`), up two from 385 — one click
test replaced by two hover tests at the item level, one at the view level
replaced by two. Pre-commit hooks clean on all three commits; no new compiler
warnings.

Negative check recorded: with `setToolTip()` commented out and
`setAcceptedMouseButtons(Qt::LeftButton)` restored, three of the four new tests
fail — `HoverPopupIsTheItemsTooltip`, `AcceptsNoMouseButtonSoThePressReachesTheView`
and `HoverThroughARealViewShowsTheAttributes`. The fourth,
`APressOverAFeatureFallsThroughToTheView`, still passes in that state, and this
is worth saying plainly: accepting the left button is not by itself enough to
grab the press, because `QGraphicsItem::mousePressEvent()`'s default ignores it.
It discriminates against what was actually there before — an override that
accepted — not against the flag alone, which is what the item-level test pins.

### Actions
- [x] Hover-to-inspect via `setToolTip(attributeText())`; the mouse handlers,
  `viewInPanMode()` and `kClickSlopPixels` deleted;
  `setAcceptedMouseButtons(Qt::NoButton)` — `src/camp_map/vector/vector_feature_item.{h,cpp}`.
- [x] `src/camp/projectview.cpp` reverted to `jazzy`; verified byte-identical.
- [x] Tests: `VectorFeatureItem.HoverPopupIsTheItemsTooltip`,
  `VectorFeatureItem.AcceptsNoMouseButtonSoThePressReachesTheView`,
  `VectorLayerInteraction.HoverThroughARealViewShowsTheAttributes` (synthesized
  `QHelpEvent` through a y-flipped view at centre / 3 px / 7 px off / clear),
  `VectorLayerInteraction.APressOverAFeatureFallsThroughToTheView` (no scene
  mouse grabber). `PointClickTargetIsWiderThanTheDrawnMarker` renamed
  `PointHoverTargetIsWiderThanTheDrawnMarker` and kept — it is about the shape.
- [x] ADR-0016 D5 rewritten (hover, the convention, and the three things the
  click version needed that are now gone); D15 kept as hover tolerance; the
  camp#225 consequence is now "fixed by construction". `.agents/README.md` and
  `vector_layer.h` updated; plan rev 11.
- [ ] (host) **Close camp#225 at merge** — fixed by construction, pinned by two
  tests.
- [ ] (host, follow-ons recorded in ADR-0016, no issues filed) **Click to pin the
  popup open** — hover answers "what is this"; comparing two features or copying
  a value wants the popup to stay, and it must be built without taking the press
  back from the view's pan gesture. **The pan cursor's invisible hotspot** is a
  CAMP-wide choice (`ScrollHandDrag`'s open hand, shared by every layer and
  mission item), not this layer's; D15's hit slack is this layer working around
  it.

## Implementation
**Status**: complete
**When**: 2026-09-15 11:47 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #226 at `2857e45`
**Addressed**: two operator decisions taken at a run-issue checkpoint,
2026-09-15, after a GUI test of the rev-11 build
**Commits**: `5811cfd`, `670f7ef`, `2857e45`

**Neither of these is a defect report.** The rev-11 build works; the operator
tested it and decided two things about how it should feel.

**1. The hover popup is now an IN-SCENE LABEL, and it is instant.** Rev 11 made
the popup the item's ordinary Qt tooltip. Testing that, the operator's verdict
was "similar to what was existing, but not the same": CAMP's own items show
their info INSTANTLY. They do it with a label, not a tooltip —
`GeoGraphicsItem` (`src/camp/geographicsitem.cpp:14-26`) owns a child
`QGraphicsSimpleTextItem` with `ItemIgnoresTransformations`, a 20 pt bold font,
a black brush and a width-0 white pen, and `Platform::hoverEnterEvent`
(`platform.cpp:170`) / `AISContact::hoverEnterEvent` (`ais_contact.cpp:272`)
fill it via `setShowLabelFlag(true)`, clearing it on leave. No delay, no tooltip
window.

`VectorFeatureItem` REPLICATES that rather than inheriting it: it lives in
`camp_map` and cannot depend on `GeoGraphicsItem` in the `camp` executable (the
same rule that put `parseVectorLayers()` in `camp_map`). So:
`setAcceptHoverEvents(true)`; a child `QGraphicsSimpleTextItem` carrying the
settings copied from `geographicsitem.cpp:16-25` with that source named in the
comment; `hoverEnterEvent()` sets its text to `attributeText()` and positions it
— for a point just beside the marker in device pixels, clear of the hit slack;
for a line or polygon at `event->pos()`, since those have no single anchor worth
labelling — and `hoverLeaveEvent()` empties it. `setToolTip()` is REMOVED, so
there is no second, delayed popup. Only one label is ever on screen, because
every item clears its own on leave.

Two things were weighed rather than assumed. The label item is created on the
**first hover**, not in the constructor: a layer may hold `kMaxFeatureItems`
(50 000) features, and a text child per feature would be 50 000 scene items,
index entries and font metrics created at load for the handful the operator ever
hovers. And `setAcceptHoverEvents(true)` is a real cost the tooltip did not have
— per-item hover tracking across the layer — which rev 11's comment had cited as
a reason NOT to accept hover events. It is paid deliberately now: the reason for
choosing hover at all was consistency with the application, and a popup that
behaves differently from every other popup does not deliver it. The hit slack
(ADR-0016 D15) is untouched; `shape()` is what the scene hit-tests to dispatch
hover events too.

**2. Pan mode shows an ARROW cursor, CAMP-wide.** Recorded last round as a
follow-on ("the pan cursor is a CAMP-wide choice, not this layer's"); the
operator decided it instead, in this PR, because it is four lines. `Qt::ArrowCursor`
goes on the **viewport** at the two points Qt installs its own open hand: after
`setDragMode(ScrollHandDrag)` in `setPanMode()`, and after
`QGraphicsView::mouseReleaseEvent()` returns, which restores the open hand at the
end of every drag. The CLOSED hand during an actual drag stays — there it is
feedback, not something being aimed. The add-\* modes keep `Qt::CrossCursor`.

The view-vs-viewport split is load-bearing and was checked rather than assumed:
the add-\* modes call `setCursor()` on the VIEW, which a viewport with no cursor
of its own inherits, and leaving pan mode calls `setDragMode(NoDrag)`, where Qt
unsets the viewport's own cursor so that inheritance resumes. Setting the arrow
on the view would simply be overridden by Qt's viewport cursor. `projectview.h`
is unchanged. This is now the branch's only `ProjectView` change — rev 11's
revert to `jazzy` otherwise stands.

**Tests.** Full camp suite rebuilt and re-run on the committed tree: **387
tests, 0 failures, 1 skipped** (the pre-existing
`GggsRenderTest.RealStoreRendersWhenProvided`) — unchanged in count, because two
tooltip-based tests were replaced one-for-one:

- `VectorFeatureItem.HoverPopupIsTheItemsTooltip` →
  `VectorFeatureItem.HoverShowsAnInSceneLabelWithTheAttributes`: no label child
  exists before the first hover; a `QGraphicsSceneHoverEvent(GraphicsSceneHoverEnter)`
  creates one carrying `attributeText()` and the vessel/AIS flag, brush, pen and
  bold font; `GraphicsSceneHoverLeave` clears it; and `toolTip()` is empty. The
  handlers are protected and a `QGraphicsItem` is not a `QObject`, so there is no
  `QApplication::sendEvent()` path to them — a four-line `HoverProbe` subclass
  re-exposes them, which is stated in the test.
- `VectorLayerInteraction.HoverThroughARealViewShowsTheAttributes` rewritten: the
  synthesized `QHelpEvent` is replaced by a buttonless
  `QMouseEvent(QEvent::MouseMove, ...)` sent to the viewport of a real y-flipped
  ScrollHandDrag `QGraphicsView` under a `camp::map::Map` — what the operator's
  hand actually produces, and what the scene turns into hover enter/leave. At
  centre and 3 px off the label carries the attributes; moved clear of any
  feature it is empty, which is what keeps exactly one label on screen.
- `VectorFeatureItem.AcceptsNoMouseButtonSoThePressReachesTheView` and
  `VectorLayerInteraction.APressOverAFeatureFallsThroughToTheView` are unchanged.

Negative check recorded: with `setAcceptHoverEvents(true)` commented out, both
new hover tests fail; restored and re-verified green.

**The cursor change has NO test.** `ProjectView` is not constructible in a test
harness — it needs the application's status bar and project, which is the same
reason `test_mission_insertion` exercises the model rather than the view. It is
verified in the GUI, and D16 says so.

### Actions
- [x] Hover label replacing the tooltip: `setAcceptHoverEvents(true)`, a lazily
  created child `QGraphicsSimpleTextItem` with `GeoGraphicsItem`'s settings,
  `hoverEnterEvent()`/`hoverLeaveEvent()`, `setToolTip()` removed —
  `src/camp_map/vector/vector_feature_item.{h,cpp}`.
- [x] `Qt::ArrowCursor` on the viewport in pan mode and after the release —
  `src/camp/projectview.cpp` only; `projectview.h` untouched.
- [x] Tests: `VectorFeatureItem.HoverShowsAnInSceneLabelWithTheAttributes` and a
  rewritten `VectorLayerInteraction.HoverThroughARealViewShowsTheAttributes`;
  the two press/no-mouse-button tests kept.
- [x] ADR-0016 D5 rewritten (the label, why instant, and the rejected tooltip);
  new **D16** for the arrow cursor; D15 points at it; two new consequences (the
  CAMP-wide cursor, the hover-tracking cost); the "pan cursor is a CAMP-wide
  choice" follow-on REMOVED because it was decided. `.agents/README.md` gains a
  ProjectView cursor convention; `vector_layer.h` and plan rev 12 follow.
- [ ] (operator) **Confirm in the GUI**: the label appears the instant the cursor
  reaches a feature and reads like the vessel/AIS labels, and the arrow cursor in
  pan mode is what you wanted — including that the add-\* modes still show the
  cross and a real drag still shows the closed hand. Neither the label's
  placement nor the cursor can be judged from a test.
- [ ] (host) **Close camp#225 at merge** — still fixed by construction, unchanged
  by this round.
- [ ] (host, follow-ons recorded in ADR-0016, no issues filed) Categorical
  styling, label-by-field, and click-to-pin-the-popup-open.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-15 12:07 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-22 at `ce3f65a`
**Mode**: pre-push
**Depth**: Deep (reason: 13 commits / ~1430 lines across camp and camp_map, a CAMP-wide cursor change, and Qt event-dispatch/lifecycle surface)
**Must-fix**: 8 | **Suggestions**: 7
**Round**: 5 | **Ship**: continue — no design question is open and every fix is precise, but two are code findings on the newly hover-driven hit path (an uncached stroker called per mouse-move; a persisted style the menu can neither show nor clear) and six are doc/comment corrections; must-fix rose 1 -> 8 because 13 commits of NEW implementation landed after round 4's fix pass, so this is effectively a first read of that work, not a failure to converge.

Build green (`ui_ws/build.sh camp`, exit 0, no new warnings). Full suite green: **387 tests, 0 errors, 0 failures, 1 skipped** — the skip is the pre-existing environment-gated `GggsRenderTest.RealStoreRendersWhenProvided`. Specialists: Static Analysis (cppcheck; cpplint unavailable on this host — a named gap, not a clean bill), Governance, Plan Drift, Claude Adversarial Lens A + Lens B. Copilot and local-model reviews off.

Not re-litigated, per the dispatch: hover-to-inspect, the instant in-scene label, the pan-mode arrow cursor, and numeric-only ramp fields are settled operator decisions from the 2026-09-15 GUI test.

### Findings
- [x] (must-fix) `shape()` rebuilds a `QPainterPathStroker` and re-strokes the whole path on EVERY call; with inspection now on hover the scene calls it per mouse-move for every candidate under the cursor (a long polyline's bounding rect makes it a candidate across most of the map), where the click version paid it once per click — cache the stroke, invalidated when `path_` changes — `src/camp_map/vector/vector_feature_item.cpp:210-219`
- [x] (must-fix, cross-confirmed Lens A + Lens B) A persisted `color_field_`/`size_field_` that is not in `numericFields()` (the file changed since, or a column was re-typed) leaves the context menu with no Color by / Size by submenu at all, so neither "(none)" nor the stale field is shown or clearable; `applyStyle()` paints it correctly as unstyled, but the setting is stuck and silently re-persisted — build the menus (or at least "(none)") whenever a field is set — `src/camp_map/vector/vector_layer.cpp:438-440`
- [x] (must-fix) `fields()` doc still says "attribute inspection (the click-to-inspect popup)"; click-to-inspect was deleted this round — `src/camp_map/vector/vector_layer.h:110-111`
- [x] (must-fix) Test-target comment still says "the attribute tooltip"; the tooltip was tried and rejected in favour of the in-scene label (D5) — `CMakeLists.txt:782`
- [x] (must-fix) ADR-0016 D5 claims "`ProjectView` is byte-identical to `jazzy` again", which D16 in the same document contradicts — the file carries 18 added lines vs `jazzy` — `docs/decisions/0016-read-only-vector-file-layer.md:155`
- [x] (must-fix) Files-to-Change row is self-contradictory: it names the rev-12 cursor change as "the only ProjectView change this branch now carries" and then says the file is "byte-identical to `jazzy`, and this branch now touches no ProjectView behaviour" — `.agent/work-plans/issue-22/plan.md:765`
- [x] (must-fix) Approach step 3 still specifies click-to-inspect in the present tense — `setAcceptedMouseButtons(Qt::LeftButton)`, `QToolTip::showText()` only in pan mode, `event->ignore()` in the add-* modes, and a possible `ProjectView` `mouseMode` accessor — all of which rev 11/12 deleted — `.agent/work-plans/issue-22/plan.md:580-599`
- [x] (must-fix) The new "`ProjectView` cursors" paragraph was inserted INSIDE the vector-layer paragraph, so its closing sentence "The layer persists as **app state** under `QSettings vectorLayers/files`..." now reads as a statement about the cursor convention — move the cursor paragraph after the vector-layer one — `.agents/README.md:124-135`
- [x] (suggestion) No test hovers a LINE or POLYGON feature: the label branch that positions at `event->pos()` (rather than beside the marker) is entirely uncovered, and `HoverShowsAnInSceneLabelWithTheAttributes` never asserts the label's position at all — `test/test_vector_feature_item.cpp:275`, `src/camp_map/vector/vector_feature_item.cpp:312-326`
- [x] (suggestion) The point label's offset is computed from `radius_` at hover-enter and never recomputed, so a `setRadius()` from `applyStyle()` while the cursor is parked on a feature leaves the label at a stale gap until the next hover-enter — `src/camp_map/vector/vector_feature_item.cpp:319`
- [x] (suggestion) The hover label is a child of the hovered feature, and feature items are unordered siblings with no `zValue()`, so a label can paint underneath a feature stacked above its parent (a polygon loaded after a point it covers) — `src/camp_map/vector/vector_feature_item.cpp:350`, `src/camp_map/vector/vector_layer.cpp:229`
- [x] (suggestion) `numericFields()` is O(features x attributes) with a `QVariant` conversion each, recomputed synchronously on every right-click; at the 50 000-feature cap that is a real per-menu-open cost in a file whose own comments guard the GUI thread at that same count — cache it beside `features_`, invalidated at load — `src/camp_map/vector/vector_layer.cpp:309-325` (deferred: the menu opens on a right-click, not per frame, and the cost is bounded by a cap the same file already treats as the GUI-thread budget; a cache adds an invalidation obligation to every load path for a cost no operator has reported — kept as a measurement-first item)
- [x] (suggestion) The arrow-cursor reset fires on ANY release while `dragMode() == ScrollHandDrag`, including the middle-button measuring-tool release — wider than the comment's "at the end of every drag", and it would silently clobber a future gesture that wants its own idle cursor — `src/camp/projectview.cpp:270-274`
- [x] (suggestion) No test covers a layer removed while the cursor is over one of its features (a hover-leave that never arrives because the item is gone); Lens B traced the teardown and found no crash path, but the case is unpinned — `test/test_vector_layer_teardown.cpp` (deferred: pinning it needs a hover-enter with no matching leave delivered through a real view mid-destruction, which the existing harness cannot stage without a timing hook in the destructor; Lens B's trace found no crash path, so this is coverage of a case believed safe rather than a fix)
- [x] (suggestion) cppcheck `useStlAlgorithm` on the two new label-finding helper loops; style only — `test/test_vector_feature_item.cpp:115`, `test/test_vector_layer_teardown.cpp:740` (deferred: style-only, in test helpers whose explicit loops read more plainly than std::find_if with a dynamic_cast predicate)

### Dismissed (checked, not findings)
- Lens A's claim that an `ItemIgnoresTransformations` label under a non-ignoring line/polygon parent is an unsupported Qt nesting: it is precisely what `GeoGraphicsItem` does for every vessel and AIS label (`src/camp/geographicsitem.cpp:16-17` on a scene-space parent) and what `camp_map/ros/markers/marker.cpp:93-97` documents. Repo precedent, not a defect. The associated TEST gap is kept above as a suggestion.
- Concurrency on `features_`: populated only in `loadFinished()` (a `QFutureWatcher::finished` slot on the GUI thread); the worker touches only `abort_flag_`, consistently under `abort_flag_mutex_`. No race.
- `setAcceptedMouseButtons(Qt::NoButton)` side effects: context menus arrive via `ProjectView::contextMenuEvent`, and the items are not `ItemIsSelectable`. None found.
- camp#225 "fixed by construction" wording: identical and accurate in ADR-0016 consequences, `.agents/README.md:122`, both `vector_feature_item` doc comments and the two pinning tests. camp#225 is still OPEN — closing it is already recorded as a host action at merge.

## Implementation
**Status**: complete
**When**: 2026-09-15 12:24 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-22 at `3c42f69`
**PR**: #226
**Addressed**: Local Review (Pre-Push), 2026-09-15 12:07 -04:00, round 5 at `ce3f65a`
**Commits**: 86914fe, 5d36d60, e7f69e4, 852e720, 32c8906, df7039d, 3c42f69

Rebuilt (`ui_ws/build.sh camp`, exit 0, no new warnings) and the full camp suite
is green: **391 tests, 0 errors, 0 failures, 1 skipped** — 387 before, +4 new
(the cached hit shape, the line/polygon label branch, the restyle-under-a-parked-
cursor label, and the stale-style-field menu). The skip is the same pre-existing
environment-gated `GggsRenderTest.RealStoreRendersWhenProvided`.

On must-fix 2 the choice was **show the stale field, do not clear it on load**:
it drops straight into the existing menu builder (a "(none)" entry plus a
checkable action per field, now built by one lambda for both submenus), needs no
status-message plumbing, and keeps a setting the operator chose — the same path
reopened over a corrected file makes it valid again, so clearing it silently
would discard that choice where nobody sees it happen. Pinned by
`VectorLayerStyleFields.AStaleStyleFieldIsShownInTheMenuAndCanBeCleared`.

### Actions
- [x] (must-fix) `shape()` re-stroked the path on every call — now a cached
  `shape_` rebuilt in the constructor and at each `prepareGeometryChange()` site
  — `src/camp_map/vector/vector_feature_item.cpp:205-241`, `.h:163-171` (86914fe)
- [x] (must-fix) A persisted `color_field_`/`size_field_` outside `numericFields()`
  now appears as "field (no numbers)" in a submenu that also carries "(none)" to
  clear it; the Colormap menu stays gated on a non-empty numeric list —
  `src/camp_map/vector/vector_layer.cpp:438-508` (852e720)
- [x] (must-fix) `fields()` doc no longer names a click-to-inspect popup —
  `src/camp_map/vector/vector_layer.h:109-112` (df7039d)
- [x] (must-fix) Test-target comment now says in-scene label, and records that the
  tooltip was tried and rejected — `CMakeLists.txt:780-786` (df7039d)
- [x] (must-fix) ADR-0016 D5 no longer claims byte-identity with `jazzy`; it names
  D16's cursor change as the one ProjectView change and scopes the revert to the
  press path — `docs/decisions/0016-read-only-vector-file-layer.md:155` (df7039d)
- [x] (must-fix) Files-to-Change ProjectView row rewritten to the same, consistent
  statement — `.agent/work-plans/issue-22/plan.md:765` (3c42f69)
- [x] (must-fix) Approach step 3 rewritten as hover-to-inspect: no accepted mouse
  buttons, no pan-mode gate, no `mouseMode` accessor, no `QToolTip` —
  `.agent/work-plans/issue-22/plan.md:603-618` (3c42f69)
- [x] (must-fix) The cursor paragraph moved out of the middle of the vector-layer
  paragraph — `.agents/README.md:128-143` (df7039d)
- [x] (suggestion) New `HoverOnALineOrPolygonLabelsAtTheCursor` covers the
  `event->pos()` branch for a line and a polygon and asserts the label's position;
  the point test now asserts its position too — `test/test_vector_feature_item.cpp`
  (5d36d60)
- [x] (suggestion) The point label's offset is recomputed in `updateLabelPosition()`,
  called from `setRadius()` as well as hover-enter —
  `src/camp_map/vector/vector_feature_item.cpp:320-330,372-386` (5d36d60)
- [x] (suggestion) The hovered ITEM is raised to `kHoveredZValue` (and its label
  with it) and dropped back on leave — a child's own zValue cannot clear a sibling
  of its parent, since Qt stacks the subtree —
  `src/camp_map/vector/vector_feature_item.cpp:350-360` (e7f69e4)
- [x] (suggestion) The arrow-cursor reset is scoped to `Qt::LeftButton` —
  `src/camp/projectview.cpp:270-280` (32c8906)
- [x] (suggestion) Cache `numericFields()` — `src/camp_map/vector/vector_layer.cpp:309-325`
  (deferred: per right-click, not per frame, and bounded by the same feature cap
  the file already treats as the GUI-thread budget; a cache buys an invalidation
  obligation on every load path for a cost nobody has measured)
- [x] (suggestion) Test a layer removed while the cursor is on one of its features
  — `test/test_vector_layer_teardown.cpp` (deferred: staging a hover-enter with no
  matching leave mid-destruction needs a timing hook in the destructor that does
  not exist; Lens B's trace found no crash path, so this is coverage of a case
  believed safe, not a fix)
- [x] (suggestion) cppcheck `useStlAlgorithm` on the two label-finding helpers
  (deferred: style-only, in test helpers whose explicit loops read more plainly
  than a `std::find_if` over a `dynamic_cast` predicate)

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-15 12:30 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-22 at `22ff76d`
**Mode**: pre-push
**Depth**: Deep (branch-level risk unchanged), scoped to the 8-commit delta since `5471f06` (+546 -118) — a targeted re-read on an already deeply reviewed branch
**Must-fix**: 1 | **Suggestions**: 5
**Round**: 6 | **Ship**: recommended — the one must-fix is a single stale sentence in ADR-0016 D14 with an obvious correction; must-fix fell 8 -> 1, no code finding survived three independent reads, and no design question is open.

Build green (`ui_ws/build.sh camp`, exit 0, no new warnings). Full suite green:
**391 tests, 0 errors, 0 failures, 1 skipped** — the skip is the pre-existing
environment-gated `GggsRenderTest.RealStoreRendersWhenProvided`; 387 -> 391 matches
the four tests this delta added. Specialists: Static Analysis (cppcheck; cpplint
still unavailable on this host — a named gap, not a clean bill), Governance +
Plan Drift, Claude Adversarial Lens A + Lens B. Copilot and local-model reviews off.

All eight round-5 must-fixes verified closed by reading the current file content,
not the diff: the cached hit shape, the stale-style-field menu, and the six
doc/plan/comment corrections. A repo-wide grep for present-tense "click-to-inspect",
"tooltip" and "byte-identical" claims found only historical, past-tense mentions.
Lens A and Lens B independently confirmed the two code fixes: `path_` and `radius_`
have no mutator that skips `rebuildShape()`, and the hover z-raise/restore, the
`addFieldMenu` by-reference capture, the `QAction` connect contexts and the
left-button cursor scoping are all sound. The four deferrals from round 5
(numericFields cache, the destroyed-while-hovered test, the two `useStlAlgorithm`
nits) are unchanged and remain reasonable; cppcheck reports nothing else.

Not re-litigated, per the dispatch: hover-to-inspect, the instant in-scene label,
the pan-mode arrow cursor, numeric-only ramp fields, and show-don't-clear for a
stale persisted style field.

### Findings
- [x] (must-fix) ADR-0016 D14 still states that `numericFields()` "is what the *Color by* and *Size by* menus list" and that the menus "offer only the fields a ramp can read" — as of 852e720 they also list a stale persisted field as "`<field>` (no numbers)" plus "(none)" to clear it. D14's own persisted-field paragraph below stops at what `applyStyle()` does, so a reader concludes the very thing the fix removed: that a stale setting is unreachable from the menu — `docs/decisions/0016-read-only-vector-file-layer.md:325-329,345-353`
- [x] (suggestion) ADR-0016 D16 describes the release-side cursor reset without the left-button scoping that 32c8906 added; `.agents/README.md:130-139` already carries it, so the ADR is now the less precise of the two — `docs/decisions/0016-read-only-vector-file-layer.md:374-376`
- [x] (suggestion) The hover z-raise is an operator-visible behaviour — the hovered feature comes to the front of its layer, not just its label — and is recorded only in the code comment; D5 documents every other part of the hover interaction — `docs/decisions/0016-read-only-vector-file-layer.md:97-113`
- [x] (suggestion, Lens A + Lens B) The geometry-cache contract is inferable only from its two call sites: make it greppable, and state the `setZValue(0.0)` reset's implicit invariant (nothing else may give a feature item a non-default zValue) before someone adds a selected-feature highlight — `src/camp_map/vector/vector_feature_item.h:157-171`, `src/camp_map/vector/vector_feature_item.cpp:368`
- [x] (suggestion, Lens A) No test covers an empty `numericFields()` WITH a stale style field set — the combination the two-part guard in `contextMenu()` is most likely to regress on silently; correct by inspection today — `test/test_vector_layer_teardown.cpp:721`, `src/camp_map/vector/vector_layer.cpp:445-491`
- [x] (suggestion, Lens B) `label_->setZValue(kHoveredZValue)` is a no-op: the label is the item's only child, and Qt compares zValue only among siblings — the item-level lift is what does the work — `src/camp_map/vector/vector_feature_item.cpp:396`

### Dismissed (checked, not findings)
- Governance reported the z-value convention as already recorded in ADR-0016; it is not — `grep -niE "zvalue|raise"` over the ADR returns nothing. Kept as the suggestion above rather than as a pass.
- The stale-field menu entry re-triggers its own setter, which early-returns on an unchanged value (`vector_layer.cpp` `setColorField`/`setSizeField`): clicking it is a visual toggle that does nothing and is rebuilt on the next open. Cosmetic, not a defect.
- The restructured constructor's always-run `rebuildShape()` on a null-anchor item: `loadFinished()` filters through `hasPlaceableCoordinate()` before construction, so the path is unreachable in production and degrades to an empty shape in a unit test — identical to the prior behaviour.
- The hovered item's zValue 1.0 cannot collide with CAMP's other z users (measuring tool 10, platform 6, ship track 5, mission items 3): those are top-level scene items, and zValue orders only siblings under the same parent — here, the features of one vector layer.

## Implementation
**Status**: complete
**When**: 2026-09-15 12:39 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Branch**: feature/issue-22 at `64730c0`
**Addressed**: Local Review (Pre-Push), 2026-09-15 12:30 -04:00, branch `feature/issue-22` at `22ff76d`
**Commits**: f2e2897, de5bc4e, 887c260, 3c188e7, e151ec3, 64730c0

### Actions
- [x] ADR-0016 D14 mentions the stale-field "(no numbers)" and "(none)" menu entries — `docs/decisions/0016-read-only-vector-file-layer.md:325-329` (`f2e2897`)
- [x] ADR-0016 D16 states the left-button scoping of the cursor reset — `docs/decisions/0016-read-only-vector-file-layer.md:374-385` (`de5bc4e`)
- [x] ADR-0016 D5 documents the hover z-raise of the hovered feature — `docs/decisions/0016-read-only-vector-file-layer.md:112-121` (`887c260`)
- [x] Geometry-cache contract and zValue-reset invariant stated in the header/source comments named — `src/camp_map/vector/vector_feature_item.h:161-168`, `src/camp_map/vector/vector_feature_item.cpp:363-369` (`3c188e7`)
- [x] Added `AStaleFieldIsShownEvenWhenNumericFieldsIsEmpty`, covering an empty `numericFields()` with a stale style field set — `test/test_vector_layer_teardown.cpp` (`e151ec3`)
- [x] Removed the no-op `label_->setZValue(kHoveredZValue)`; comment explains the label is the item's only child so the item-level lift does the work. `HoverShowsAnInSceneLabelWithTheAttributes` updated to assert `label->zValue() == 0.0` instead of the stale `> 0.0` expectation it relied on — `src/camp_map/vector/vector_feature_item.cpp:394-399`, `test/test_vector_feature_item.cpp:313-320` (`64730c0`)

### Notes
- No behaviour changed beyond the label zValue removal (action 6), which was itself a no-op removal — the raised z-order behaviour is unchanged, only the redundant statement is gone.
- `plan.md` already states the current menu/zValue behaviour accurately (line 18); no rev bump needed.
- Full local suite: `ui_ws/build.sh camp` then `ui_ws/test.sh camp` — 392 tests, 0 errors, 0 failures, 1 skipped (pre-existing skip, unrelated).

### Next step
review-code (re-review) via `.agent/scripts/dispatch_subagent.sh --mode in-process --issue 22 --skill review-code`

## Integrated Review
**Status**: complete
**When**: 2026-09-15 13:07 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #226 at `136a505`
**Sources**: 3 (Copilot R3 @ `f7aeb8c` — 1 inline + 6 suppressed; Copilot R4 @ `136a505` — 1 inline + 7 suppressed; CI rollup @ `136a505`)
**Cross-source confirmations**: 0 (the two local rounds at/near this head — `## Local Review (Pre-Push)` rounds 5 @ `ce3f65a` and 6 @ `22ff76d` — raise none of the findings below; every local finding of both rounds is checked closed in the `## Implementation` entries at `3c42f69` and `64730c0`)
**CI**: all-pass (`build-and-test` SUCCESS, `copilot-pull-request-reviewer` SUCCESS, both on `136a505`)

Round 3 of triage. Only comments created after 2026-09-15T13:20:00Z are triaged
here (the two rounds before that are the `## Integrated Review` entries of
2026-09-14 14:58 and 2026-09-15 09:31, both fully addressed). The R3 review was
submitted against `f7aeb8c`; each of its findings was re-read against the current
file at `136a505` and none was closed by the commits since — R4 re-raises five of
the six under the same or a neighbouring line. Two items Copilot marks "critical"
are dismissed with the reason below; one is a repeat of a dismissal already
recorded empirically in round 1.

Copilot raised nothing against the settled operator decisions (hover-to-inspect,
the instant in-scene label, the arrow pan cursor, numeric-only ramps,
show-don't-clear for a stale style field) except the label-position item below,
whose actionable half is a stale doc sentence, not the interaction.

### Findings
- [x] (must-fix, Copilot R3+R4) `m_unavailableVectorLayerFiles` is purged by EXACT STRING (`removeAll(fname)`), but its entries are only canonical when the file existed at the time they were written: `canonicalVectorLayerPath()` falls back to the raw spelling for a path that does not resolve. A dangling symlink opened once (the layer is created and persisted even when the file will not open), then persisted, then unavailable at startup, keeps its raw spelling in the list; when the target appears and the operator opens that same symlink, `fname` is now the resolved TARGET and `removeAll` misses the raw entry. `persistVectorLayers()` then keeps writing it back, so removing the reopened layer through the Layers tab does not stick and the layer returns on the next launch — the camp#90/#117 class this PR exists not to repeat, and the same finding round 1 fixed for the non-alias spelling. Purge by canonical-equivalent identity (drop any entry whose `canonicalVectorLayerPath()` equals `fname`, keeping the exact-match removal), with a regression test on the symlink path — `src/camp/autonomousvehicleproject.cpp:435-436`
- [x] (should-fix, Copilot R3+R4) `parseVectorLayers()` binds `diag` to the caller's `ParseDiagnostics` and ACCUMULATES into it; the header documents the parameter as "filled in with what was skipped and why". Reusing one object across two parses carries `aborted`, `geometry_cap_reached`, `layers_total` and every counter into the second result, so an uncapped parse can be reported as capped. No live caller reuses one (`VectorLayer::load()` builds a fresh `LoadResult` per load), so this is a latent API trap rather than a current defect — and the fix is one line: reset a supplied object before binding — `src/camp_map/vector/vector_parse.cpp:331-332`, `src/camp_map/vector/vector_parse.h:155`
- [x] (should-fix, Copilot R3+R4) `firstCoordinate()` accepts an INTERIOR-ring vertex as a polygon's anchor. For a polygon whose exterior vertices are all unplaceable but whose hole has a valid one, `hasPlaceableCoordinate()` admits the item, the constructor closes an empty exterior and `addRing()` builds only the hole — which `Qt::OddEvenFill` then paints as solid fill, turning a hole into a feature. Require a placeable EXTERIOR vertex for a polygon (points and lines have no interior rings, so nothing else changes); the feature is then skipped and counted in the layer's `skipped` tally like any other unplaceable one — `src/camp_map/vector/vector_feature_item.cpp:72-81,155-166`
- [x] (should-fix, Copilot R4) The `!loaded_` status branch never says the file was CAPPED. `capped` comes from `diagnostics.geometry_cap_reached`, so "the cap was hit and every capped geometry was unplaceable" (a `.prj`-less national shapefile is exactly that) reports `(no placeable features; 50000 skipped)` with no hint that the rest of the file was not read — the log line says it, the Layers tab does not. Add the capped note to this path as the loaded path already does — `src/camp_map/vector/vector_layer.cpp:257-268`
- [x] (should-fix, Copilot R3) `geometry_cap_reached` is set immediately after the geometry that reaches `max_geometries`, without establishing that anything remains. A file holding exactly `max_geometries` geometries therefore reports "stopped at the cap; rest of file not read" in the Layers tab although the file was read in full — a false partial-read claim in the one status line this design makes load-bearing. Establish that more input exists (a bounded lookahead over this layer's next feature and any remaining layers) before setting the flag — `src/camp_map/vector/vector_parse.cpp:451-461`
- [x] (should-fix, Copilot R4) The unhandled-geometry `qWarning()` fires once per geometry, and an unhandled geometry does NOT spend the budget, so `max_geometries` bounds nothing here: a large file of curve types can emit unbounded log I/O while the worker reads all of it. `diagnostics.geometries_unhandled` already counts them — report the count once per layer, as the file already does for `points_dropped` and `polygons_without_exterior_ring` — `src/camp_map/vector/vector_parse.cpp:281-284`, `:469-479`
- [x] (should-fix, Copilot R4) The transform path is tested only WGS84 -> WGS84. Every fixture with a spatial reference uses `SetWellKnownGeogCS("WGS84")`, except the `LOCAL_CS` one, which tests transform FAILURE; `LineAndPolygonVerticesAreLatLonWhenTransformed` therefore exercises target-axis order but never reprojection from projected metres. Reading UTM survey data is a must-have of this layer, and a broken projected->WGS84 transform would pass the whole suite today. Add a projected fixture (EPSG:32619 eastings/northings) and assert the resulting lat/lon — `test/test_vector_parse_attributes.cpp:597-637`, `:70,306,370`
- [x] (should-fix, Copilot R4) `updateLabelPosition()`'s doc says a line's or polygon's "label follows the cursor"; it is placed at `event->pos()` on hover-ENTER only and then stays put while the cursor travels along the feature. AGENTS.md forbids documenting from assumption, so the sentence has to match the code. Whether the label should track the cursor within a feature is an operator UX question (hover-to-inspect itself is settled; this detail was never put to the operator) — the fix here is the doc correction, with a `hoverMoveEvent()` left as a question for the operator, not a bot-driven change — `src/camp_map/vector/vector_feature_item.h:157-158`, `src/camp_map/vector/vector_feature_item.cpp:340-347`
- [x] (suggestion, Copilot R3) `addRing()` drops an unplaceable vertex and keeps the subpath open, so the next valid vertex is joined with `lineTo()` — a straight segment across the missing data rather than a break. This is deliberate and argued at the call site, and Copilot's premises are partly wrong (an out-of-range latitude is CLAMPED by `placeableToMap()`, per ADR-0016 D13, not dropped; the parser drops failed transforms before this code sees them, so only a genuinely invalid coordinate from the no-SRS pass-through branch reaches it). What is missing is the CONSEQUENCE in the ADR: D4/D13 record the drop and the clamp but not that a dropped mid-ring vertex leaves a fabricated segment. Record it in ADR-0016; do not change the drawing (splitting the subpath would break a polygon's fill and is a bigger decision than this MVP) — `src/camp_map/vector/vector_feature_item.cpp:26-46`, `docs/decisions/0016-read-only-vector-file-layer.md:85-95,323-336`

### False positives
- (Copilot R3, `src/camp/autonomousvehicleproject.cpp:451`, flagged critical) "The `destroyed` connection can call `onVectorLayerDestroyed()` after `~AutonomousVehicleProject` has destroyed `m_vectorLayers`, so the slot iterates a destroyed `std::vector`." Repeat of the round-1 claim, dismissed then on measurement and unchanged since: `~QObject` removes every connection in which the object is the RECEIVER before `deleteChildren()` runs, so by the time the child `Map`'s `VectorLayer`s are destroyed there is no connection left to the slot. Verified empirically on this host's Qt 5.15.13 with a minimal parent/child/`destroyed`-slot program — the slot does not fire. The member-vs-child destruction ORDER Copilot describes is correct; the conclusion does not follow from it.
- (Copilot R4, `src/camp_map/vector/vector_feature_item.cpp:196`) "At `MapView`'s minimum scale the cosmetic outline extends ~50 000 scene metres beyond the polygon's bounding rect, so items near a viewport edge are culled with their visible outline missing." The pen is COSMETIC: its width is one device pixel at every zoom, so the un-covered margin is one pixel at every zoom too — the 50 000 m figure is that same one pixel expressed in scene units at `min_zoom_scale_` (1/50000), not a 50 km visual loss. The worst case is a one-pixel sliver of outline at the very edge of the viewport, which is exactly the "one-pixel overdraw at the edges" the `boundingRect()` comment already records as accepted. No Qt undefined behaviour either: for a polygon `shape_` IS `path_`, so the hit shape is inside the bounding rect, and the line branch is grown by the stroker half-width for that reason.

### Previously triaged, unchanged
- (Copilot R4, `src/camp_map/vector/vector_parse.cpp:146`) "A single ring's vertex count is unbounded, so one-part files defeat the memory bound." Correct, and already triaged in round 2 as a deferred OPERATOR decision: silently truncating a ring draws a wrong shape, which is worse than the honest geometry cap, so the options (drop-and-count vs accept-and-say-so) are folded into the deferred ADR-0016 D11 decision, which states in terms that neither the vertex count nor an attribute value size is bounded. No overclaim to correct and no new action.

## Implementation
**Status**: complete
**When**: 2026-09-15 13:34 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #226 at `7c0dfb7`
**Addressed**: `## Integrated Review` of 2026-09-15 13:07 -04:00 (round 3, PR #226 at `136a505`)
**Commits**: `9bf8ba6`, `76c0790`, `4433e4a`, `e20c006`, `704c99c`, `17e47d8`, `3d10e9c`, `0120983`, `280402a`, `7c0dfb7`

All nine open findings actioned at the operator's "fix all 9" decision; none
deferred. The two false positives and the previously-triaged item were left
alone, and no settled operator decision (hover label, arrow cursor, numeric-only
ramps) was reopened.

Full camp suite after the last fix: **401 tests, 0 failures, 1 skipped** (up from
394 — seven new tests).

### Actions
- [x] (must-fix) Unavailable-layer purge now drops any entry whose
  `canonicalVectorLayerPath()` equals the opened file, keeping the exact-match
  removal. `canonicalVectorLayerPath()` and the new `withoutVectorLayerFile()`
  live in `camp::vector` so the rule is testable (the project is not
  constructible in a harness) and `AutonomousVehicleProject` delegates to both.
  Regression test drives the dangling-symlink-then-target-appears lifecycle over
  a real temporary symlink, plus the exact-match and leave-alone cases —
  `src/camp/autonomousvehicleproject.cpp:435-445`,
  `src/camp_map/vector/vector_layer.cpp:563-590`,
  `test/test_vector_layer_persistence.cpp` (`ReopenedDanglingSymlinkCanBeRemoved`,
  `WithoutVectorLayerFileKeepsUnrelatedEntries`) — `9bf8ba6`
- [x] (should-fix) A caller-supplied `ParseDiagnostics` is reset before the
  parser binds to it, so a reused object cannot carry `aborted`,
  `geometry_cap_reached` or a counter into the next parse; header contract
  updated to say so — `src/camp_map/vector/vector_parse.cpp:331-341`,
  `vector_parse.h`, test `SuppliedDiagnosticsAreResetPerParse` — `76c0790`
- [x] (should-fix) `firstCoordinate()` no longer falls back to interior rings, so
  a polygon whose exterior is entirely unplaceable is skipped and counted rather
  than drawn from its hole (which `Qt::OddEvenFill` paints as fill). Points and
  lines carry no interior rings, so nothing else changes —
  `src/camp_map/vector/vector_feature_item.cpp:72-85`, `vector_feature_item.h`,
  test `PolygonWithUnplaceableExteriorIsRejectedDespiteAValidHole` — `4433e4a`
- [x] (should-fix) The capped note now rides every branch of the not-loaded
  status path, so "the cap was hit and every capped geometry was unplaceable"
  says the rest of the file was not read —
  `src/camp_map/vector/vector_layer.cpp:257-282`, test
  `CappedButEmptyLayerStillReportsTheUnreadRemainder` (with a
  `waitForStatus()` helper, since `waitForLoad()` answers `loaded()`, false by
  design for an empty layer) — `e20c006`
- [x] (should-fix) `geometry_cap_reached` is set only once a bounded lookahead
  establishes that input remains: the current feature's unread parts (new
  `ParseBudget::input_remaining`), then one feature on the current layer, then at
  most one per remaining layer — `src/camp_map/vector/vector_parse.cpp:349-386,
  480-497`, tests `CapAtExactlyTheFileSizeIsNotAPartialRead` and
  `CapInsideAFeatureReportsOnlyAnActualRemainder` — `704c99c`
- [x] (should-fix) One unhandled-geometry warning per layer (count + first type
  seen, carried by the new `ParseDiagnostics::first_unhandled_geometry_type`),
  following the `points_dropped` pattern, instead of one qWarning per geometry
  on a path the geometry budget does not bound —
  `src/camp_map/vector/vector_parse.cpp:281-296, :505-513`, test
  `UnhandledGeometriesAreReportedOncePerLayer` (counts the messages through a
  temporary `qInstallMessageHandler`) — `17e47d8`
- [x] (should-fix) Projected-CRS reprojection fixture: an EPSG:32619 (UTM 19N)
  GeoPackage whose eastings/northings are asserted back as lat/lon, with the
  expected values computed through the same PROJ this build links —
  `test/test_vector_parse_attributes.cpp`
  (`ProjectedCoordinatesAreReprojectedToLatLon`) — `3d10e9c`
- [x] (should-fix) The "follows the cursor" claim corrected in
  `updateLabelPosition()`'s doc, at the `hoverEnterEvent()` call site, in the
  test's name and message, and in ADR-0016 D5 (which stated label placement
  nowhere). A `hoverMoveEvent()` is recorded as an open operator question, not
  made — `src/camp_map/vector/vector_feature_item.h:155-168`,
  `vector_feature_item.cpp:340-352`, `docs/decisions/0016-*.md` — `0120983`
- [x] (suggestion) ADR-0016 D13 now records the consequence D4/D13 left unsaid: a
  dropped mid-ring vertex leaves a fabricated straight segment, bounded by the
  clamp and by the parser's earlier drop. The drawing is unchanged —
  `docs/decisions/0016-read-only-vector-file-layer.md` — `280402a`

### Documents kept in sync
- `plan.md` — **rev 14** (the plan's head was rev 13; no rev 14 existed) records
  the five behaviour changes and the new tests.
- `.agents/README.md` — the vector paragraph's "three of them bite" list gains
  the canonical-identity purge and the capped-but-empty status, and is now four.

### Next step
Lifecycle: **Implementation** -> **review-code** (re-review the fixes). Not
dispatched from here; the host orchestrator drives. Nothing was pushed.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-15 13:42 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-22 at `faa58a1`
**Mode**: pre-push
**Depth**: Deep (reason: cross-layer camp <-> camp_map, GDAL input handling, worker-thread lifecycle) — horizon scoped to the 11 commits since `861bcae`
**Must-fix**: 2 | **Suggestions**: 5
**Round**: 7 | **Ship**: recommended — both must-fixes are precise mechanical corrections (a per-layer/per-parse scope mix in a log line; a missing abort poll in the new lookahead) with obvious fixes; no design question is open, no code finding from rounds 1-6 regressed, and all nine round-3 findings verified closed against the code rather than the claim.

Specialists: Static Analysis (cppcheck 2.13 + repo pre-commit, both clean on touched lines),
Governance, Plan Drift, Claude Adversarial Lens A + Lens B. Copilot and local review off (default).
Build and tests re-run here: `./ui_ws/build.sh camp` clean, `./ui_ws/test.sh camp`
→ **401 tests, 0 errors, 0 failures, 1 skipped**, matching the Implementation entry's claim.
All 11 commits carry the agent identity; working tree clean.

### Findings
- [x] (must-fix) `first_unhandled_geometry_type` is set once per PARSE but printed as that LAYER's first type, beside a count that is per-layer — layer 2 of a mixed dataset logs layer 1's type; reset/snapshot it at layer entry like `unhandled_before` and extend the test to two layers — `src/camp_map/vector/vector_parse.cpp:303-305`, `:553-560`, `src/camp_map/vector/vector_parse.h:137`
- [x] (must-fix) `moreInputRemains()` polls no abort predicate, so after an abort is requested the worker still runs up to one unabortable `GetNextFeature()` per remaining layer on the path the destructor's GUI-thread join depends on; take `budget.aborted` and return early (the value is cosmetic — an aborted result is discarded) — `src/camp_map/vector/vector_parse.cpp:392-415`, called at `:535`
- [x] (suggestion) The lookahead's multi-layer branch is untested — all three cap tests use a single-layer fixture; add a 2-layer case with the cap on layer 0's last feature, layer 1 empty vs. one feature — `src/camp_map/vector/vector_parse.cpp:401-412`
- [x] (suggestion) `withoutVectorLayerFile()` runs a `canonicalFilePath()` stat/readlink per unavailable entry synchronously on the GUI thread, and its own docstring's motivating case is an unmounted share; bounded by list length and an amplification of the adjacent pre-existing `QFileInfo::exists()`, worth acknowledging — `src/camp/autonomousvehicleproject.cpp:437-443`
- [x] (suggestion) ADR-0016 D11 does not record that `geometry_cap_reached` now means input was actually left unread, established by a bounded lookahead — the rule the Layers-tab status leans on lives only in code comments and plan rev 14 — `docs/decisions/0016-read-only-vector-file-layer.md:252-284`
- [x] (suggestion) Plan rev 14's new-tests bullet omits `WithoutVectorLayerFileKeepsUnrelatedEntries` — `.agent/work-plans/issue-22/plan.md:42-46`
- [x] (suggestion) Plan step 1's "documented, logged skip" passage still reads as the pre-round-3 per-geometry warning; not false, but a reader working from the Approach section would miss the change — `.agent/work-plans/issue-22/plan.md:519-532`

## Implementation
**Status**: complete
**When**: 2026-09-15 13:56 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-22 at `ac14fd8`
**PR**: #226
**Addressed**: `## Local Review (Pre-Push)` (2026-09-15 13:42 -04:00, Round 7, verdict changes-requested, at `faa58a1`) — all 7 open findings: 2 must-fix + 5 suggestions, per the host-decided scope.
**Commits**: e5eddb1, f9f25a5, 5c2f703, 246f0cd, e4fa456, ac14fd8

Build clean (`./ui_ws/build.sh camp`); `./ui_ws/test.sh camp` →
**402 tests, 0 errors, 0 failures, 1 skipped** (401 before; the new two-layer
cap-lookahead test is the +1, the two-layer unhandled-type case extended an
existing test rather than adding one).

The per-layer scope fix was verified to be load-bearing: with the guard
commented out and everything else unchanged, the extended test fails on
`layer compound - skipped 2 geometry(ies) of an unhandled type (first: "Circular String")`
— layer 1's type on layer 2's line, exactly the defect reported.

Two judgement calls, both recorded in the commits that made them:

- **No test for the abort poll in `moreInputRemains()`.** The feature loop's
  abort branch runs *before* the cap branch, so reaching the lookahead with the
  flag already raised means it rose inside that window. Driving that from the
  public API needs a predicate keyed to the parser's internal poll count, which
  pins an implementation detail rather than the contract.
- **The GUI-thread stat in `withoutVectorLayerFile()` is documented, not
  mitigated.** The only mitigation is an asynchronous purge, which would race
  `persistVectorLayers()` and let the camp#90/#117 write-back return — a worse
  trade than a bounded stat over an operator-sized list. The cost, its bound,
  and that reasoning are now on the function's declaration, with a pointer at
  the call site.

### Actions
- [x] (must-fix) `first_unhandled_geometry_type` scoped per layer — RAII guard at layer entry restores the parse-wide first type on every exit, including the abort and cap early returns; test extended to two layers of two curve types — `src/camp_map/vector/vector_parse.cpp`, `vector_parse.h`, `test/test_vector_parse_attributes.cpp` (e5eddb1)
- [x] (must-fix) `moreInputRemains()` polls `budget.aborted` on entry and between layers, returning false — `src/camp_map/vector/vector_parse.cpp` (f9f25a5)
- [x] (suggestion) Two-layer cap fixture covers the lookahead's multi-layer branch: cap on layer 0's last feature, trailing layer empty vs. one feature — `test/test_vector_parse_attributes.cpp` (5c2f703)
- [x] (suggestion) GUI-thread `canonicalFilePath()` cost documented with its bound and why an async purge is not the trade — `src/camp_map/vector/vector_layer.h`, `src/camp/autonomousvehicleproject.cpp` (246f0cd)
- [x] (suggestion) ADR-0016 D11 records the "cap flag means input was ACTUALLY left unread" rule and its bounded lookahead — `docs/decisions/0016-read-only-vector-file-layer.md` (e4fa456)
- [x] (suggestion) Plan rev 14's new-tests bullet now lists `WithoutVectorLayerFileKeepsUnrelatedEntries` — `.agent/work-plans/issue-22/plan.md` (ac14fd8)
- [x] (suggestion) Plan step 1's "documented, logged skip" passage corrected (per-layer summary, and `wkbGeometryCollection` is handled), plus a rev 15 entry for this round — `.agent/work-plans/issue-22/plan.md` (ac14fd8)

## Integrated Review
**Status**: complete
**When**: 2026-09-15 14:51 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #226 at `1927c69`
**Sources**: 2 (Copilot R5 @ `1927c69` — 1 inline + 6 suppressed, submitted 2026-09-15 18:30 UTC; CI rollup @ `1927c69`)
**Cross-source confirmations**: 0 (the local sources at this head — `## Local Review (Pre-Push)` round 7 @ `faa58a1` and the `## Implementation` that closed it at `ac14fd8`/`1927c69` — raise none of the findings below; every round-7 finding is checked closed and none regressed)
**CI**: all-pass (`build-and-test` SUCCESS, `copilot-pull-request-reviewer` SUCCESS, both on `1927c69`)

Round 4 of triage. Only comments created after 2026-09-15T17:05:00Z are triaged
here — that is exactly the Copilot review of 18:30 UTC against the current head;
the four earlier reviews are the rounds already recorded as `## Integrated
Review` entries (2026-09-14 14:58, 2026-09-15 09:31, 2026-09-15 13:07) and every
finding they raised is checked closed. Copilot's inline comment and its first
suppressed comment are the same defect seen from the two ends of the same data
flow and are recorded once, below, with both sites.

Copilot raised nothing against a settled operator decision (hover-to-inspect,
the instant in-scene label, the arrow pan cursor, numeric-only ramps,
show-don't-clear for a stale style field, the canonical-equivalent purge, the
documented-not-mitigated GUI-thread stat). The one finding that touches the
canonical-identity purge EXTENDS it to the sibling list rather than reopening it.

### Findings
- [ ] (should-fix, Copilot R5) **The restored ORDER is not canonical-identity aware, so a reopened once-unavailable layer loses its slot.** `restorePersistedVectorLayers()` stores `canonicalVectorLayerPath(fname)` in `m_restoredVectorLayerOrder`, and for a path that does not resolve at startup (a dangling symlink, an unmounted-share spelling that differs from its resolved form) that is the RAW spelling. Round 3 fixed the sibling list — `m_unavailableVectorLayerFiles` is now purged by canonical-equivalent identity when the file is reopened — but `m_restoredVectorLayerOrder` keeps the raw entry, while `loaded` holds the resolved TARGET. `rebuildPersistedVectorLayerFiles()` then matches the restored slot by literal `loadedFiles.contains(restored)`, misses, skips the slot, and the trailing append loop puts the reopened file LAST: `[dangling link, B]` persists as `[B, target]`. No layer is lost and removal still sticks, so this is an ordering regression, not the camp#90/#117 class — but layer order is the operator's stacking order and `restorePersistedVectorLayers()` documents the slot as kept. Preferred fix: when `openVectorLayer()` promotes a path out of the unavailable list, replace the canonical-equivalent entry in `m_restoredVectorLayerOrder` with `fname` in place (one resolve pass, only on promotion), rather than making `rebuildPersistedVectorLayerFiles()` stat every entry on every persist. Regression test: unavailable link first, another layer second, reopen the link, assert `[target, B]` — `src/camp/autonomousvehicleproject.cpp:443-446`, `src/camp_map/vector/vector_layer.cpp:602-620`, `test/test_vector_layer_persistence.cpp` (`ReopenedDanglingSymlinkCanBeRemoved` drives this lifecycle with `loaded` EMPTY, which is why the gap is untested)
- [ ] (should-fix, Copilot R5) **Per-layer style has no QSettings round-trip test.** `writeSettings()`/`readSettings()` persist `color_field`, `size_field` and `colormap` under `MapItem/<settingsKey()>`, and the whole point of the path-based key (camp#126) is that a style survives reopening. The suite checks only that two layers' `settingsKey()` DIFFER (`SettingsKeyIsPathNotBasename`); `test_vector_layer_styling.cpp` exercises the free mapping functions with no QSettings at all. A mistyped key, a group mismatch, or a lost `readSettings()`/`applyStyle()` call would leave every persisted style silently forgotten and the suite green. `VectorLayer` IS constructible in the harness (that test builds two), and `test_range_persist.cpp` is the established pattern for exposing the protected hooks. Add: set non-default values on one layer, construct another for the same path, assert all three restored — `src/camp_map/vector/vector_layer.cpp:516-543`, `test/test_vector_layer_persistence.cpp:117-135`
- [ ] (should-fix, Copilot R5) **A line or polygon whose exterior loses every vertex is still emitted and still SPENDS a geometry-cap slot.** `readRing()` drops each vertex whose transform fails (counting `points_dropped`); when it drops them all, the `wkbLineString` and `wkbPolygon` branches push the empty `ParsedGeometry` anyway and call `budget.spend()`. `VectorLayer` then rejects it (`hasPlaceableCoordinate()` needs an exterior vertex — the round-3 fix) and counts it as unplaceable, so nothing wrong is DRAWN; the cost is that a run of out-of-domain features burns cap slots that later valid features needed, and a mixed file can report the cap reached having produced fewer items than the cap. The `wkbPoint` branch above already gets this right — it counts the drop and breaks WITHOUT emitting or spending — so the two branches are inconsistent with the rule the same function already states. Skip the emission when the exterior comes back empty (for the polygon, before reading its holes), and cover an all-dropped line in the cap test — `src/camp_map/vector/vector_parse.cpp:205-216`, `:228-252`
- [ ] (should-fix, Copilot R5) **The cap-path early return skips the per-layer diagnostic summaries.** `return result;` at the geometry cap sits ABOVE the three `qWarning()` blocks, so a layer that hit the cap never reports its `points_dropped`, `polygons_without_exterior_ring` or `geometries_unhandled` counts. `VectorLayer` re-reports `polygons_without_exterior_ring` and `layers_failed` itself, but nothing anywhere reports `points_dropped` or `geometries_unhandled` — so on the one path where the operator is already being told the read was partial, two of the four "what was left out" diagnostics vanish entirely. Unlike the abort early return (whose result the caller discards by contract), the cap result is CONSUMED. Emit the accumulated per-layer summaries before the cap return — hoisting the three blocks into a lambda called at both exits keeps them from drifting — `src/camp_map/vector/vector_parse.cpp:588-592`, `:600-618`
- [ ] (suggestion, Copilot R5) **The lookahead can leave `ParseOptions::aborted` true with `ParseDiagnostics::aborted` false, contradicting the header.** `moreInputRemains()` polls the abort predicate on entry and between layers, but the flag can rise during the `GetNextFeature()` it then runs; the per-feature `aborted()` check has already passed for this iteration, so the cap branch returns with `aborted` unset. The header states in terms that when the predicate returns true "the parse stops where it is and returns what it has, with `ParseDiagnostics::aborted` set" — that promise is violated in this window. No operator-visible failure can occur today: the abort flag is set ONLY in `~VectorLayer`, which then joins the worker and is destroyed, so `loadFinished()` never runs and no result is consumed after an abort. It is still a one-line correction to a documented contract on a public API: re-check the predicate after the lookahead and set `diag.aborted` before returning. (Note the race also cuts the other way, and that half is the reason not to leave it: `giveUp()` makes the lookahead answer "nothing remains", so a genuinely capped parse could return `geometry_cap_reached = false` — a "read in full" claim in the status line this design leans on — reachable only through the same unconsumed path.) — `src/camp_map/vector/vector_parse.cpp:393-429`, `:588-591`, `src/camp_map/vector/vector_parse.h:70-79`

### False positives
- (Copilot R5, `src/camp_map/vector/vector_parse.cpp:589`) "A caller that consumes the parser result can treat a cancellation partial as a valid capped parse." The cancellation-partial half of the claim cannot occur: everything in the returned data was built BEFORE this iteration's `aborted()` check passed, so no ring in it was truncated by the abort — the result is a complete parse up to the cap, which is precisely what `geometry_cap_reached` describes. And there is no consuming caller: `abort_flag_` is written in exactly one place, `~VectorLayer`, which joins the worker and destroys the object, so `loadFinished()` — the only consumer — never runs on an aborted parse. What survives of the comment is the header-contract inaccuracy, recorded as the suggestion above rather than as the defect Copilot described.

### Previously triaged, unchanged
- Nothing re-raised from rounds 1-3. The two standing dismissals (the `destroyed`-slot use-after-free, measured on this host's Qt; the cosmetic-pen bounding-rect margin) and the deferred unbounded-ring/attribute-size item (ADR-0016 D11) were not raised again in R5.

## Integrated Review
**Status**: complete
**When**: 2026-09-17 08:17 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #226 at `5d53d98`
**Sources**: 3 (Copilot R6 @ `5d53d98` — 0 inline + 9 suppressed, submitted 2026-09-15 19:09:45 UTC; the round-4 `## Integrated Review` @ `1927c69` whose five findings are still open; CI rollup @ `5d53d98`)
**Cross-source confirmations**: 5
**CI**: all-pass (`build-and-test` SUCCESS, `copilot-pull-request-reviewer` SUCCESS, both on `5d53d98`)

Round 5 of triage, and the first round with cross-source confirmations. Only the
Copilot review of 2026-09-15 19:09:45 UTC is new; the five earlier reviews are the
rounds already recorded as `## Integrated Review` entries (2026-09-14 14:58,
2026-09-15 09:31, 13:07, 14:51). The only commit since `1927c69` is `5d53d98`,
which touches progress.md alone — so every open finding of the round-4 entry was
re-read against the current files and every one is unchanged and still open. They
are carried forward here, unchecked, so `address-findings` reads ONE complete
list; five of them are re-raised by R6 at the same or a neighbouring line and are
marked `(cross-confirmed)` accordingly. Two findings are genuinely new, and one
R6 comment is a re-raise of a dismissal already recorded in round 3.

Copilot again raised nothing against a settled operator decision — except the new
label-hit-test finding, which does not question hover-to-inspect but reports that
the instant in-scene label REGRESSES the pan-start-on-feature guarantee the same
decision rests on.

### Findings
- [x] (cross-confirmed: Copilot R5 + R6 @ `5d53d98`, should-fix) **The restored ORDER is not canonical-identity aware, so a reopened once-unavailable layer loses its slot.** `restorePersistedVectorLayers()` stores `canonicalVectorLayerPath(fname)` in `m_restoredVectorLayerOrder`, and for a path that does not resolve at startup (a dangling symlink, an unmounted-share spelling) that is the RAW spelling. Round 3 fixed the sibling list — `m_unavailableVectorLayerFiles` is purged by canonical-equivalent identity on reopen (`autonomousvehicleproject.cpp:444-446`, verified unchanged) — but `m_restoredVectorLayerOrder` keeps the raw entry while `loaded` holds the resolved TARGET. `rebuildPersistedVectorLayerFiles()` then matches by literal `loadedFiles.contains(restored)`, misses, skips the slot, and the trailing append loop puts the reopened file LAST: `[dangling link, B]` persists as `[B, target]`. No layer is lost and removal still sticks, so this is an ordering regression, not the camp#90/#117 class — but layer order is the operator's stacking order and `restorePersistedVectorLayers()` documents the slot as kept. R6 names the same fix independently: when `openVectorLayer()` promotes a path out of the unavailable list, replace the canonical-equivalent entry in `m_restoredVectorLayerOrder` with `fname` in place (one resolve pass, only on promotion), rather than making `rebuildPersistedVectorLayerFiles()` stat every entry on every persist. Regression test: unavailable link first, another layer second, reopen the link, assert `[target, B]` — `src/camp/autonomousvehicleproject.cpp:443-446`, `src/camp_map/vector/vector_layer.cpp:602-620`, `test/test_vector_layer_persistence.cpp` (`ReopenedDanglingSymlinkCanBeRemoved` drives this lifecycle with `loaded` EMPTY, which is why the gap is untested)
- [x] (cross-confirmed: Copilot R5 + R6 @ `5d53d98`, should-fix) **Per-layer style has no QSettings round-trip test.** `writeSettings()`/`readSettings()` persist `color_field`, `size_field` and `colormap` under `MapItem/<settingsKey()>`, and the whole point of the path-based key (camp#126) is that a style survives reopening. The suite checks only that two layers' `settingsKey()` DIFFER (`SettingsKeyIsPathNotBasename`); `test_vector_layer_styling.cpp` exercises the free mapping functions with no QSettings at all. A mistyped key, a group mismatch, or a lost `readSettings()`/`applyStyle()` call would leave every persisted style silently forgotten and the suite green. `VectorLayer` IS constructible in the harness (that test builds two), and `test_range_persist.cpp` is the established pattern for exposing the protected hooks. Add: set non-default values on one layer, construct another for the same path, assert all three restored — `src/camp_map/vector/vector_layer.cpp:516-543`, `test/test_vector_layer_persistence.cpp:117-135`
- [x] (cross-confirmed: Copilot R5 + R6 @ `5d53d98` — R6 files the two branches separately, should-fix) **A line or polygon whose exterior loses every vertex is still emitted and still SPENDS a geometry-cap slot.** `readRing()` drops each vertex whose transform fails (counting `points_dropped`); when it drops them all, the `wkbLineString` branch pushes the empty `ParsedGeometry` and calls `budget.spend()` regardless (`vector_parse.cpp:206-218`, unchanged), and the `wkbPolygon` branch does the same after also walking every interior ring (`:220-255`). `VectorLayer` then rejects the item (`hasPlaceableCoordinate()` needs an exterior vertex — the round-3 fix) and counts it unplaceable, so nothing wrong is DRAWN; the cost is that a run of out-of-domain features burns cap slots later valid features needed, and a mixed file can report the cap reached having produced fewer items than the cap. The `wkbPoint` branch above already gets this right — it counts the drop and breaks WITHOUT emitting or spending — so the branches contradict a rule the same function already follows. Skip the emission when the exterior comes back empty (for the polygon, RETURN before reading its holes, which also saves the ring walk), and cover an all-dropped line in the cap test — `src/camp_map/vector/vector_parse.cpp:206-218`, `:220-255`
- [x] (cross-confirmed: Copilot R5 + R6 @ `5d53d98`, should-fix) **The cap-path early return skips the per-layer diagnostic summaries.** `return result;` at the geometry cap (`vector_parse.cpp:589-591`) sits ABOVE the three `qWarning()` blocks at `:596-618`, so a layer that hit the cap never reports its `points_dropped`, `polygons_without_exterior_ring` or `geometries_unhandled` counts. `VectorLayer` re-reports `polygons_without_exterior_ring` and `layers_failed` itself, but nothing anywhere reports `points_dropped` or `geometries_unhandled` — so on the one path where the operator is already being told the read was partial, two of the four "what was left out" diagnostics vanish entirely. Unlike the abort early return (whose result the caller discards by contract), the cap result is CONSUMED. Emit the accumulated per-layer summaries before the cap return — hoisting the three blocks into a lambda called at both exits keeps them from drifting — `src/camp_map/vector/vector_parse.cpp:588-592`, `:596-618`
- [x] (cross-confirmed: Copilot R5 + R6 @ `5d53d98`, suggestion) **The lookahead can leave `ParseOptions::aborted` true with `ParseDiagnostics::aborted` false, contradicting the header.** `moreInputRemains()` polls the abort predicate on entry and between layers (`vector_parse.cpp:403-421`), but the flag can rise during the `GetNextFeature()` it then runs; the per-feature `aborted()` check has already passed for this iteration, so the cap branch returns with `aborted` unset. The header states that when the predicate returns true "the parse stops where it is and returns what it has, with `ParseDiagnostics::aborted` set" — that promise is violated in this window. No operator-visible failure can occur today: the abort flag is set ONLY in `~VectorLayer`, which then joins the worker and is destroyed, so `loadFinished()` never runs and no result is consumed after an abort. It is still a one-line correction to a documented contract on a public API: re-check the predicate after the lookahead and set `diag.aborted` before returning. (The race also cuts the other way, which is the reason not to leave it: `giveUp()` makes the lookahead answer "nothing remains", so a genuinely capped parse could return `geometry_cap_reached = false` — a "read in full" claim in the status line this design leans on.) — `src/camp_map/vector/vector_parse.cpp:393-429`, `:588-591`, `src/camp_map/vector/vector_parse.h:100-119`
- [x] (should-fix, Copilot R6) **The hover label is a child item that still accepts mouse buttons, reopening the pan-start-on-feature failure for lines and polygons.** `VectorFeatureItem`'s constructor sets `setAcceptedMouseButtons(Qt::NoButton)` and says in terms why (ADR-0016 D5: every press over a feature must fall through to `QGraphicsView` so a pan gesture started on a feature pans — camp#225 — and so `ProjectView`'s add-* placement clicks are never swallowed). `labelItem()` then creates `label_ = new QGraphicsSimpleTextItem(this)` and sets font/brush/pen/flags but NOT its accepted buttons, so the child keeps `QGraphicsItem`'s default of accepting the left button. For a line or polygon `hoverEnterEvent()` puts the label at `event->pos()` — its top-left corner exactly under the cursor, and it does not track the cursor afterwards (no `hoverMoveEvent()`), so any small move down-and-right while still inside the feature leaves the cursor squarely over the text. A press there is delivered to the label, not to the view, and the pan never starts. The item is also lifted to `kHoveredZValue` for the duration of the hover, so it is the topmost candidate under the cursor. The guarantee is only true by construction if it is set on the whole subtree: add `label_->setAcceptedMouseButtons(Qt::NoButton);` in `labelItem()`, next to the flags, with the same [camp#22 / ADR-0016 D5] reference so the pairing is not lost again. Worth a test that presses on a hovered line's label and asserts the event is not accepted — `src/camp_map/vector/vector_feature_item.cpp:122-128`, `:337-366`, `:402-427`
- [x] (nit, Copilot R6) **The cap is documented and REPORTED in "features", but it counts emitted geometry parts.** `ParseOptions::max_geometries` is spent once per emitted `ParsedGeometry` (`vector_parse.h:80-91`, and `ParseDiagnostics` is explicit that "an OGR feature count is not a geometry count — one multi-part feature emits several"). A `MultiPolygon`, `MultiLineString` or `GeometryCollection` recurses into one `ParsedGeometry` per part (`vector_parse.cpp:258-288`), and `VectorLayer` builds one `VectorFeatureItem` per part, so `features_.size()` is an ITEM count. `vector_layer.h:77-82` nevertheless promises "the first `kMaxFeatureItems` features", and the user-facing strings say the same: `"%1 features"`, `"stopped at the %1-feature cap"`, `"(no placeable features; %1 skipped)"` and the `qWarning()` at `vector_layer.cpp:239-243`. On a KML placemark file or a multipolygon coastline the Layers tab reports a feature count the file does not have. Nothing is mis-drawn and the cap serves its stated purpose (bounding GUI-thread items and worker memory) exactly — the constant's own NAME, `kMaxFeatureItems`, is the accurate one. Fix is wording, not behaviour: say "items" / "drawn geometries" in the class doc and in the four status/log strings, or count source features consistently. Pick one and make the header, the status line and the log agree — `src/camp_map/vector/vector_layer.h:65-82`, `src/camp_map/vector/vector_layer.cpp:237-243`, `:262-284`

### False positives
- (Copilot R6, `src/camp_map/vector/vector_parse.cpp:589`) "A caller that consumes the parser result can treat a cancellation as a complete capped parse" — carried unchanged from round 4, where the same claim was dismissed at the same line: everything in the returned data was built BEFORE this iteration's `aborted()` check passed, so no ring in it was truncated by the abort; the result is a complete parse up to the cap, which is what `geometry_cap_reached` describes. And there is no consuming caller — `abort_flag_` is written in exactly one place, `~VectorLayer`, which joins the worker and destroys the object, so `loadFinished()` never runs on an aborted parse. What survives of the comment is the header-contract inaccuracy, carried above as the suggestion, not the defect Copilot describes.

### Previously triaged, unchanged
- (Copilot R6, `src/camp_map/vector/vector_parse.cpp:153`) "`readRing()` appends every transformed vertex to an unbounded vector, so one ring of millions of vertices defeats the memory bound." Correct, and a third raising of the item dismissed in rounds 2 and 3: silently truncating a ring DRAWS A WRONG SHAPE, which is worse than the honest geometry cap, so the options (drop-and-count vs accept-and-say-so) are folded into the deferred ADR-0016 D11 operator decision, which states in terms that neither the vertex count nor an attribute value size is bounded. The abort-latency half of the concern — the GUI-thread join waiting out a huge ring — was fixed in round 2 (`kVertexPollInterval`, `vector_parse.cpp:122`, `:143-148`). No overclaim to correct and no new action; if the repetition is unwanted, D11 is the place to settle it, not this PR.
- The two standing dismissals from rounds 1-3 — the `destroyed`-slot use-after-free (measured on this host's Qt) and the cosmetic-pen bounding-rect margin — were not raised again in R6.
- All five round-4 findings are carried forward above rather than closed: nothing between `1927c69` and `5d53d98` touched source (the single commit is a progress.md append), and each was re-read at its cited lines to confirm it.

### Next step
address-findings (7 open: 5 carried + 2 new; 5 cross-confirmed) via `.agent/scripts/dispatch_subagent.sh --mode in-process --issue 22 --skill address-findings`

## Implementation
**Status**: complete
**When**: 2026-09-17 08:35 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #226 at `607a6a9`
**Addressed**: the `## Integrated Review` of 2026-09-17 08:17 -04:00 (round 5, PR #226 @ `5d53d98`) — all 7 open findings
**Commits**: d3ba0da, b06e3f7, 5573b7d, a09da39, bcff67d, fe95514, 76c2f4c, 607a6a9

Every finding was fixed; none was deferred. Each fix is its own commit, and each
behaviour change carries a regression test that was confirmed to FAIL against the
pre-fix source (checked by stashing the source edit and re-running) except where
noted. `./ui_ws/build.sh camp` and `./ui_ws/test.sh camp` both clean:
**410 tests, 0 errors, 0 failures, 1 skipped** (the skip is the pre-existing
GL-dependent one).

### Actions
- [x] Restored ORDER is now canonical-identity aware: `withVectorLayerFilePromoted()` rewrites the raw-spelling entry in place on the same promotion that purges the unavailable list, so a reopened dangling symlink keeps its slot instead of being appended last — `src/camp_map/vector/vector_layer.cpp:602-618`, `src/camp/autonomousvehicleproject.cpp:443-461`; tests `ReopenedDanglingSymlinkKeepsItsSlot`, `PromotionRewritesOnlyTheMatchingEntry` (d3ba0da)
- [x] Per-layer style QSettings round trip pinned: a `TestableVectorLayer` (the `test_range_persist.cpp` pattern) sets all three values on one layer and asserts a second layer on the same path restores them, plus that a different path is untouched — `test/test_vector_layer_persistence.cpp` `StyleRoundTripsThroughSettings` (b06e3f7)
- [x] A line or polygon whose exterior loses every vertex is neither emitted nor charged to the geometry cap; the polygon returns before walking its holes — `src/camp_map/vector/vector_parse.cpp:206-232`, `:234-276`; test `AllDroppedExteriorIsNeitherEmittedNorCharged` (fails without the fix: 5 geometries instead of 3) (5573b7d)
- [x] The cap-path early return now emits the per-layer diagnostic summaries: the three `qWarning` blocks are one lambda called at both CONSUMED exits (the abort exit deliberately does not call it — that result is discarded by contract) — `src/camp_map/vector/vector_parse.cpp`; test `CapPathStillReportsTheLayerSummaries` (a09da39)
- [x] An abort that rises inside the cap lookahead is reported as an abort, restoring the header's contract, and the header records the extra poll — `src/camp_map/vector/vector_parse.cpp`, `vector_parse.h:63-70`; test `AnAbortAtAnyPollIsReportedAsAborted` sweeps EVERY poll position rather than a guessed one (fails without the fix at the lookahead's poll) (bcff67d)
- [x] The hover label sets `setAcceptedMouseButtons(Qt::NoButton)`, so the ADR-0016 D5 / camp#225 guarantee holds for the whole subtree — `src/camp_map/vector/vector_feature_item.cpp:337-380`; test `TheHoverLabelAcceptsNoMouseButtonEither` (fails without the fix), plus `APressOverTheHoverLabelFallsThroughToTheView` through a real view. The view-level test is the end-to-end statement, NOT the regression detector — a flagless `QGraphicsSimpleTextItem` ignores a press it accepts the button for, so the scene ends with no grabber either way; that scope limit is written into the test's comment rather than left implied (fe95514)
- [x] The cap is documented and reported in ITEMS (one per emitted geometry part), so the header, the status line, the log and ADR-0016 D11 agree and no longer state a feature count the file does not have — `src/camp_map/vector/vector_layer.h:65-90`, `vector_layer.cpp:237-290`, `docs/decisions/0016-read-only-vector-file-layer.md`; existing status assertions updated (76c2f4c)

### Notes for the re-review
- The plan is in sync: **rev 16** records all seven fixes and the tests added (607a6a9).
- The one judgement call worth a second opinion: the "nit" was taken as a WORDING fix (items) rather than making the cap count source features, because counting features would change what the cap bounds — the cap exists to bound GUI-thread items and worker memory, which are per-part quantities.
- Nothing was pushed; the host performs pushes.

### Next step
review-code (re-review the fixes) via `.agent/scripts/dispatch_subagent.sh --mode in-process --issue 22 --skill review-code`

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-17 08:45 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-22 at `7f486f0`
**Mode**: pre-push
**Depth**: Deep (reason: cross-layer camp <-> camp_map, GDAL input handling, worker-thread lifecycle) — horizon scoped to the 9 commits since `3cedc7b`
**Must-fix**: 1 | **Suggestions**: 8
**Round**: 8 | **Ship**: recommended — the one must-fix is a precise, mechanical correction (a diagnostic counter the new geometry skip has to increment, folded into the Layers-tab status) with no design question open; the count fell from 2 to 1 and every one of the seven round-5 findings is verified closed against the code rather than the claim.

Specialists: Static Analysis (cppcheck 2.13 clean on touched lines — only the known Qt
`slots` unknownMacro noise and a pre-existing style note on an untouched line; repo
pre-commit clean over `3cedc7b..HEAD`), Governance, Plan Drift, Claude Adversarial
Lens A + Lens B. Copilot and local review off (default). Build and tests re-run here:
`./ui_ws/build.sh camp` clean, `./ui_ws/test.sh camp` → **410 tests, 0 errors,
0 failures, 1 skipped**, matching the Implementation entry's claim. All 9 commits carry
the agent identity; plan rev 16 is in sync; working tree clean.

All 7 findings of the `## Integrated Review` of 2026-09-17 08:17 -04:00 are CLOSED.
The two the implementer flagged for judgement were both checked directly and both
calls are UPHELD: (a) the cap nit as wording is right — `max_geometries` is spent per
emitted geometry PART and one `VectorFeatureItem` is built per part, so counting
source features would unbind the very quantities the cap exists to bound; (b) the
view-level `APressOverTheHoverLabelFallsThroughToTheView` really cannot fail without
the fix, verified by MUTATION (the `setAcceptedMouseButtons` line commented out and
rebuilt): the item-level `TheHoverLabelAcceptsNoMouseButtonEither` fails
(`acceptedMouseButtons() == 31`) while the view-level test still passes. Keeping both
with that scope written into the comment is the right call; the end-to-end test guards
against a future label that does grab the press.

The must-fix is a CONSEQUENCE of fix 3, not a defect in it, and was raised
independently by both adversarial lenses and the lead.

### Findings
- [x] (must-fix) The all-dropped-exterior skip removed the only OPERATOR-VISIBLE report of that class: the geometry no longer reaches `VectorLayer`, so `skipped` stays 0 and the Layers tab prints the bare `(no features)` — indistinguishable from an empty file — for a file whose features exist but fall outside their projection's inverse domain; a genuinely empty exterior ring is now counted by nothing at all (`points_dropped` has no vertex to count, `polygons_without_exterior_ring` sees a ring). Give the two skips their own diagnostic that `loadFinished()` folds into `skipped`/the status — `src/camp_map/vector/vector_parse.cpp:223`, `:253`, `src/camp_map/vector/vector_layer.cpp:274-277`
- [x] (suggestion) The cap path sets `diag.aborted` and then calls `reportLayerDiagnostics()` two lines later, which the lambda's own comment says the abort path deliberately must not do — closing a layer can now log per-layer "what was left out" warnings for a result the caller discards whole; guard with `if(!diag.aborted)` — `src/camp_map/vector/vector_parse.cpp:650-652`, `:534-565`
- [x] (suggestion) The new re-check makes `aborted` and `geometry_cap_reached` both true on a TRIMMED result, the one combination `ParseDiagnostics` says cannot happen ("a parse that hits BOTH … reports `aborted`, not this … also not trimmed"); clear the cap flag when the re-check fires (or correct the header) and assert `EXPECT_FALSE(geometry_cap_reached)` in the sweep test — `src/camp_map/vector/vector_parse.cpp:637-652`, `src/camp_map/vector/vector_parse.h:116-120`, `test/test_vector_parse_attributes.cpp` (`AnAbortAtAnyPollIsReportedAsAborted`)
- [x] (suggestion) Residual "feature" spellings of the quantity commit 76c2f4c renamed to items, in the same function and the same ADR the commit claims now agree: the skipped-count log line, the empty-layer status, and two ADR sentences — `src/camp_map/vector/vector_layer.cpp:237`, `:277`, `docs/decisions/0016-read-only-vector-file-layer.md:375`, `:479`
- [x] (suggestion) `AllDroppedExteriorIsNeitherEmittedNorCharged`'s hole is INSIDE the orthographic domain, so its vertices add nothing to `points_dropped` whether or not they are read — the "the hole is never read" property the early return exists for is untested; put the hole out of domain and assert 11 vs 7 — `test/test_vector_parse_attributes.cpp:1442-1443`
- [x] (suggestion) The promotion gate `purged.size() != m_unavailableVectorLayerFiles.size()` is sound only because `withoutVectorLayerFile()` does not de-duplicate, unlike both its siblings; compare contents or state the dependency at the call site — `src/camp/autonomousvehicleproject.cpp:458`
- [x] (suggestion) `withVectorLayerFilePromoted()` stats EVERY non-exact-matching entry of the whole restored order on the GUI thread, at the moment a share has just come back and the others are most likely dead mounts; the comment cites `withoutVectorLayerFile()`'s smaller per-unavailable-entry cost. Short-circuit after the single rewrite, or correct the comment — `src/camp/autonomousvehicleproject.cpp:455-460`, `src/camp_map/vector/vector_layer.cpp:608-621`
- [x] (suggestion) With the skip, a file whose geometries ALL lose their exterior never reaches the cap and is read to its last feature (memory stays bounded, wall time does not) — the cap's "the rest of the file is never read" no longer holds for that file. Bound it or record the deliberate change — `src/camp_map/vector/vector_parse.cpp:223`, `:253`, `src/camp_map/vector/vector_layer.h:73-90`
- [x] (suggestion) Doc-impact of fix 6: ADR-0016 D5 and `.agents/README.md` still state the no-mouse-button guarantee as a property of the ITEM ("Do not give the item a mouse handler without re-reading ADR-0016 D5"), which is exactly the reading that let the child label keep the default; one clause each to say it holds for the whole subtree — `docs/decisions/0016-read-only-vector-file-layer.md:155-158`, `.agents/README.md:119-124`

### Next step
address-findings (1 must-fix + 8 suggestions) via `.agent/scripts/dispatch_subagent.sh --mode in-process --issue 22 --skill address-findings`

## Implementation
**Status**: complete
**When**: 2026-09-17 09:05 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-22 at `94faee8`
**Addressed**: the `## Local Review (Pre-Push)` of 2026-09-17 08:45 -04:00 (round 8, at `7f486f0`) — 1 must-fix + 8 suggestions, all actioned, none deferred
**Commits**: b164ba7, df407fc, c016364, 6d89249, e45f604, 1a1713f, 63d7ce1, 4c91b24, 94faee8 (+ this progress/plan commit)

Build `./ui_ws/build.sh camp` clean; `./ui_ws/test.sh camp` → **411 tests, 0 errors,
0 failures, 1 skipped** (410 before: one new layer-level regression test). Plan bumped
to rev 17 with the round-8 bullet list. Pre-commit hooks ran on every commit; nothing
pushed.

### Actions
- [x] (must-fix) The all-dropped-exterior skip now has its own diagnostic —
  `ParseDiagnostics::geometries_with_empty_exterior`, incremented at both skips,
  logged per layer beside its siblings, and folded by `loadFinished()` (together
  with `polygons_without_exterior_ring`, which had the identical gap) into
  `skipped`, so the Layers tab no longer answers "(no items)" for an out-of-domain
  file. Regression test `VectorLayerTeardown.GeometriesDroppedByTheParseAreReportedInTheStatus`
  (2 empty line strings + 1 ringless polygon → status is not the empty-file verdict
  and counts 3), plus the counter asserted on the parse fixture —
  `src/camp_map/vector/vector_parse.h:147-163`, `vector_parse.cpp:223`, `:253`,
  `vector_layer.cpp:245-262`, `test/test_vector_layer_teardown.cpp`
- [x] (suggestion) The cap path's `reportLayerDiagnostics()` is guarded by
  `if(!diag.aborted)`, so the abort window no longer logs per-layer summaries for a
  result the caller discards whole — `src/camp_map/vector/vector_parse.cpp:673-681`
- [x] (suggestion) `geometry_cap_reached` is cleared when the re-check raises the
  abort, restoring the documented mutual exclusion; the header now says how much an
  aborted result holds is unspecified (partial from a work-loop abort, exactly
  `max_geometries` from this late window) because the caller discards it, and the
  poll sweep asserts `EXPECT_FALSE(geometry_cap_reached)` at every rise position —
  `vector_parse.cpp:663-672`, `vector_parse.h:101-124`, `test_vector_parse_attributes.cpp`
- [x] (suggestion) The four residual "feature" unit spellings are now items: the
  skipped-count log line, the `(no items)` status (and its three comment/doc
  references), and ADR-0016's D13 and consequence sentences —
  `vector_layer.cpp:237`, `:291`, `vector_layer.h:167`, `docs/decisions/0016-…:375`, `:480`
- [x] (suggestion) The undrawable polygon's hole is out of domain, so 7 dropped
  points (rather than 11) is what proves the interior rings are never read —
  `test/test_vector_parse_attributes.cpp:1405-1450`
- [x] (suggestion) The promotion gate compares the purged list itself
  (`purged != m_unavailableVectorLayerFiles`), not its length, so it no longer
  depends on `withoutVectorLayerFile()` not de-duplicating —
  `src/camp/autonomousvehicleproject.cpp:458-470`
- [x] (suggestion) `withVectorLayerFilePromoted()` stops resolving once it has
  rewritten the entry (`!promoted &&` on the canonicalising half of the identity
  test), bounding the GUI-thread stats at the promoted slot instead of the whole
  restored order; the exact-string duplicate collapse is unchanged and tested, the
  surviving second raw spelling is dropped by `rebuildPersistedVectorLayerFiles()`,
  and the cost is stated for this function on its own declaration and at the call
  site — `vector_layer.cpp:608-640`, `vector_layer.h:333-346`, `autonomousvehicleproject.cpp:441-447`
- [x] (suggestion) The cap's changed reach is recorded: a file whose geometries all
  drop out is read to its end (memory unaffected, still abortable, drop count now in
  the status), with the rejected alternative named —
  `vector_parse.h:93-112`, `vector_layer.h:83-91`, `docs/decisions/0016-…` consequences
- [x] (suggestion) ADR-0016 D5 and `.agents/README.md` state the no-mouse-button
  guarantee for the item AND every child it puts in the scene —
  `docs/decisions/0016-…:155-166`, `.agents/README.md:119-127`

### Next step
review-code (re-review of the round-8 fixes) via
`.agent/scripts/dispatch_subagent.sh --mode in-process --issue 22 --skill review-code`

## Integrated Review
**Status**: complete
**When**: 2026-09-17 09:37 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #226 at `6139f33`
**Sources**: 2 (Copilot R7 @ `6139f33` — 3 inline + 5 suppressed, submitted 2026-09-17 13:29:14 UTC; CI rollup @ `6139f33`)
**Cross-source confirmations**: 0 (the local sources at this head — `## Local Review (Pre-Push)` round 8 @ `7f486f0` and the `## Implementation` of 09:05 that closed all 9 of its findings — raise none of the below; every round-8 finding was re-read against the code at this head and none regressed)
**CI**: all-pass (`build-and-test` SUCCESS, `copilot-pull-request-reviewer` SUCCESS, both on `6139f33`)

Round 6 of triage. Only the Copilot review of 2026-09-17 13:29:14 UTC is triaged
here; the six earlier reviews are the rounds already recorded as `## Integrated
Review` entries (2026-09-14 14:58, 2026-09-15 09:31, 13:07, 14:51, 2026-09-17
08:17) and every finding they raised is checked closed. This is the first round
whose findings are all NEW ground: nothing Copilot raises here repeats a finding
of rounds 1-5, and it raises nothing against a settled operator decision except
the unbounded-ring item, which is the fourth raising of the ADR-0016 D11
deferral and stays deferred.

Two comments (the inline at `vector_layer.cpp:262` and the suppressed one at
`vector_parse.cpp:197`) are the two ends of one defect and are recorded once.
Notably, that defect is the branch the round-8 must-fix did NOT cover: standalone
points are the one geometry class whose parse-side drop still does not reach the
Layers-tab status.

### Findings
- [x] (should-fix, Copilot R7) **Startup restore persists once per layer, so an exit or crash mid-restore ERASES every entry the loop has not reached yet.** `restorePersistedVectorLayers()` builds `m_restoredVectorLayerOrder` and `m_unavailableVectorLayerFiles` incrementally and calls `openVectorLayer()` inside the loop; `openVectorLayer()` ends with `persistVectorLayers()`, which rewrites `vectorLayers/files` from `rebuildPersistedVectorLayerFiles(m_restoredVectorLayerOrder, m_unavailableVectorLayerFiles, loaded)` — three lists that, at iteration k, know nothing about entries k+1..n. Each write is a fresh `QSettings` whose destructor syncs, so the TRUNCATED list reaches disk immediately, once per restored layer. The window is not theoretical: `openVectorLayer()` starts an asynchronous GDAL parse on a worker thread and returns, so parses of the earlier layers are running while the loop is still opening later ones, and a worker-side abort (a malformed file, an out-of-memory) takes the process down with the key truncated. The consequence is exactly the camp#90/#117 class this PR exists to fix — silently forgotten layers — reached from the other direction. Fix: populate the full order/unavailable state first (or gate per-layer persistence behind a `restoring_` flag) and persist ONCE after the loop, which is already the last statement of the function. Regression test: a harness-visible seam on the rebuild rule (the rule already lives in `camp::vector::rebuildPersistedVectorLayerFiles`, which IS testable) asserting that a rebuild from a partially populated order never drops an entry the input list carried — `src/camp/autonomousvehicleproject.cpp:485-492`, `:528-562`, `src/camp_map/vector/vector_layer.cpp:656-676`
- [x] (should-fix, Copilot R7 — inline `vector_layer.cpp:262` + suppressed `vector_parse.cpp:197`, one defect at both ends) **A standalone Point whose transform fails is dropped by the parse and counted by NOTHING the Layers tab reads, so a file of such points reports the empty-file verdict.** The `wkbPoint` branch increments `diagnostics.points_dropped` and emits no geometry (`vector_parse.cpp:190-197`), which is right — but `loadFinished()` folds only `geometries_with_empty_exterior + polygons_without_exterior_ring` into `skipped` (`vector_layer.cpp:254-262`), and it cannot fold `points_dropped` because that counter also counts individual bad VERTICES of lines and polygons that were drawn fine. A point layer whose features all fall outside their projection's inverse domain therefore produces zero features, `skipped == 0`, and the status `(no items)` — an EMPTY-FILE verdict with a completely different remedy. This is the identical failure the round-8 must-fix corrected for lines and polygons; the point branch is the one case it did not reach, and Copilot's warning not to just add `points_dropped` is correct. Fix: a `ParseDiagnostics` counter of its own (e.g. `point_geometries_dropped`), incremented at that break, logged per layer beside its siblings in `reportLayerDiagnostics()`, and added to `skipped` in `loadFinished()`. Test alongside `VectorLayerTeardown.GeometriesDroppedByTheParseAreReportedInTheStatus` with out-of-domain points — `src/camp_map/vector/vector_parse.cpp:182-203`, `src/camp_map/vector/vector_parse.h:147-163`, `src/camp_map/vector/vector_layer.cpp:245-262`
- [x] (should-fix, Copilot R7) **`future_watcher_.result()` copies the whole parse result, and the watcher keeps its own copy for the layer's lifetime.** Qt 5.15's `QFutureWatcher<T>::result()` is `T result() const { return m_future.result(); }` and `QFuture<T>::result()` is `inline T result() const` — both return BY VALUE (`/usr/include/x86_64-linux-gnu/qt5/QtCore/qfuturewatcher.h:127`, `qfuture.h:97`), so `const LoadResult result = future_watcher_.result();` materialises a second full `std::vector<ParsedLayer>` — every coordinate and every per-part `QMap<QString,QVariant>` attribute copy — while the future's result store still holds the first. Peak memory during `loadFinished()` is therefore doubled at the very moment the GUI thread is also building up to 50 000 `VectorFeatureItem`s, and because `future_watcher_` is a member that is never reset, the future's copy is RETAINED for the layer's whole lifetime even though the items hold their own attributes. That directly undercuts the memory bound `kMaxFeatureItems` is documented to provide. Qt 5 has no `takeResult()`, but the copy is avoidable today: `const LoadResult& result = *future_watcher_.future().constBegin();` yields `const T&` (`qfuture.h:119` → `resultReference(index)`), and clearing the future at the end of the slot (`future_watcher_.setFuture(QFuture<LoadResult>());`, after the last use of `result`) releases the retained copy — an empty future leaves the destructor's `waitForFinished()` a no-op. Alternatively make the worker's payload a `std::shared_ptr<LoadResult>` — `src/camp_map/vector/vector_layer.cpp:190-193`, `:130-142`, `src/camp_map/vector/vector_layer.h:224`
- [x] (should-fix, Copilot R7) **The persisted colormap name is restored verbatim — no case fold, no registry check — unlike every sibling layer.** `readSettings()` stores whatever string is in QSettings into `colormap_` (`vector_layer.cpp:544`). `resolvePalette()` renders an unknown name as grayscale (`:33-39`), so nothing looks broken, but `colormap_` keeps the invalid spelling: the Colormap menu checks `name == colormap_` against registry names (`:527-533`), so NO palette shows as checked, the operator cannot see which ramp is in force, and `writeSettings()` writes the bad value back every session. `marine_colormap::find_palette()`/`palette_index()` are documented as name lookups with no case handling, and `RasterLayer::readSettings()` (camp#141) does exactly what is missing here — `.toLower()`, then `if(!marine_colormap::palette_index(colormap)) colormap = "grayscale";` (`src/camp_map/raster/raster_layer.cpp:681-686`). Reachable without hand-editing settings: a palette renamed or removed from the registry between builds leaves every layer styled with it in this state. Fix: mirror the raster precedent, falling back to `grayscale` so what is drawn and what is checked agree; cover it in the existing `StyleRoundTripsThroughSettings` test — `src/camp_map/vector/vector_layer.cpp:536-551`, `:33-39`
- [x] (suggestion, Copilot R7) **The Size by menu offers numeric fields that only line/polygon features carry, where selecting one silently does nothing.** `numericFields()` folds over ALL features (`vector_layer.cpp:340-351`) and feeds both submenus (`:519-520`), while `applyStyle()` accumulates `size_range` over `feature->isPoint()` only — deliberately, so unsized geometry cannot set the marker extent (`:394-403`). Pick a line-only field under Size by and `size_range` is invalid, `radiusForValue()` returns the default radius for every marker, the action shows as checked, and the choice is persisted: a menu entry whose only effect is to look selected. This PR already rejects that pattern elsewhere — the stale persisted field is shown as `<field> (no numbers)` precisely so a setting that does nothing says so. Fix: build the Size by list from point-carried numeric fields (and annotate a set-but-unusable one the same way), leaving Color by on the full list — `src/camp_map/vector/vector_layer.cpp:340-351`, `:385-403`, `:492-520`
- [x] (suggestion, Copilot R7) **A geometry with no PLACEABLE coordinate is still emitted and still spends a cap slot, which is the one case the round-5/8 "do not charge undrawable geometry" rule does not cover.** A layer with no spatial reference at all takes `toWgs84()`'s untransformed branch (`vector_parse.cpp:48-60`), so projected metres become an out-of-range `QGeoCoordinate`: no vertex FAILED, the exterior is not empty, the geometry is emitted and `budget.spend()` is called — and `VectorLayer` then rejects it on `hasPlaceableCoordinate()`, which is just `QGeoCoordinate::isValid()` (`vector_feature_item.cpp:92-112`). On a single-layer file that costs nothing real (everything in it is unplaceable), but a multi-layer container in which an SRS-less layer precedes a good one can exhaust the cap on shapes that draw nothing and never read the valid layer. The parser can apply the same test it already applies to an empty exterior — if no vertex of the exterior is placeable, drop and count instead of emitting and charging — which keeps memory bounded and the wall-time consequence already accepted for `geometries_with_empty_exterior`. NOT accepted from Copilot: the "user-visible contract" half of its comment, dismissed below. Second, smaller half, worth fixing either way: the cap `qWarning()` still says "what is shown is the first 50000 drawn items and no more", which overstates whenever some emitted items were unplaceable — the status line right below it is already honest (it prints the real item count and the unplaceable count) — `src/camp_map/vector/vector_parse.cpp:182-276`, `src/camp_map/vector/vector_parse.h:83-112`, `src/camp_map/vector/vector_layer.cpp:239-244`
- [x] (nit, Copilot R7) **`kMaxFeatureItems`' header claim that the cap bounds parse memory carries no pointer to the limit ADR-0016 D11 records.** The substance — one ring of millions of vertices is not bounded — is the standing D11 deferral and stays deferred (see the dismissal below); what makes Copilot raise it a fourth time is that the claim reads as unqualified where a reader meets it. D11 states the exception in terms ("Neither the vertex count of a single ring (100 M vertices is ~1.6 GB of coordinates) nor the size of a single attribute value is bounded", `docs/decisions/0016-read-only-vector-file-layer.md:301-309`); one clause in the header saying so and naming D11 ends the rediscovery without pre-empting the operator's decision — `src/camp_map/vector/vector_layer.h:65-80`

### False positives
- (Copilot R7, `src/camp_map/vector/vector_layer.cpp:224`, the second half of its comment) "Change the documented/user-visible contract to say it caps parsed parts rather than drawn items" — already done, in rounds 5 and 8, and re-verified at this head: `kMaxFeatureItems`' header says "one per emitted geometry part", ADR-0016 says "The status, the log and `kMaxFeatureItems` all count drawn items — one per emitted geometry part", and the status line reports the ACTUAL item count with the unplaceable count beside it (`(N items, stopped at the 50000-item cap; rest of file not read, M unplaceable)`), not an assumed N. The operator is not told a number the file does not have. What survives of the comment is the cap-starvation mechanism, carried above as a suggestion, not this.
- (Copilot R7, `src/camp_map/vector/vector_layer.h:74`, the substantive half) "`readRing()` has already materialized every vertex of the current line or polygon before `appendGeometry()` spends a cap slot; a single huge feature can still exhaust memory" — correct, and the FOURTH raising of the item dismissed in rounds 2, 3 and 5. It is not a defect to fix in this PR: silently truncating a ring DRAWS A WRONG SHAPE, which is worse than the honest geometry cap, so the two options (drop-and-count, or accept-and-say-so) are folded into the deferred ADR-0016 D11 operator decision, which states the unbounded vertex count in terms. The abort-latency half was fixed in round 2 (`kVertexPollInterval`, `vector_parse.cpp:122`, `:143-148`). Only the doc-pointer nit above is taken from it.

### Previously triaged, unchanged
- The two standing dismissals from rounds 1-3 — the `destroyed`-slot use-after-free (measured on this host's Qt) and the cosmetic-pen bounding-rect margin — were not raised again in R7.
- All 9 findings of the `## Local Review (Pre-Push)` round 8 @ `7f486f0` are checked closed by the `## Implementation` of 09:05 and were re-read at their cited lines here: each is present in the code at `6139f33` and none regressed. The round-8 upheld decision that the cap counts emitted parts by design stands; finding 6 above does not reopen it (it changes what is CHARGED, not what is counted).

### Next step
address-findings (7 open: 4 should-fix, 2 suggestions, 1 nit; 0 must-fix, 0 cross-confirmed) via `.agent/scripts/dispatch_subagent.sh --mode in-process --issue 22 --skill address-findings`

## Implementation
**Status**: complete
**When**: 2026-09-17 10:12 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #226 at `384010b` (branch `feature/issue-22`; fixes not yet pushed)
**Addressed**: the `## Integrated Review` of 2026-09-17 09:37 -04:00 (round 6, Copilot R7 @ `6139f33`) — all 7 open findings
**Commits**: `c366e89`, `4c67110`, `56b04cf`, `6a954ea`, `8f3d527`, `cb2e955`, `8d9f7a2`, `54ae83d` (+ this entry)

### Actions
- [x] Startup restore builds the whole state first and persists ONCE — `camp::vector::planVectorLayerRestore()` classifies the entire persisted list (order / unavailable / openable) without opening or writing anything, and `persistVectorLayers()` is suppressed for the duration of the open loop via `m_restoringVectorLayers` — `src/camp_map/vector/vector_layer.cpp:656-694`, `src/camp/autonomousvehicleproject.cpp:495-580`, `src/camp/autonomousvehicleproject.h:252-261`. Test: `VectorLayerPersistence.RestorePlanIsCompleteBeforeAnyFileIsOpened` (the plan is complete for every entry before the first open, planning writes nothing, and the mid-loop write it replaces is asserted lossy).
- [x] A standalone point dropped by the parse reaches the status — new `ParseDiagnostics::point_geometries_dropped`, logged per layer beside its siblings and folded into `skipped` — `src/camp_map/vector/vector_parse.h`, `vector_parse.cpp` (wkbPoint branch + `reportLayerDiagnostics()`), `vector_layer.cpp` (`loadFinished()`). Test: `VectorLayerTeardown.DroppedStandalonePointsAreReportedInTheStatus`, over a GPKG in an orthographic SRS whose points lie outside the projection's inverse domain (a transform failure the harness can rely on) — the layer reports 3 skipped instead of "(no items)".
- [x] No second copy of `LoadResult` — `loadFinished()` reads `*future.constBegin()` (`const T&`) off a named local future and clears `future_watcher_`'s future on every exit path, releasing the store; a `load_reported_` guard covers the re-entrant `finished()` that clearing can deliver — `src/camp_map/vector/vector_layer.cpp:190-233`, `vector_layer.h`. No new test: the property is memory, not behaviour; the existing teardown/status/cap tests all drive this slot.
- [x] Persisted colormap validated and case-folded against the registry, falling back to grayscale — the `RasterLayer::readSettings()` rule — `src/camp_map/vector/vector_layer.cpp` (`readSettings()`). Test: `VectorLayerPersistence.StyleRoundTripsThroughSettings` extended with a bogus name (-> grayscale) and a miscased one (-> registry spelling). Note the fixture had to change from `"plasma"` to `"turbo"`: `plasma` is NOT in the marine_colormap registry, so the old round-trip was asserting that an invalid name survives verbatim — the very defect this finding names.
- [x] Size by lists point-bearing numeric fields only — `pointNumericFields()` (both lists fold through `numericFieldsOf(bool)`), the two submenus are built from their own lists, and a set-but-unusable size field is annotated `(no numbers on any point)` — `src/camp_map/vector/vector_layer.cpp` (`numericFieldsOf`/`pointNumericFields`/`contextMenu`), `vector_layer.h`. Test: `VectorLayerStyleFields.SizeByOffersOnlyFieldsThePointsCarry`; `AStaleStyleFieldIsShownInTheMenuAndCanBeCleared` updated for the new note.
- [x] The cap is not charged for geometry with no placeable coordinate — `isPlaceable()` / `firstPlaceableCoordinate()` / `hasPlaceableCoordinate()` moved beside the parse (one rule, two callers; `vector_feature_item` now uses them), all three geometry branches drop-and-count into the new `ParseDiagnostics::geometries_without_placeable_vertex`, which `loadFinished()` folds into `skipped`; the cap `qWarning()` no longer claims "the first 50000 drawn items" — `src/camp_map/vector/vector_parse.{h,cpp}`, `vector_feature_item.{h,cpp}`, `vector_layer.cpp`. Test: `VectorLayerTeardown.CapIsNotSpentOnGeometryThatCannotBePlaced`, which replaces `CappedButEmptyLayerStillReportsTheUnreadRemainder` — see the consequence below.
- [x] `kMaxFeatureItems` names what it does not bound and points at ADR-0016 D11 — `src/camp_map/vector/vector_layer.h`. The D11 deferral itself stands (it was dismissed as a false positive in the review).

### Consequence worth a reviewer's eye
Extending "do not charge the cap for undrawable geometry" to the UNPLACEABLE class
makes the round-4 combination — "the cap was hit AND every capped geometry was
unplaceable" — unreachable through this parser: a `.prj`-less file is now read to
its last feature instead of stopping at the cap, and is reported as read in full
(every dropped geometry counted, no "rest of file not read", which would now be
the false statement). Memory is unaffected (nothing dropped is materialised) and
the parse stays abortable at the same granularity; the residue is wall time on a
file that shows nothing. The round-4 test was rewritten to assert the new truth
rather than deleted, and the trade is recorded in ADR-0016's consequences,
`ParseOptions::max_geometries` and plan rev 18. The capped-and-partial status path
is still covered by `FeatureCapBoundsGuiThreadWorkAndIsReported` over a file with
drawable features.

### Verification
`./ui_ws/build.sh camp` clean; `./ui_ws/test.sh camp` — **414 tests, 0 failures,
1 skipped** (was 411 + the 3 new regression tests; one test renamed/rewritten).
Pre-commit hooks ran on every commit; nothing pushed (the host pushes).

### Next step
review-code (re-review the fixes) via `.agent/scripts/dispatch_subagent.sh --mode in-process --issue 22 --skill review-code`

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-17 10:25 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-22 at `08c5832`
**Mode**: pre-push
**Depth**: Deep (reason: cross-layer camp <-> camp_map, worker-thread lifecycle, GDAL input handling) — horizon scoped to the 10 commits since `6139f33`
**Must-fix**: 1 | **Suggestions**: 11
**Round**: 9 | **Ship**: recommended — the single must-fix is a precise, mechanical correction (one `setStatus`/`qWarning` on a newly added early-return path) with no design question open; the count is 1 against round 8's 1, not rising, and all 7 findings of the 09:37 integrated review are verified closed against the code rather than the claim.

Specialists: Static Analysis (cppcheck 2.13 clean on touched lines — one `useStlAlgorithm`
style note on a MOVED line, dropped; repo pre-commit clean over `6139f33..HEAD`),
Governance, Plan Drift, Claude Adversarial Lens A + Lens B. Copilot and local review off.
Build and tests re-run here: `./ui_ws/build.sh camp` clean, `./ui_ws/test.sh camp` →
**414 tests, 0 errors, 0 failures, 1 skipped**, matching the Implementation entry's claim.
All 10 commits carry the agent identity; plan rev 18 is in sync; working tree clean.

All 7 findings of the `## Integrated Review` of 2026-09-17 09:37 -04:00 are CLOSED, and no
regression was found in any of them. The four points the implementer flagged were each
checked directly and all four calls are UPHELD. (a) The colormap fixture move is right and
the old test really was asserting the defect — `marine_colormap/test/test_palette.cpp:56`
asserts `palette_index("plasma")` has no value; all 8 registry names are lowercase so
`.toLower()` cannot destroy a valid choice, `palette_index()` returns `std::optional` so the
`if(!...)` tests emptiness rather than index 0, and grayscale (index 0) survives. (b) The
unplaceable-geometry trade is recorded in ADR-0016 and `ParseOptions::max_geometries`, and
the rewritten test is non-vacuous (4 unplaceable points against a cap of 2, all 4 counted).
(c) The `LoadResult` lifetime reasoning is CORRECT, verified independently by both
adversarial passes against the installed Qt 5.15 headers: declaration order is load-bearing
and right (`future` before `release_future`, so the clear runs while the local still holds a
ref), the named local is necessary because `const_iterator` stores the `QFuture*`, the
`isResultReadyAt(0)` guard is not decoration (`advanceIndex` returns -1 on an empty store),
and the destructor's join is a correct no-op afterwards — but this fix introduced the one
must-fix below. (d) The restore rework is behaviourally identical to the old inline loop
line by line, the in-loop promotion branch is provably inert during restore, and none of
`persistVectorLayers()`'s three callers is wrongly suppressed.

### Findings
- [x] (must-fix) The no-result early return reports NOTHING — status stays "(loading...)" forever with no log line; reachable when the worker THROWS (`reportException` sets Canceled, `reportResult` then bails), i.e. the `bad_alloc` the cap exists to bound. Previously this rethrew and terminated — loud; it is now silent, which is the class this round exists to remove. Add `setStatus("(load failed)")` + a `qWarning()` (guard a genuine abort with `!isAborted()`) and correct the comment, which names cancellation rather than the exception case. Cross-pass confirmed (Lens A + Lens B), reachability verified against `qtconcurrentrunbase.h:104-126` and `qfutureinterface.h:192-198` — `src/camp_map/vector/vector_layer.cpp:229-234`
- [x] (suggestion) `geometries_unhandled` is the one drop class still not folded into the status, so a curve-only file reports the bare "(no items)" empty-FILE verdict — verbatim the defect rounds 8 and 9 each closed once; the counter already exists and is already logged per layer, and wants its own note rather than a sum into "unplaceable". Cross-pass confirmed — `src/camp_map/vector/vector_layer.cpp:340-343`, `src/camp_map/vector/vector_parse.cpp:374`
- [x] (suggestion) A feature whose `GetGeometryRef()` is null is dropped with NO counter of any kind — the one geometry class genuinely lost silently; reachable because CSV/GML/DXF are on `kAllowedDrivers` and a CSV with no usable X/Y fields is exactly this. Same empty-file verdict; needs a new counter, distinguished from the `budget.exhausted()` half of the condition — `src/camp_map/vector/vector_parse.cpp:178`
- [x] (suggestion) Nothing prevents a re-entrant restore and `RestoreScope` FAILS OPEN if one occurs (an inner destructor clears the flag while the outer loop runs). Not reachable today, but that rests on unasserted properties of `openVectorLayer()` two files away; a one-line `if(m_restoringVectorLayers) return;` closes it — `src/camp/autonomousvehicleproject.cpp:528`
- [x] (suggestion) The `if(!layers) return;` exit leaves an entry in `plan.order` but in neither the unavailable nor the loaded list, so the single trailing persist silently deletes it — the one way a COMPLETED restore loses an entry. Pre-existing, but the new comments claim completeness — `src/camp/autonomousvehicleproject.cpp:419-420`
- [x] (suggestion) The re-entrancy comment asserts as fact that `setFuture()` with an empty future can deliver `finished()` again; Lens B's reading of Qt 5.15 is that `sendCallOutEvent()` drops every callout but `Canceled` on a canceled future. Not settled (`qfuturewatcher.cpp` is not installed here), so soften the wording per "never document from assumptions" — and KEEP `load_reported_` either way — `src/camp_map/vector/vector_layer.cpp:210-215`
- [x] (suggestion) The reworded cap `qWarning()` explains a discrepancy this same round made unreachable (nothing unplaceable is emitted any more); the surprise that CAN still occur is the round-5 one, that the cap is spent per emitted part — `src/camp_map/vector/vector_layer.cpp:288-301`
- [x] (suggestion) No test covers the round-9 concurrency change (future released, slot idempotent, teardown still joins after a completed load) — the change with the least visible failure mode; and the new placeability drop is exercised on the POINT branch only, leaving the LineString and Polygon branches (including the polygon's deliberate placement before the interior-ring loop) uncovered — `test/test_vector_layer_teardown.cpp`
- [x] (suggestion) `QSettings::value(key, default)` returns the default only when the key is ABSENT, so a present-but-empty `colormap` key lands on grayscale rather than the class default `viridis`; `raster_layer.cpp:692-694` handles the partial/corrupt case explicitly elsewhere — `src/camp_map/vector/vector_layer.cpp:670-671`
- [x] (suggestion) `palette_index()` is an exact CASE-SENSITIVE scan while the menu offers `palette_names()` verbatim, so a future non-lowercase registry name would reset that layer to grayscale on every launch. PRE-EXISTING and cross-file (`raster_layer`, `gggs_tile_layer`, `sonar_live_cache_layer`, `grid_map` all carry it) — file separately rather than growing this PR — `src/camp_map/vector/vector_layer.cpp:670-673` — **filed as https://github.com/rolker/camp/issues/235** (deferred: out of scope for this PR by the reviewer's own note; pre-existing and spanning four files this branch does not touch)
- [x] (suggestion) Three residues of the Size-by split: the `Size by` submenu is now just "(none)" on a layer whose points carry no numeric field (consider omitting it when list and setting are both empty); `size_fields.isEmpty()` in the early return is redundant (`size_fields` is a subset of `color_fields`); and `contextMenu()` now runs the `numericAttribute()` fold TWICE over the full feature set on every right-click — `src/camp_map/vector/vector_layer.cpp:572-593`
- [x] (suggestion) The `isPlaceable()` doc block claims the guard prevents `childrenBoundingRect()` poisoning, but `hasPlaceableCoordinate()` admits a geometry on ANY ONE placeable vertex, so a line with one good vertex and 999 UTM-northings-as-degrees still enters the bounding rect and the scene index. The design is defensible; the comment should state the bound it delivers — `src/camp_map/vector/vector_parse.h:213-224`

### Next step
address-findings (1 must-fix, 11 suggestions; 2 of the 12 cross-pass confirmed) via `.agent/scripts/dispatch_subagent.sh --mode in-process --issue 22 --skill address-findings`

## Implementation
**Status**: complete
**When**: 2026-09-17 10:52 -04:00
**By**: Claude Code Agent (Claude Opus 5 (1M context))

**Branch**: feature/issue-22 at `dc22673`
**Addressed**: `## Local Review (Pre-Push)` of 2026-09-17 10:25 -04:00 (round 9, branch at `08c5832`) — 1 must-fix, 11 suggestions
**Commits**: `ffded44`, `e3fe43c`, `20c0b0e`, `b0c4de7`, `1a4e3bb`, `865127f`, `ab56f80`, `51ae582`, `5d2c319`, `dc22673`

### Actions
- [x] (must-fix) The no-result early return reports "(load failed)" + a `qWarning()` unless the layer is being torn down, and the comment now names the real cause — the worker THROWING, which cancels the future without storing a result — rather than cancellation — `src/camp_map/vector/vector_layer.cpp:229-256` (`ffded44`). No test: the work function is a private, non-virtual member handed straight to `QtConcurrent::run()`, so nothing can substitute it, and exhausting the heap for real is neither deterministic nor bounded; the reasoning and what IS covered instead are written into `test/test_vector_layer_teardown.cpp` beside `UnopenableFileReportsLoadFailed`.
- [x] `geometries_unhandled` reaches the status with its own note (`(no drawable items; N of an unhandled geometry type (first: X))`), not a fold into the unplaceable count — different remedy — `src/camp_map/vector/vector_layer.cpp` (`e3fe43c`). Test: `UnhandledGeometryTypesAreReportedInTheStatus`, over a GPKG of CIRCULARSTRINGs (no hand-written GeoJSON can express one).
- [x] A null `GetGeometryRef()` is counted, in a new `ParseDiagnostics::features_without_geometry` split out of the shared `budget.exhausted()` return, and carries its own status note — `src/camp_map/vector/vector_parse.cpp:178`, `vector_parse.h`, `vector_layer.cpp` (`20c0b0e`). Test: `FeaturesWithNoGeometryAreReportedInTheStatus`, over a CSV with no coordinate columns.
- [x] `restorePersistedVectorLayers()` refuses re-entry at the door — `RestoreScope` fails OPEN, so the inner call's destructor would clear the flag under the outer loop — `src/camp/autonomousvehicleproject.cpp:528` (`b0c4de7`).
- [x] The `if(!layers)` exit now REMEMBERS the file (unavailable list) and warns, instead of leaving it in the order of record but in neither list, where the trailing persist deletes it — `src/camp/autonomousvehicleproject.cpp:419-420` (`1a4e3bb`).
- [x] The re-entrancy comment was rewritten twice: first softened per "never document from assumptions" (`865127f`), then replaced with the MEASURED answer (`dc22673`) — with the run-once flag removed, clearing the watcher's future re-delivers `finished()` at once and the re-entered slot reports a succeeded load as "(load failed)". The flag stays and is now covered — `src/camp_map/vector/vector_layer.cpp:210-215`.
- [x] The cap `qWarning()` names the surprise that can still occur (the cap is spent per emitted geometry PART) instead of the unplaceable discrepancy this round made unreachable — `src/camp_map/vector/vector_layer.cpp:288-301` (`865127f`).
- [x] Tests for the round-9 concurrency change and the uncovered placeability branches — `ASecondFinishedDeliveryChangesNothingAndTeardownStillJoins` (invokes the private slot through the meta-object; asserts the status VALUE, because an unguarded slot corrupts it before a before/after capture could see it — verified by mutation) and `LinesAndPolygonsThatCannotBePlacedAreDroppedAndCounted` (line + holed polygon, capped and uncapped) — `test/test_vector_layer_teardown.cpp` (`dc22673`).
- [x] An empty persisted `colormap` value falls back to the CLASS DEFAULT, not to the unknown-name grayscale: `QSettings::value(key, default)` defaults an absent key only — `src/camp_map/vector/vector_layer.cpp:670-671` (`ab56f80`). Test: added to the palette round trip in `test/test_vector_layer_persistence.cpp`.
- [x] (deferred: filed rather than fixed, per the reviewer's own out-of-scope note — pre-existing, and four of the five files are outside this branch) `palette_index()` is case-sensitive while five layers' `readSettings()` lowercase the persisted name — filed as https://github.com/rolker/camp/issues/235 ("Part of #22", both remedies described: one shared camp-side resolver, or a case-insensitive `palette_index()` in `marine_colormap`) — `src/camp_map/vector/vector_layer.cpp:670-673`.
- [x] The three Size-by residues: both numeric-field lists now come from ONE pass (`numericFieldLists()`, replacing two full folds per right-click), the redundant `size_fields.isEmpty()` test is gone (subset of the colour list), and "Size by" is omitted when it would hold nothing but "(none)" — unless a size field is set, the only place it can be cleared — `src/camp_map/vector/vector_layer.cpp:572-593`, `vector_layer.h` (`51ae582`). Test: `SizeByIsOmittedWhenItCanOfferNothing`.
- [x] The `isPlaceable()` doc block states the bound it actually delivers: any ONE placeable vertex admits the geometry, so a mostly-bad line still reaches the bounding rect; what is removed is the geometry no vertex of which is on the earth — the whole-file case. Why the weaker bound is deliberate, and where the NaN really dies (`toWgs84()` drops the vertex), are recorded — `src/camp_map/vector/vector_parse.h:213-224` (`5d2c319`).

### Verification
- `./ui_ws/build.sh camp` clean; `./ui_ws/test.sh camp` → **419 tests, 0 errors, 0 failures, 1 skipped**.
- Pre-commit hooks ran on every commit; no `--no-verify`.
- plan.md **Rev 19** records this pass (design unchanged).

### Next step
review-code (re-review the fixes) via `.agent/scripts/dispatch_subagent.sh --mode in-process --issue 22 --skill review-code`

## Integrated Review
**Status**: complete
**When**: 2026-09-17 11:29 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #226 at `a4d7fcd`
**Sources**: 2 (Copilot R8 @ `a4d7fcd` — 2 inline + 6 suppressed, submitted 2026-09-17 15:05:25 UTC; CI rollup @ `a4d7fcd`)
**Cross-source confirmations**: 0 at this head — the local sources (`## Local Review (Pre-Push)` round 9 @ `08c5832` and the `## Implementation` of 10:52 that closed its 12) sit one head back, and their 12 findings are all verified closed against the code at `a4d7fcd`, none regressed. See the lineage note on finding 1 below: it is the round-9 placeability finding read through a consumer the doc-only close did not cover, which is a confirmation in substance even though the head SHAs differ.
**CI**: all-pass (`build-and-test` SUCCESS, `copilot-pull-request-reviewer` SUCCESS, both on `a4d7fcd`)

Round 7 of triage. Only the Copilot review of 2026-09-17 15:05:25 UTC is triaged here;
the seven earlier reviews are the rounds already recorded as `## Integrated Review`
entries (2026-09-14 14:58, 2026-09-15 09:31, 13:07, 14:51, 2026-09-17 08:17, 09:37) and
every finding they raised is checked closed. Four of this round's seven items are new
ground and are taken as valid; two are the standing dismissals raised again (the
`destroyed`-slot use-after-free for the third time, the ADR-0016 D11 unbounded-ring
deferral for the fifth); one is a new claim about `setColormap()` that no live caller can
reach.

### Findings
- [x] (must-fix, Copilot R8) **Invalid coordinates reach the EDITABLE MISSION TREE through the shared parser.** The admission tests added in round 9 are `hasPlaceableCoordinate()` — any ONE placeable vertex admits the whole geometry (`vector_parse.cpp:240`, `:283-287`, `:327`; `vector_parse.h:269-289` states the weaker bound in terms). That bound was justified for the DISPLAY consumer, which filters again per vertex (`vector_feature_item.cpp:37`) — but the parser has a second production consumer, and it does not: `VectorDataset::buildItems()` copies EVERY emitted vertex straight into `Point`/`LineString`/`Polygon` mission items (`src/camp/vector/vectordataset.cpp:43-84`), which are draggable, persisted in the mission file and candidates for transmission to the robot (ADR-0016 Context, `docs/decisions/0016-read-only-vector-file-layer.md:38`). A `.prj`-less shapefile whose vertices are mostly UTM northings read as degrees therefore imports as a mission item at a coordinate that is nowhere on earth. Fix in the CONSUMER, not the parser: apply the same `isPlaceable()` per-vertex filter (and a count for the operator) in `buildItems()`, which is also what ADR-0016 D4 requires — "the parser is faithful; PLACEMENT policy belongs to the display layer" (`:86-96`), the principle Copilot's second option names. **Lineage**: the round-9 `## Local Review (Pre-Push)` @ `08c5832` raised the same weaker bound and was closed by DOCUMENTING it (`5d2c319`); documenting the bound did not protect the consumer that has no second filter — `src/camp/vector/vectordataset.cpp:43-84`, `src/camp_map/vector/vector_parse.cpp:283-287`
- [x] (must-fix, Copilot R8) **Two OGR native handles are freed only on the normal return path, and the throwing path is one this code explicitly handles.** (a) `readRing()` calls `OGRPointIterator::destroy(pi)` after the vertex loop (`vector_parse.cpp:139`, `:157`); `points.push_back()` inside that loop can throw `std::bad_alloc`, and the iterator leaks. (b) The per-feature loop calls `OGRFeature::DestroyFeature(feature)` after `appendGeometry()` returns (`:698`, `:712-714`, `:804`); a throw out of `readAttributes()` or `appendGeometry()` — the same allocation failure — skips it and leaks the feature. This is not a theoretical path: `VectorLayer::loadFinished()` handles exactly this exception in terms ("the worker THROWING ... the `std::bad_alloc` the geometry cap exists to bound", `vector_layer.cpp:239-256`), and the #152 cleanup guarantee this PR inherits is stated as holding on every return path. Fix: hold both handles in `std::unique_ptr` with the matching OGR deleter (reset the feature one at the `:804` advance), as the dataset and the transformation in the same file already are — `src/camp_map/vector/vector_parse.cpp:139-157`, `:698-714`, `:804`
- [x] (should-fix, Copilot R8) **A reopened once-unavailable layer is APPENDED to the live map, so the on-screen stacking contradicts the order just persisted.** The promotion rewrites the entry IN ITS SLOT in `m_restoredVectorLayerOrder` (`withVectorLayerFilePromoted()`, `vector_layer.cpp:887-919`, called at `autonomousvehicleproject.cpp:493-495`), but the layer itself is constructed with the default row: `new camp::vector::VectorLayer(layers, fname)` (`:502`) reaches `Map::setMapItemParent()` with `row == -1`, which is normalised to "append at the end" (`src/camp_map/map/map.cpp:312-321`). A restored order of `[missing, B]` therefore persists as `[missing, B]` and draws as `[B, missing]` until the next launch — the operator's z-order silently disagrees with the file that will be replayed. Fix: compute the promoted entry's row among the currently loaded layers and insert/move the layer there as part of the promotion, so the model and the order of record agree in the same session — `src/camp/autonomousvehicleproject.cpp:470-502`
- [x] (should-fix, Copilot R8) **ADR-0016 D16's pan-mode arrow cursor is not in force on launch.** D16 decides "In pan mode the cursor is an ARROW, CAMP-wide" (`docs/decisions/0016-read-only-vector-file-layer.md:437-449`), and `setPanMode()` is what installs it (`projectview.cpp:345-367`). Nothing calls `setPanMode()` at startup: the `ProjectView` constructor does not (`:29-45`), `MainWindow` only runs `setupUi()` (`mainwindow.cpp:44`), which applies the `.ui`'s `ScrollHandDrag` property (`mainwindow.ui:178`) and with it Qt's open hand. `mouseMode` is initialised to `pan` and the label reads "Mode: pan", so the view CLAIMS pan mode while showing the hand. CAMP idles in pan mode, and ADR-0016 D15 (`:420-436`) records that in the 2026-09-15 GUI test nobody landed the cursor inside 5 pixels with that hand — the launch state is the state the operator spends most of the session in. Fix: call `setPanMode()` after the UI property has been applied (in `MainWindow` after `setupUi()`, or set the viewport cursor from the `ProjectView` constructor) — `src/camp/projectview.cpp:29-45`, `:345-367`

### False positives
- (Copilot R8, `src/camp/autonomousvehicleproject.cpp:512`) "The `destroyed` connection is unsafe during project teardown: the map's scene-owned vector layers are destroyed from QObject child cleanup AFTER the derived destructor has destroyed `m_vectorLayers`, so `onVectorLayerDestroyed()` iterates a dead vector — and cover normal application shutdown." Third raising; dismissed in rounds 1 and 3 on MEASUREMENT and unchanged since. `~QObject` removes every connection in which the object is the RECEIVER before it calls `deleteChildren()`, so by the time the child `Map`'s `VectorLayer`s are destroyed there is no connection left to the slot. Verified empirically on this host's Qt 5.15.13 with a minimal parent/child/`destroyed`-slot program: the slot does not fire. The member-vs-child destruction ORDER Copilot describes is correct; the conclusion does not follow from it. The added "normal application shutdown" clause is covered by the same mechanism — shutdown destroys the project through `~AutonomousVehicleProject`, which is the measured path. Any path that destroys a layer while the project is still alive is the safety net this connection exists for, and is safe by construction.
- (Copilot R8, `src/camp_map/vector/vector_parse.cpp:146`) "The geometry cap does not bound this allocation: one valid LineString or polygon ring can append an unbounded number of vertices; add a per-geometry vertex/byte budget." Fifth raising (rounds 2, 3, 5, 7). Not a defect to fix in this PR: it is a DEFERRED OPERATOR DECISION recorded in ADR-0016 D11 in terms — "Neither the vertex count of a single ring (100 M vertices is ~1.6 GB of coordinates) nor the size of a single attribute value is bounded, and both are deliberately left to one deferred decision rather than fixed mechanically" (`docs/decisions/0016-read-only-vector-file-layer.md:302-309`) — because silently truncating a ring DRAWS A WRONG SHAPE, which is worse than the honest geometry cap. `kMaxFeatureItems`' header now names what it does not bound and points at D11 (round-7 close). The abort-latency half was fixed in round 2 (`kVertexPollInterval`, `vector_parse.cpp:122`, `:143-148`).
- (Copilot R8, `src/camp_map/vector/vector_layer.cpp:590`) "`setColormap()` stores any caller-supplied name verbatim although the public contract says unknown names fall back to grayscale; a typo renders grayscale while `colormap_` matches no menu action and is persisted unchanged." The failure mode cannot occur: `setColormap()` has exactly ONE production caller, the context-menu lambda, and it captures a name taken verbatim from `marine_colormap::palette_names()` (`vector_layer.cpp:768-773`) — a registry name by construction, so a typo has no door. The other writer of the field, `readSettings()`, case-folds and validates against the registry and falls back to grayscale (`:805-809`, the round-9 fix), so the cross-session case Copilot's contract sentence is about — a palette renamed or removed between builds — is already closed. `colormap_` therefore cannot hold an unregistered name in a shipped build. Residual, recorded rather than fixed: the header's invariant (`vector_layer.h:186-188`) is enforced in one of the field's two writers, so a FUTURE caller of the public setter could reintroduce it — but the three sibling layers validate in neither setter (`raster_layer.cpp:513-521`, `gggs_tile_layer.cpp:1570-1576`, `sonar_live_cache_layer.cpp:1837-1841`), so hardening only this one would make it the odd file out; that is a CAMP-wide question, adjacent to the already-filed rolker/camp#235 (case-sensitive `palette_index()` across five layers), not a defect of this branch.

### Observation (not a finding)
- `VectorDataset::open()` calls `parseVectorLayers(dataset.get())` with DEFAULT `ParseOptions`, so `max_geometries == 0` — the 50 000-geometry cap that protects the read-only layer does not apply to File > Open Geometry. Pre-existing behaviour, preserved by this PR's parser move and not raised by any reviewer; noted here so the next reader of the shared-parser contract does not have to rediscover which consumer is capped — `src/camp/vector/vectordataset.cpp:34`, `src/camp_map/vector/vector_parse.h:117`

### Next step
address-findings (2 must-fix, 2 should-fix; 0 cross-confirmed) via `.agent/scripts/dispatch_subagent.sh --mode in-process --issue 22 --skill address-findings`

## Implementation
**Status**: complete
**When**: 2026-09-17 11:51 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #226 at `e9fc4ac`
**Branch**: feature/issue-22 at `e9fc4ac`
**Addressed**: `## Integrated Review` of 2026-09-17 11:29 -04:00 (round 7 of triage, PR head `a4d7fcd`) — 2 must-fix, 2 should-fix
**Commits**: `280ff12`, `dddda24`, `3a39f8c`, `e9fc4ac`

### Actions
- [x] (must-fix) Every vertex is tested for placeability before it becomes a MISSION ITEM. `camp::vector::placeableGeometry()` reduces a parsed geometry to the vertices `isPlaceable()` admits, with counts, and `VectorDataset::buildItems()` applies it and reports one warning per import. The parser is unchanged — ADR-0016 D4 keeps it faithful — and the helper lives beside `isPlaceable()` so the display and mission paths cannot drift. An exterior with no surviving vertex takes the geometry whole, holes included (`firstPlaceableCoordinate()`'s rule: a polygon rebuilt from its hole is painted SOLID by `Qt::OddEvenFill`) — `src/camp_map/vector/vector_parse.h`, `vector_parse.cpp`, `src/camp/vector/vectordataset.cpp` (`280ff12`). Tests: `UnplaceableVerticesAreFilteredForTheMissionItemConsumer` (a no-SRS layer whose line is 1 good vertex + 3 UTM northings read as degrees, and a mixed polygon with an unplaceable hole; it also asserts the parse diagnostics report NOTHING about this file, which is why the consumer cannot lean on them) and `AnUnplaceableExteriorTakesTheHolesWithIt`. Tested at the helper rather than through `buildItems()`: that is a private member of a `MissionItem` needing an `AutonomousVehicleProject`, which is not constructible in a harness (the reason `mission_insertion.cpp` exists as its own seam).
- [x] (must-fix) `OGRPointIterator` and the per-feature `OGRFeature` are held in `std::unique_ptr` with OGR deleters, so the throw path frees them — `points.push_back()` in the vertex loop and `readAttributes()`/`appendGeometry()` in the feature loop each allocate, and the `std::bad_alloc` the cap exists to bound is the path `loadFinished()` already handles by name. The feature is `reset()` at the loop advance. Parity with the dataset and the transformation in the same file; the #152 "every return path" guarantee now covers unwinding — `src/camp_map/vector/vector_parse.cpp` (`dddda24`). No new test: an allocation failure mid-parse is neither deterministic nor bounded, and these handles are invisible to `GetOpenDatasets()`. The existing coverage is `test_vector_dataset_cleanup` under valgrind, re-run here — `definitely lost: 0 bytes, 0 errors` — which keeps the non-throwing paths honest.
- [x] (should-fix) A reopened once-unavailable layer is MOVED to its restored row instead of being left on top. `camp::vector::vectorLayerRestoredRow()` holds the rule; `openVectorLayer()` calls it for every file that is in the order of record, so the restore loop and the promotion share one path (during the restore it returns the row the layer already has). Rows are read from the live sibling list, so charts/AIS/collision layers are never disturbed and never have to be enumerated — `src/camp_map/vector/vector_layer.h`, `vector_layer.cpp`, `src/camp/autonomousvehicleproject.cpp` (`3a39f8c`). NOTE ON THE FINDING'S MECHANISM: it says the layer is APPENDED via `row == -1`; the constructor path actually reaches `Map::setMapItemParent()` with the default `row == 0`, so the layer lands on TOP. The substance holds either way — a Map row is the reverse of the order of record (row 0 is drawn last), the restore loop opens the order front to back, so `[missing, B]` replays with B on top while the reopened layer sat above it. Tests: `ReopenedLayerTakesItsRestoredRow` + `ReopenedMiddleLayerLandsBetweenItsNeighbours` (the rule, including the two no-anchor cases) and `ReopenedLayerIsMovedToItsRowInTheModel`, which drives a real `camp::map::Map` because the returned row is in CURRENT coordinates and `setMapItemParent()` subtracts one for the removal it performs first — an off-by-one there is a silent no-op. Mutation-checked: without the move the model test fails.
- [x] (should-fix) `MainWindow` calls `m_ui->projectView->setPanMode()` after `setupUi()`, so ADR-0016 D16's arrow cursor is in force at launch instead of `ScrollHandDrag`'s open hand that the `.ui` property installs. It must follow `setupUi()`, not sit in the `ProjectView` constructor, or the `.ui` reinstalls the hand over it — `src/camp/mainwindow.cpp` (`e9fc4ac`). Test: `check_pan_mode_at_launch`, a source check in the style of `check_worker_alt_stacks` — it requires a non-comment `setPanMode()` call in `mainwindow.cpp` AFTER `setupUi(`, and fails with the ADR reference otherwise. A gtest was considered and rejected: `ProjectView` is an executable-side widget pulling in roslink / platform_manager / mission_manager / helm_manager, and what regresses here is the ORDER of two constructor calls, which is what a source check reads. Both mutations (call removed; call moved above `setupUi()`) were verified to fail the check.

### Verification
- `./ui_ws/build.sh camp` clean; `./ui_ws/test.sh camp` → **425 tests, 0 errors, 0 failures, 1 skipped** (419 before this pass).
- `valgrind --leak-check=full ./build/camp/test_vector_dataset_cleanup` → `definitely lost: 0 bytes in 0 blocks`, `ERROR SUMMARY: 0 errors`.
- Pre-commit hooks ran on every commit; no `--no-verify`. All four commits carry the agent identity.
- plan.md **Rev 20** records this pass (design unchanged).

### Note for the re-review
The three `## False positives` and the `## Observation` of the source review were
left alone deliberately — they are dismissals, not actions. The observation
(`VectorDataset::open()` parses with `max_geometries == 0`, so File > Open Geometry
is uncapped) is untouched and still true; this pass filters what that path BUILDS,
not how much of it is read.

### Next step
review-code (re-review the fixes) via `.agent/scripts/dispatch_subagent.sh --mode in-process --issue 22 --skill review-code`
