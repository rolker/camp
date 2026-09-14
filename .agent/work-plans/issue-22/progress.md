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
