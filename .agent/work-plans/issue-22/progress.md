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
