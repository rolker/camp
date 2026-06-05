---
issue: 59
---

# Issue #59 — Port camp2 multi-layer + OSM/WMTS map system into deployed CAMP

## Plan Authored
**Status**: complete
**When**: 2026-06-01 20:27 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-59/plan.md` at `15eed92`
**PR**: https://github.com/rolker/camp/pull/60 (`[PLAN]` prefix)
**Phases**: 6 stacked PRs (PR1 sweep, PR2 substrate import, PR3 scene swap + UI split, PR4 mission items, PR5 ROS overlays + manager retirement, PR6 cleanup)

### Open questions
- [x] Radar message type → **N/A: this boat has no radar.** Grids path stays general-purpose; ROS 1 RadarSector still deleted (PR1). Not a deployment gate.
- [x] `getDepth()` consumers → **Preserve depth as a first-class layer type.** Multiple depth layers allowed, toggled in tree; `getDepthRaster()` → `getDepth(geo)` query over enabled depth layers in tree order (overlap resolved by order). A\*/survey/cursor depth kept (PR3/PR4).
- [x] QSettings migration → **One-time reset**, no migration code.
- [x] ADR? → **Yes — start an ADR system in `camp`** (`docs/decisions/` + architecture ADR), lands with PR3.
- [x] `MeasuringTool`/`Orbit` scale → **Preserve behavior**; port `bgr->mapScale()` to `MapView` viewport scale, distances stay geodesic. Implementation detail.

## Plan Review
**Status**: complete
**When**: 2026-06-01 21:31 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context)) (in-context — author self-review)

**Plan**: `.agent/work-plans/issue-59/plan.md` at `814e7b8`
**PR**: https://github.com/rolker/camp/pull/60
**Verdict**: changes-requested

### Findings
- [x] (must-fix) PR3 build-break → **addressed**: revised plan adds a `geoToPixel`→`geoToMap` shim in PR3a and parity-gates retirement of `backgroundraster`/`georeferenced` to PR6 (delete only when replacement ≥ parity)
- [x] (must-fix) PR2 camp2 framing → **superseded by shared-lib decision**: camp2 is NOT retired; PR2 now extracts `libcamp_map` and relinks the camp2 exe (self-verifying), camp links it in PR3a
- [x] (suggestion) Split PR3 → **done**: PR3a scene/chart+shim+ADR, PR3b tabbed UI, PR3c depth-layer
- [x] (suggestion) A* understated → **noted in plan PR4**: pixel-grid → multi-layer `getDepth(geo)` needs a defined planning grid (possibly its own sub-PR)
- [x] (suggestion) Pin tests → **done**: Testing table added (web_mercator round-trip PR2, depth-order resolution PR3c, marker-action parity PR5); manual for Qt/UI
- [ ] (note) review-issue not run on #59 (optional)

### Plan revision (post-review, 2026-06-01)
Discussion added two corrections: (a) **don't assume retiring camp2** → end-state is a shared `libcamp_map` both binaries link; (b) **parity audit before any replacement** → committed parity matrices (Phase 0) + a load-bearing rule: never delete a camp original until its replacement is verified ≥ parity (audit is bidirectional — keep camp2's `CUBE`, preserve camp's `DELETEALL`). Tests added where practicable. Plan reframed as "adopt the framework, preserve the functionality" with three buckets (true-replacements / camp-only carryover / camp2-only additions).

## PR1 implemented — dead-code sweep
**Status**: complete
**When**: 2026-06-02 20:18 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

Branch merged up to `origin/jazzy` first (picked up `b5c40d3` ros_widget stale-stamp guard). PR1 = Scope A dead-code sweep, **1597 deletions, 0 additions, no behavior change**. Deleted `radar/`, `geoviz/`, `sonar_manager/`, `sound_play/`, `scaledview.{h,cpp}`; removed radar's orphaned menu actions/slots/`showRadar`+`selectRadarColor` signals (decided: full removal, the consuming connections were already commented out so they were invisible no-ops); removed the inert `actionSay_something` menu item (sound_play remnant); removed `RadarDisplayType`/`GeovizDisplayType` enum entries; cleaned `CMakeLists.txt`. `./ui_ws/build.sh camp` green (both binaries link). Commit `3a70f62`.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-02 20:22 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: approved

**Branch**: feature/issue-59 at `3a70f62`
**Mode**: pre-push
**Depth**: Standard (reason: 1597 lines changed, but deletions-only / no new logic)
**Must-fix**: 0 | **Suggestions**: 0

Static analysis: no findings (zero added lines). Two independent adversarial readers (fresh-context Claude subagent + Copilot CLI) both returned clean. Highest-risk item — removing two middle entries from the `GeoGraphicsItem::type()` enum renumbers later values — verified SAFE: project save/load is JSON keyed by string type names (`missionitem.cpp` read/write), `type()` overrides return symbolic names, no integer-literal comparisons / `QDataStream` / `QSettings` persist the enum. `mainwindow.ui` action defs + addaction insertions removed consistently (no dangling refs).

### Findings
- [ ] No issues found. LGTM.

## Phase 0 — parity audit (doc)
**Status**: complete
**When**: 2026-06-02 20:40 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

Committed `docs/parity/` (README + raster.md + markers.md + grids.md). Built by reading actual camp + camp2 source (3 parallel analysts), every feature claim cited `file:line`. Confirmed the three buckets vs `CMakeLists.txt`: true-replacements (raster/markers/grids), camp-only carryover (AIS/collision/platform/nav_source/mission-items/vector/measuring/orbit — camp2/ros has only geometry/grids/markers), camp2-only additions (OSM/WMTS/tree/geometry/tools).

**Retirement blockers found (gate PR5/PR3c):**
- raster: depth band + `getDepth(geo)` is camp-only → PR3c. Color tables/mipmap/compositing = parity (verbatim copy). camp2 adds async load + metersPerUnit + opacity.
- markers: camp2 adds CUBE (keep), BUT `DELETEALL` is a **camp2 regression** (clears one ns, not all — spec violation); DELETE **leaks** empty Marker/MarkerNamespace objects; camp-only: empty-frame drop, already-expired ingest drop, text scaling, TF-buffering.
- grids: OccupancyGrid colormap = **byte-for-byte parity**. GridMap **"speed" semantic colormap + fixed /3.0 range is camp-only** (camp2 = auto-range grayscale) → bag-replay verify. Suspected camp2 bug: warn throttle `2` vs `2000` ms. Verify OccGrid centering + lazy-subscribe.

**Open decisions RESOLVED (2026-06-02):** (1) GridMap "speed" colormap — situational, **dropped** → adopt camp2 grayscale auto-range; reusable selectable-colormap facility (`camp::map::ColorMap`, also depth-shading consumer) split out to **[#63](https://github.com/rolker/camp/issues/63)**, out of #59 scope. (2) Marker fill-alpha — **make it a config option** (not hardcoded); default at PR5. Parity docs updated accordingly.

## PR2 — extract libcamp_map / libcamp_map_ros
**Status**: complete
**When**: 2026-06-02 22:15 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Lib-boundary decision was SPLIT** (plan Open Questions). Implementing it exposed a flaw in the earlier "core is ROS-free / one-directional" verification: that scan only caught direct `#include <rclcpp>` and **missed** `Map::Map()` constructing a `camp::ros::Node` (`map/map.cpp:32`) — `ros::Node` lives in the ros layer → **core↔ros circular dependency**, so a pure file-move split can't link. Surfaced to Roland; he chose **B: split + invert Node creation**.

**Inversion (breaks the cycle):** `Map` no longer creates the `ros::Node`; it stores its `ToolsManager` and exposes `Map::toolsManager()`. The app layer (`MainWindow`) now attaches the node there — same parent (tools_manager), same item-tree discovery, same timing. Core is now genuinely ROS-free.

**Result:** `libcamp_map` (pure Qt/GDAL core, 24 src) + `libcamp_map_ros` (ros/ layers, 16 src, links camp_map + rclcpp/tf2). camp2 exe = main/ only, links camp_map_ros. **Verified:** build green (both libs + both exes); `ldd libcamp_map.so` shows **zero ROS libs** ✓; `libcamp_map_ros.so` links libcamp_map + rclcpp/tf2 ✓. Dropped unused Qt5::Test from libs (kept on camp2 exe — QAbstractItemModelTester) + unused `<QApplication>` in map.cpp. `map/utilities.{h,cpp}` are empty stubs, never built — left as-is. Commit `49dc283`. Runtime (GUI) not verified headless; wiring preserved.

**Lesson:** dependency-boundary verification must follow transitive includes through intermediate headers + check cross-namespace *instantiations* (`new other::Type`), not just grep direct system includes.

## Local Review (Pre-Push) — PR2
**Status**: complete
**When**: 2026-06-02 22:30 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: approved

**Branch**: feature/issue-59 at `49dc283` (PR2 code commit)
**Mode**: pre-push
**Depth**: Standard (reason: build-system + cross-cutting C++ change, despite small line count)
**Must-fix**: 0 | **Suggestions**: 0

Two independent adversarial readers (fresh-context Claude subagent + Copilot CLI) both returned clean on the inversion + split. Confirmed: Node ownership is via the QGraphicsItem **scene-tree** (Node→LayerManager→MapTool→MapItem→QGraphicsObject; `setParentItem`), so the `auto ros_node` local going out of scope in MainWindow doesn't destroy it — identical owner chain to before (scene ← Map ← MainWindow); no leak/dangle/double-free. Discovery unchanged: ros layers find the node via `parentOfType<Node>()` walking up from children created inside `Node::nodeStarted()`, independent of who constructs the Node. Shutdown equivalent (`qApp` ≡ `QCoreApplication::instance()`; idempotent quit). CMake: source union byte-identical to old 42-entry list; ament emits plain `target_link_libraries` (no keyword-mix); AUTOMOC/UIC/RCC global → per-target; rpath via ament env hook. `map_tiles.ui` unused (dead in old build too, no regression).

### Findings
- [ ] No issues found. LGTM.

## PR3a — architecture ADR (Web-Mercator scene + layer model)
**Status**: in progress (ADR landed; scene code pending)
**When**: 2026-06-04
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

First PR3a commit (`ae06045`): `docs/decisions/0002-web-mercator-scene-and-layer-model.md`. Bootstraps the architecture ADR ahead of the scene-swap code, recording the already-made decisions (Web-Mercator scene owned by `Map`; two-model Layer/Mission split tabbed in one dock; depth-as-first-class-layer with `getDepth(geo)` order resolution gating BackgroundRaster retirement at PR3c; split shared lib `libcamp_map`/`libcamp_map_ros`, camp2 kept as sandbox).

**Design crux pinned by the ADR (verified against source):** the `geoToPixel` shim is a one-function swap in `geographicsitem.cpp` — replace `bg->geoToPixel(point)` with `web_mercator::geoToMap(point)`, *keeping* the parent-offset subtraction (`ret - parentItem()->scenePos()`, `geographicsitem.cpp:42`), which is coordinate-system-agnostic. The real PR3a work is therefore **reparenting overlay items** out of `BackgroundRaster` and into `Map`'s scene/layers, plus relinking camp to `camp_map_ros` and swapping scene ownership (`AutonomousVehicleProject::m_scene` → `Map::scene()`). Nothing deleted in PR3a; old paths run through the shim until parity-gated retirement.

**Paused for Roland's review of the ADR/approach before writing the scene-swap code** (foundational, 24-file blast radius).

## PR3a-i — adopt Web-Mercator scene (substrate swap)
**Status**: complete (build-verified; runtime/visual pending)
**When**: 2026-06-04
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

Commit `362157c` (6 files, +107/-68). Deployed camp's scene is now the `camp::map::Map` Web-Mercator scene. Substrate swap only — no overlays migrated, nothing deleted.

**Key design pivot during implementation (vs the ADR's first sketch):** rather than introduce a new overlay-root item and rewire `findParentGraphicsItem` / `updateBackground` / all six live ROS overlays, **`BackgroundRaster` stays in the scene as a non-painting origin anchor** (`ItemHasNoContents` + raised Z). It keeps its georeferencing + depth band (depth oracle) and remains the QGraphicsItem every overlay already parents to — so the entire existing overlay + `findParentBackgroundRaster` machinery is untouched. Mission items, AIS, platform, grid, collision-monitor, nav_source, markers, and the measuring tool all keep working with zero per-overlay edits. The chart *image* is drawn by a reprojected `raster::RasterLayer` (exact GDAL warp to EPSG:3857) added in `openBackground`; the anchor at the origin makes the `geoToPixel` shim (`web_mercator::geoToMap(point) - parent.scenePos()`) reduce to absolute Web-Mercator.

**Chart placement question (Roland, 2026-06-04):** confirmed we must reproject (option B) — a georeferenced non-Web-Mercator raster cannot sit correctly on a Web-Mercator scene via a corner-fit; `RasterLayer` does the real GDAL warp. `BackgroundRaster` (no reprojection, has depth) coexists as depth-only until PR3c.

**Touched:** CMake (link `camp_map_ros`); `autonomousvehicleproject.{h,cpp}` (own `Map`, scene from Map, RasterLayer display, anchor demotion); `geographicsitem.cpp` (shim→`geoToMap`); `projectview.cpp` (mouse↔geo→`web_mercator`); `waypoint.cpp` (drag readout→`mapToGeo`).

**Verification:** `./ui_ws/build.sh camp` green (both binaries + libs; only pre-existing unused-param/sign-compare warnings). **Runtime/visual not yet checked** — needs a display + a chart to confirm chart/overlay alignment, arrow/label scaling (keys off `mapScale()`, pixel-tuned — expected follow-up), and that the OSM/WMTS base layers from `Map`'s `BackgroundManager` render.

**Known follow-ups:** arrow/label scale magnitudes under metres; `BackgroundRaster` still builds display pixmaps it no longer paints (slim in PR3c); RasterLayer not removed on chart delete (PR3b layer-tree); spurious anchor boundingRect at origin (cosmetic).

**Next:** PR3a-ii folded into PR3a-i (RasterLayer display already here). Remaining PR3a sequence → PR3b (tabbed Layers/Mission UI) → PR3c (depth-as-layer, retire BackgroundRaster).

### PR3a-i — visual verification + Y-flip fix (2026-06-04)
Roland ran the local build (chart 13283). **Bug found + fixed:** chart + overlays rendered upside down — Web Mercator is Y-up, QGraphicsView is Y-down; `camp::MapView` compensates with a negative-Y scale, `ProjectView` was missing it. Fix (commit `754ffb1`): `scale(1,-1)` in the ProjectView ctor + `fitInView` the chart extent on load (both preserve the flip; uniform zooms keep the sign). Overlay labels use `ItemIgnoresTransformations` → stay upright; mouse↔geo unaffected. **Roland confirmed: chart renders north-up.** Substrate swap visually verified. Still to do: arrow/label scale cosmetic (pixel-tuned `mapScale`), then `/review-code` + push.

### PR3a-i — second visual pass: markers/grids flip (deferred to PR5)
Roland tested in the simulator (survey pattern + live boat track on chart 13283). Everything positioned via the per-point `geoToPixel`→`geoToMap` shim renders correctly (survey pattern, nav trail, platform path, AIS, collision monitor). **One regression found:** overlays that render in a *local frame with the old Y-down convention baked in* fight the new view Y-flip:
- `markers.cpp` LINE_STRIP/LINE_LIST: local ENU offsets added with a reflection (`y = x·sinr - y·cosr`) that was correct for the old pixel scene → double-flips under the view flip. Also has a `pixel_size_` (chart metres/pixel) units assumption that's wrong against Web-Mercator scene units.
- `grids/grid.cpp`: `drawImage` of the costmap pixmap → mirrored by the flipped view (same class of issue).
- AIS / collision-monitor are fine (per-point geoToPixel).

**Decision (Roland, 2026-06-04): DEFER to PR5, documented — do not patch.** `markers` and `grids` are the Bucket-A "true replacement" overlays already slated for wholesale swap to camp2's `ros/markers` + `ros/grids`, which are written for the flipped Web-Mercator scene. Patching camp's soon-to-be-deleted versions (flip **and** units) is throwaway work; the substrate swap (the PR3a-i deliverable) is proven correct by every other overlay. **Consequence:** camp's marker-path + costmap overlays render flipped on-branch between PR3a-i and PR5. Noted in `docs/parity/markers.md` + `grids.md`.

## Local Review (Pre-Push) — PR3a-i
**Status**: complete
**When**: 2026-06-04
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: approved (findings addressed)

**Branch**: feature/issue-59 at `e6f7647`
**Mode**: pre-push
**Depth**: Standard (reason: cross-cutting C++ substrate change; PR1/PR2 in-diff already reviewed/approved, specialists focused on the PR3a-i delta f2960b7..HEAD)
**Specialists**: Claude adversarial (fresh subagent) + Copilot adversarial (cross-model) on the delta; governance/plan-drift by lead. Static: build green + pre-commit ran each commit.
**Must-fix**: 0 | **Suggestions**: 4 (3 addressed, 1 noted)

Adversarial verdict: no crash-class or coordinate-correctness bug on the normal single-chart path; scene ownership (no double-free — scene parented to Map, AVP dtor empty), construction order, Y-flip/fitInView interaction, nested-item shim, RasterLayer↔overlay frame agreement, depth oracle, and null guards all verified correct. **Both reviewers independently flagged the RasterLayer lifecycle (cross-confirmed) as the top issue.**

### Findings
- [x] (suggestion, cross-confirmed) RasterLayer lifecycle unmanaged — open-2nd-chart stacks both; delete orphans layer — `autonomousvehicleproject.cpp:openBackground/deleteItem`. FIXED `e6f7647` (track m_currentRasterLayer; replace on open, tear down on delete).
- [x] (suggestion) `topLevelLayers()` nullable, deref'd in RasterLayer ctor — `autonomousvehicleproject.cpp`. FIXED `e6f7647` (null guard).
- [x] (suggestion, Copilot) fitInView used only 2 corners → wrong for rotated/sheared charts — `projectview.cpp:updateBackground`. FIXED `e6f7647` (4-corner bounding box).
- [ ] (suggestion, noted) over-broad link `camp_map_ros` vs `camp_map` — left as-is: the exe is already a full ROS node (ament deps unchanged) and PR5 adopts camp2 ros overlays, so the breadth is forward-looking, not added weight. Dropped `setSceneRect` (pan-margin/centerOn-clamp behaviour change) — mitigated by world-spanning OSM base layers giving a large itemsBoundingRect; revisit with PR3b viewport work.

## PR3b — tabbed Mission/Layers dock
**Status**: complete (build + visually verified; pushed)
**When**: 2026-06-05
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

Commit `da3c376`. Left panel's single mission treeView → QTabWidget: **Mission** tab (existing treeView / AutonomousVehicleProject, edits the plan) + **Layers** tab (camp::map_tree_view::MapTreeView bound to project->map() — backgrounds, OSM/OpenSeaMap/NOAA-WMTS tiles, chart RasterLayers, inline visibility checkboxes + opacity delegate). Built in code (reparent treeView into the tab widget at its splitter slot) — no .ui surgery. detailsView follows active tab (mission selection on Mission; cleared on Layers; layer detail widgets = future). The Layers tab gives user management of the stacked chart layers from PR3a-i (visibility toggle), closing the multi-chart review note. **Roland visually verified: tabs + layer checkboxes work.**

**Deferred (documented):** background still shows as a mission-tree node — removal entangled with project save/load persistence; lands with PR3c depth-as-layer / persistence migration.

## PR3c-i — depth via getDepth(geo); A* self-defined planning grid
**Status**: complete (build + logic/review-verified; runtime needs a depth raster)
**When**: 2026-06-05
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

Commits `09aab4a` (core) + `6ae55ef` (review fixes). Decoupled depth + A* from BackgroundRaster's pixel grid — the precondition for retiring it. `AutonomousVehicleProject::getDepth(geo)` walks a depth-provider list (first valid wins, NaN if none) + `hasDepth()` gates planning. **A* redesigned (Roland's call) to a self-defined square grid in Web-Mercator metres, fixed N=256/axis sized from the planning area** (not a raster) → bounded compute. `astar::Context` drops BackgroundRaster*, carries gridSize + pre-sampled depthGrid; **unknown depth = obstacle** (Roland) via a sub-minDepth sentinel. planPath builds the grid per segment (bbox(start,goal)+0.5 margin), pre-samples depth per cell, runs A*, keeps exact endpoints. surveyarea/cursor/menu-gating all on getDepth(geo)/hasDepth().

## Local Review (Pre-Push) — PR3c-i
**Status**: complete
**When**: 2026-06-05
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: approved (findings addressed)
**Branch**: feature/issue-59 at `6ae55ef`
**Mode**: pre-push | **Depth**: Standard (A* algorithm + coordinate redesign)
**Must-fix**: 0 | **Suggestions**: 4 (2 fixed, 2 noted)

Claude adversarial: grid/bbox/centring math, fill↔read index consistency (y*N+x), unknown=obstacle sentinel, and A* termination (start/finish obstacle → empty → straight fallback, no loop/crash) all verified CORRECT; no must-fix.

### Findings
- [x] (suggestion, Copilot) endpoint drift + segment-join mismatch — A* success used cell-center approximations for whole path; now interior cells only + exact endpoints — `trackline.cpp`. FIXED `6ae55ef`.
- [x] (suggestion, Claude) NaN depth → NaN swath/garbage point in adaptive lines — guard added — `surveyarea.cpp`. FIXED `6ae55ef`.
- [ ] (suggestion, Copilot — DESIGN, for Roland) A*-failure straight-line fallback can route a direct line through unknown/shallow water, partly undoing the unknown=obstacle safety. Old behaviour too, but now more reachable. Options: warn the user on fallback / leave the segment unplanned. Roland's call.
- [ ] (suggestion, Claude) perf: N*N (65536) per-cell OGR transforms per segment; batch into one Transform() call if planning feels slow on long multi-segment lines.

### PR3c-i — runtime verified (2026-06-05)
Tested with the real NH GRANIT Lake Massabesic bathy (`ccomjhc_project11/projects/2026-Lake_Massabesic/data/massabesic_bathy.tif`, depth m positive-down 0–16.8, EPSG:26919), displayed via a local imagery+depth VRT (`~/data/test_charts/massabesic_chart.vrt`). **Roland confirmed:** depth-shaded chart renders correctly georeferenced/oriented on the Web-Mercator scene over OSM tiles; **adaptive track lines** tighten in shallows / widen in deep water (getDepth(geo) swath scaling); **A* "Plan path"** on a trackline works (self-defined grid, unknown=obstacle). Depth pipeline + A* grid redesign validated end-to-end. (Adaptive density high = pre-existing swath params, not a correctness issue.)

## PR5 step 1 — grids+markers via camp2 ros overlays (unified node)
**Status**: complete (build + runtime-verified; pushed)
**When**: 2026-06-05
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

Commit `9b4bcfc`. Replaced camp's flipped, BackgroundRaster-parented grids/markers with camp2's scene-correct ros overlays. **camp::ros::Node adopt constructor** (tools_manager, node, buffer): uses camp's existing rclcpp node+buffer instead of spawning its own thread (Roland's "unify on camp's node" call), defers manager creation via QTimer::singleShot(0, this, ...) (safe — `this` receiver guard), and the dtor skips rclcpp::shutdown()/thread-teardown when owns_thread_ is false (node belongs to host). Skips camp2's GeometryManager (create_geometry_manager_=false) since camp keeps its own collision-zone PolygonStamped renderer (avoids double-draw). MainWindow attaches the Node to project->map()->toolsManager() on first rosConnected (m_map_ros_started guard). Retired camp's GridManager+MarkersManager (creation/members/menu actions/slots). camp2 standalone path unchanged. **Roland verified in sim: grids/markers render right-side-up in the Layers tab; flip regression fixed.**

Remaining PR5: re-home camp-only overlays (AIS, platform/ship_track, collision_monitor, nav_source) onto Map layers off the BackgroundRaster anchor; retire AISManager/CollisionMonitorManager windows; then retire BackgroundRaster + remove geoToPixel shim (PR6). Deferred: camp Grid/Markers class files now dead (delete in cleanup); camp2 geometry for non-collision polygons (re-enable with a collision-name exclusion if wanted).
