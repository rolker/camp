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

## Integrated Review
**Status**: complete
**When**: 2026-06-05 09:54 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**PR**: #60 at `f32ceb4`
**Sources**: Copilot — 6 review rounds, 33 inline comments (all against earlier stack commits; none at head); local timeline (Local Review (Pre-Push) PR1/PR2/PR3a-i/PR3c-i)
**Cross-source confirmations**: 0 true cross-source (all GitHub findings are Copilot); several themes repeated across ≥2 Copilot rounds (noted inline)
**CI**: none configured on repo (the lone "check" is the Copilot reviewer job)

### Findings
- [ ] (high, Copilot ×6) grid_map: `colormap_`/`last_msg_`/`has_last_msg_`/`process_future_` read+written across ROS-callback, UI (setColormap/contextMenu) and QtConcurrent-worker threads with no synchronization — data race / UB / sporadic crashes — `src/camp2/ros/grids/grid_map.{h,cpp}`
- [ ] (high, Copilot ×3) grid_map: colormap re-render dropped — `setColormap()`→`gridMapCallback(last_msg_)` no-ops while `process_future_.isRunning()`, and no follow-up render is scheduled when the worker finishes, so a ramp change mid-render is silently lost — `src/camp2/ros/grids/grid_map.cpp:40,162`
- [ ] (med-high, Copilot) raster_layer: `setColormap()`→`loadFile()` reassigns the watched future without aborting/joining the prior QtConcurrent job; the dtor only joins the *current* future, so an orphaned load can outlive the layer → potential use-after-free (and a concurrent `colormap_` read/write race) — `src/camp2/raster/raster_layer.cpp:219-227,33-38`
- [ ] (med, Copilot) raster_layer: `imageReady()` applies transform/pos from `LoadResult{}` on an aborted/failed load (zero-filled scale/pos), collapsing the item to zero-scale at origin and clearing status — guard `result.mipmaps.empty()` — `src/camp2/raster/raster_layer.cpp:204-217`
- [ ] (med, Copilot ×2) raster_layer: constant-value scalar raster (`min_value==max_value`) → `ColorMap::color()` returns transparent for every pixel (`!(max>min)`), raster disappears; grid_map already widens the range (grid_map.cpp:100-103), raster_layer does not — `src/camp2/raster/raster_layer.cpp:130-139`, `src/camp2/map/color_map.cpp:96`
- [ ] (med, Copilot ×2) astar: `depthAt(int,int)` called with a `double` coordinate (truncates toward zero, not floor) and only one axis gets floor/ceil neighbour obstacle checks — inconsistent obstacle test on the planner grid — `src/camp/astar.cpp:147,174`
- [ ] (low-med, Copilot) trackline: `planPath()` dereferences `autonomousVehicleProject()` (`avp->hasDepth()`) without a null check — crash on not-yet-attached items — `src/camp/trackline.cpp:264`
- [ ] (low, Copilot ×2) color_map.cpp uses `std::min`/`std::max` with no `#include <algorithm>` (transitive only) — `src/camp2/map/color_map.cpp`
- [ ] (low, Copilot) grid_map.h uses QImage/std::vector/std::string/std::pair without including their headers (transitive only) — `src/camp2/ros/grids/grid_map.h`
- [ ] (low, Copilot) surveyarea: `int` loop over `guidePath.size()` (size_t) — signed/unsigned — `src/camp/surveyarea.cpp:332`
- [ ] (low, Copilot) parity README still says ColorMap is tracked out-of-#59-scope in #63, but this PR implements `camp::map::ColorMap` — doc drift — `docs/parity/README.md:74`
- [ ] (deferred→PR6, Copilot ×7) projectview: mission-item creation + mouseMove gated on `getBackgroundRaster()` non-null though geo conversion is now web_mercator — blocks OSM/WMTS-only operation; resolved by the planned BackgroundRaster retirement — `src/camp/projectview.cpp:72-220`
- [ ] (deferred→PR6, Copilot) chart loaded twice (BackgroundRaster RGBA+mipmaps oracle + RasterLayer display) — transitional cost; goes away with BackgroundRaster retirement — `src/camp/autonomousvehicleproject.cpp:150`
- [ ] (optional, Copilot) astar `Context::depthAt` — primary (x,y) bounds check already present (astar.h:85, returns unknownDepth); residual `depthGrid.size()` vs `gridSize` consistency is low-risk defensive — `src/camp/astar.h:88`

### False positives
- (Copilot ×2) "camp_map / camp_map_ros not added to ament `export_`, so downstream can't link them" — intentional: these are camp-internal split libs consumed only by the CCOMAutonomousMissionPlanner executable; only the pluginlib plugin (rqt_helm_manager) needs export. Installing to lib/ is for the executable's runtime linkage. No downstream ament package links these, so "downstream can't link" cannot occur. (If one is ever added, add them to the export set then.) — `CMakeLists.txt:285,338`
- (Copilot) "projectview Y-flip inverts overlays (markers.cpp:104-112, grids/grid)" — ADDRESSED: the referenced legacy camp Grid/Markers are retired by PR5; the camp2 replacements render right-side-up and Roland sim-verified the flip regression is fixed (see Local Review (Pre-Push) PR3c-i, commit 9b4bcfc) — `src/camp/projectview.cpp:44`

### Resolution (2026-06-05 10:28 -04:00)
Triaged findings fixed on `feature/issue-59` (build + 35-test suite green):
- grid_map data race + dropped colormap re-render → `e1ab49c` (mutex + rendering_/render_pending_ coalescing + teardown join)
- raster_layer UAF + failed-load guard + constant-raster widen → `8be1824`
- astar grid-cell conversions + trackline null-project guard → `bf457d3`
- color_map `<algorithm>`, grid_map.h self-contained, surveyarea signed/unsigned, parity-README ColorMap note → `63a6abf`

Deferred (not regressions; tracked work): projectview BackgroundRaster gating (#9-15) and chart double-load (#7) → resolved by the planned BackgroundRaster retirement (PR6). astar depthGrid sizing (#6) → primary bounds check already present; optional. False positives unchanged.

### Resolution follow-up (2026-06-05 10:49 -04:00)
- projectview BackgroundRaster gating (#9-15) → **done now**, not deferred: `1bcde46` removes the chart-load guard from mission-item creation, mouse-move readout, and the boat-command context menu (all use web_mercator::mapToGeo, chart-independent). Kept the genuinely chart-dependent view-center/extent-fit and the middle-button MeasuringTool coupling. **Needs a runtime check over an OSM/WMTS-only background before merge** (GUI path, not unit-covered).
- Still deferred to BackgroundRaster retirement (PR6): chart double-load (#7); MeasuringTool's bg dependency; full BackgroundRaster + geoToPixel-shim removal; overlay re-homing (AIS/platform/collision_monitor/nav_source); AISManager/CollisionMonitorManager window retirement; dead camp Grid/Markers .h/.ui/manager file deletion; .agents/README.

## BackgroundRaster Retirement — increment plan (PR6)
**Started**: 2026-06-05 11:19 -04:00 — **By**: Claude Code Agent (Claude Opus 4.8 (1M context))

Investigation (30-file BackgroundRaster surface): coordinate conversion is
**already** Web-Mercator (geoToPixel ignores its bg arg via Q_UNUSED), so the
math doesn't block. Three structural blockers remain, plus persistence/metadata.
Ordered, independently-buildable increments — each GUI/safety-affecting step
has a runtime-verification gate:

1. **Depth → standalone DepthRaster** ✅ done (`this commit`). Decouples the
   depth oracle from BackgroundRaster's graphics identity. **Verify in sim**
   (cursor depth + shoal avoidance over a real depth chart) before field use.
2. **Overlay anchoring** → introduce a persistent scene-origin anchor (or
   scene-direct parenting) replacing `setParentItem(bg)` in the 4 overlay
   families (GeoGraphicsMissionItem, AISContact, Platform, CollisionMonitor) and
   the updatingBackground/backgroundUpdated reparenting signals. Coordinate-
   neutral (proven) but GUI-affecting: verify hit-detection, z-order, labels.
   **✅ DONE (2026-06-07): all 4 families re-homed to the Map persistent anchor
   (top_level_items_) + nav_source. Each manager (AIS/Platform/CollisionMonitor)
   gained setAnchor(), wired in MainWindow to project->originAnchor(); items create
   parented to the anchor, updateBackground no longer reparents, and the
   findParentBackgroundRaster() render gates are removed (project unconditionally
   via bg-free geoToPixel). Commits a039539 (incr3 scale prerequisite), afe44ad
   (mission items), 973bce1 (AIS), 2351db4 (platform+nav_source), 29d4a5f
   (collision monitor). Build + 44 tests pass. SIM-VERIFY PENDING: live AIS /
   platform / collision overlays over BOTH a loaded chart and OSM-only.**
3. **Scale helpers** → replace bg->mapScale()/scaledPixelSize()/pixelSize()
   (arrow + ship-outline scaling) with a Web-Mercator scene scale. Verify
   arrow/ship-outline sizing.
   **✅ DONE for the vessel/contact path (2026-06-07, a039539): added
   GeoGraphicsItem::metresPerPixel(geo) = web_mercator::metersPerUnit(at)/|view.m11()|
   (latitude-corrected — raw 1/m11 under-sizes ~24% at 43N) and a bg-free
   geoToPixel(geo) overload; ShipTrack::drawTriangle/drawShipOutline dropped their
   BackgroundRaster* param; AIS + Platform size icons from metresPerPixel(vessel geo)
   not bg->scaledPixelSize() (which was both chart-gated AND dimensionally broken in
   the WM scene). REMAINING (optional, low-priority): the 3 mission-item mapScale
   consumers (geographicsmissionitem drawArrow, waypoint shape(), ETE-label offset)
   still read bg->mapScale()/avp->mapScale() — they are null-safe (guarded, default
   scale) so they don't break with no chart; migrate them to retire
   BackgroundRaster::mapScale/scaledPixelSize/pixelSize entirely in increment 5.**
4. **MeasuringTool** → reparent to the anchor; obtain the project via a stored
   pointer instead of dynamic_cast<BackgroundRaster*>(parent()).
5. **Metadata + persistence + deletion** → move filename/projection display off
   BackgroundRaster; handle its MissionItem write/read (project-file compat —
   verify load of existing .json projects); then delete backgroundraster.{h,cpp}
   (+ its now-redundant depth/image load), backgrounddetails, the geoToPixel(bg)
   overload, findParentBackgroundRaster, georeferenced if unused. Also fixes the
   triage #7 chart double-load.

## Increment 2 (mission-item subset) + chart-swap UAF + A* tests
**When**: 2026-06-07 — **By**: Claude Code Agent (Claude Opus 4.8 (1M context)) — PR #60, HEAD after these commits

Three commits (`ce84b18`, `afe44ad`, `0bc2007`), all pushed:

- **`ce84b18` — A* shoal-avoidance gtests** (`test/test_astar.cpp`, 5 tests; CMake `test_astar` links only `astar.cpp`, no Qt/ROS): open-water reaches goal; shoal-with-gap detoured (path never crosses `<minDepth`); full barrier → no path (fail-safe); unknown-depth barrier impassable (the `getDepth==NaN → unsafe` contract); out-of-grid = obstacle. 44 camp tests pass. `AVP::getDepth`/`hasDepth` provider wiring stays GUI/sim-verified (QAbstractItemModel, not cheaply unit-instantiable) — documented in the test header.

- **`afe44ad` — anchor top-level mission items to persistent scene root** (increment-2 mission-item subset). Root cause of the OSM/WMTS-only crash: `MissionItem::findParentGraphicsItem()` returned the (null-when-no-chart) `BackgroundRaster` for top-level items → `waypoint.cpp` itemChange null-deref + survey pattern orphaned from the scene. Fix reuses `camp::map::Map`'s existing persistent `top_level_items_` (scene root, origin, identity transform, outlives any chart) via new `Map::rootItem()` + `AVP::originAnchor()`; `findParentGraphicsItem()` returns the anchor for top-level items; `GeoGraphicsMissionItem::updateBackground()` no longer reparents to bg (kept positions = absolute Web-Mercator); `waypoint.cpp` itemChange guards the parent deref. **Sim-verified by Roland 2026-06-07: waypoint + survey pattern + trackline all create and render over an OSM-only background, no crash.** Z-order preserved (mission items carry explicit `setZValue(3.0)`; anchor reuse is the ADR-0002 documented scene root). **The other 3 overlay families remain deferred (see increment-2 note above).**

- **`0bc2007` — fix use-after-free replacing a loaded chart.** Loading a 2nd background (KAP→VRT) or deleting the current chart segfaulted: `delete m_currentRasterLayer` bypassed the Map model's `beginRemoveRows`/`endRemoveRows`, desyncing its row count + leaving the MapTreeView a dangling index to the freed layer (logged "endInsertRows: Invalid index (4,0) in model camp::map::Map"). Fix detaches via `m_map->setMapItemParent(layer, nullptr)` before `delete`, at both sites (openBackground + deleteItem). First load unaffected (delete nullptr). **Sim-verified by Roland 2026-06-07: KAP→VRT swap replaces cleanly, no crash.**

**Known follow-ups surfaced during sim verification (NOT yet addressed):**
- **Chart appears in BOTH the Layers tab AND the Mission tab, and Mission-tab entries accumulate** (each `openBackground` does `new BackgroundRaster(fname, m_root)` and `setCurrentBackground` removes the old from the scene but not from the mission model → pile-up + leak). The proper fix is retiring BackgroundRaster as a mission-tree node, which is gated on finishing increment-2 overlay re-homing (the bg's last scene users). Tracked as the bg-off-mission-tree work in increment 5.
- **nav_source (boat icon/track) won't render over OSM-only** (still `findParentBackgroundRaster()`-gated) — same class as the mission-item fix, part of the deferred increment-2 overlay re-homing.
- **Scroll widgets** on ProjectView are vestigial under Web-Mercator drag-pan — small UI cleanup, pending.
- camp still has **no CI and no `.pre-commit-config.yaml`** — these increments rely on manual build + the pre-push review; there is no automated gate.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-07 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: approved (must-fix items were documentation, fixed in follow-up commit)
**Branch**: feature/issue-59 at `0bc2007` (+ doc fixes)
**Mode**: pre-push
**Depth**: Deep (reason: scene-graph parenting + use-after-free / memory-lifecycle change in safety-relevant code)
**Must-fix**: 2 (both documentation) | **Suggestions**: 3

### Findings
- [x] (must-fix) Stale comment in `setCurrentBackground` said "overlays parent to it" — untrue for mission items after the anchor change — `autonomousvehicleproject.cpp:800` — FIXED
- [x] (must-fix) progress.md not updated to record increment-2-core / test / swap-fix / deferred-3-families — FIXED (this entry)
- [ ] (suggestion) nav_source won't render over OSM-only (deferred increment-2 overlay re-homing) — `nav_source.cpp:269,313`
- [ ] (suggestion) astar.cpp `NeighborsMask` prints `NDir:` to stdout unconditionally (pre-existing debug noise) — `astar.cpp:71`
- [ ] (suggestion) camp has no CI / no pre-commit — manual build + pre-push review is the only gate
- Code itself: Claude adversarial (Deep) + governance both found NO code must-fix; parenting/anchor reuse is ADR-0002-compliant, UAF fix correct at both sites, A* tests mutation-checked for real teeth.

## Increments 3 + 2 complete — overlay re-homing + chart-independent scale
**When**: 2026-06-07 — **By**: Claude Code Agent (Claude Opus 4.8 (1M context)) — PR #60

Five commits, all pushed (a039539, afe44ad already covered above, 973bce1, 2351db4, 29d4a5f):

- **a039539 (incr 3)**: chart-independent vessel/contact icon scale. `metresPerPixel(geo)` (view zoom × cos-latitude), bg-free `geoToPixel(geo)`, ShipTrack helpers drop `BackgroundRaster*`, AIS+Platform size from it. Fixes a latent unit bug (`scaledPixelSize` was metres-per-CHART-pixel ÷ display-pixels-per-SCENE-unit, dimensionally wrong in the WM scene) AND removes the chart dependency.
- **973bce1 (incr 2 / AIS)**, **2351db4 (incr 2 / platform+nav_source)**, **29d4a5f (incr 2 / collision monitor)**: each manager gained `setAnchor()` (wired in MainWindow to `project->originAnchor()`); items create parented to the anchor; `updateBackground` stops reparenting; the `findParentBackgroundRaster()` render gates are removed.

Pattern per family: m_background member → m_anchor; `new X(this, m_anchor)`; updateBackground = refresh-only (Q_UNUSED bg); item-side `if(bg)`/`findParentBackgroundRaster()` gates removed → unconditional bg-free projection. CollisionMonitor::polygonPath dropped its bg param + `bg &&` gate (kept >=3-vertex guard); active-fill state logic untouched.

**Consequence now true:** after this, NOTHING parents to the BackgroundRaster anymore. It is still (a) a mission-tree node (the Mission-tab duplication/accumulation Roland flagged), (b) added to the scene as a now-unused non-painting anchor, (c) the georef holder. All three are increment-5 (bg retirement) — now UNBLOCKED, since the overlays no longer need it as a scene anchor.

**SIM-VERIFY PENDING (Roland drives) — safety-relevant, do before trusting:**
- Collision monitor: live slowdown/stop polygons + CollisionMonitorState — zones render in the right place over BOTH a loaded chart and OSM-only, and the amber/red active fill still lights on SLOWDOWN/STOP.
- Platform + nav_source: boat icon, heading, nav track render correctly over both backgrounds; icon size sane at multiple zooms (the scale formula changed — sizing may differ slightly from before, for the better).
- AIS: contacts + prediction render over both backgrounds.
- Mission items with a chart loaded still render correctly (regression check on the scale/anchor changes).

## ADR-0003 accepted — backgrounds-as-layers redesign supersedes the old increment 5
**When**: 2026-06-07 — **By**: Claude Code Agent (Claude Opus 4.8 (1M context)) — PR #60 (a7c44b2)

The original increment-5 framing (keep BackgroundRaster as a headless depth/georef
holder, serialize it for back-compat) is **superseded by ADR-0003**. Roland's design
input reframed it: multiple backgrounds is a wanted feature (one-at-a-time was a
limitation), and we are NOT preserving the old project-file format. So backgrounds
fully dissolve into the layer model. Decisions (see docs/decisions/0003): stackable
RasterLayers; independent visual + depth layer trees (depth = subtree in Layers,
co-sourced layers independent); BackgroundRaster deleted; split persistence
(backgrounds = Map/app state, missions = project files); depth-only load offers to
generate a depth-shaded visual layer (#63); RasterLayer gains a SYNCHRONOUS extent
(B2) so fit-to-extent / zoom-on-open survives the async pixel warp.

**Staged implementation plan (each independently buildable on the #59 branch):**
1. **RasterLayer synchronous extent** (camp2 shared lib): compute geotransform+size
   in the ctor (cheap GDAL metadata), set scenePos + valid boundingRect immediately;
   pixel warp stays async; preserve the abort/reload path. Foundation; no behavior
   change for camp/camp2.
2. **Layers-tab Remove action** on RasterLayer context menu (prerequisite to taking
   backgrounds out of the mission tree — else charts become un-removable).
3. **Retire BackgroundRaster as a mission node** (the big one): openBackground stops
   creating a BackgroundRaster + mission insert; backgrounds are pure RasterLayers
   (stacking enabled); ProjectView fit-to-extent reads the RasterLayer; retire
   m_currentBackground/setCurrentBackground; collapse geoToPixel overloads → bg-free;
   redirect the 2 mapScale glyph readers to AVP::mapScale(); delete backgroundraster.
   {h,cpp} + BackgroundRasterType + findParentBackgroundRaster; mission file stops
   carrying backgrounds. Keep georeferenced.{h,cpp} (DepthRaster/VectorDataset use it).
4. **Depth-layer tree**: DepthLayer MapItem type + depth subtree in the Layers tab +
   MapTreeView; getDepth(geo) walks enabled depth layers in tree order; multiple
   depth layers; DepthRaster (incr 1) is the backend.
5. **Map-state persistence**: background/depth layer set (filename, visible, order,
   opacity, colormap) persists across restart (camp2-style); missions stay project-file.
6. **Depth-only → generate visual layer**: offer ColorMap depth-shaded RasterLayer
   (#63), independent of the depth layer.

Status: ADR locked; implementation not yet started. Sim-verify gates from increments
2+3 (collision monitor / platform / AIS over chart + OSM-only) still pending Roland.

## Sim verification — platform overlay (increments 2+3)
**When**: 2026-06-07 — **By**: Roland (operator-driven), set up by Claude Code Agent

Launched marine_simulation `simulator_launch.py` via ros2launch_session with the
worktree env sourced, so the operator station ran the **worktree CAMP build**
(re-homed overlays) with the 13283 KAP chart auto-loaded. **Roland verified: the
ben platform overlay (boat icon, heading, nav track) renders correctly on the
chart — no regression from the increment-2 anchor re-home or the increment-3
metresPerPixel icon scale.** Covers the "platform WITH a chart" gate (the case
most at risk from re-homing). Still unverified (need specific data / a no-chart
CAMP): AIS contacts, collision-monitor zones, and the OSM-only-background path —
those remain open sim gates.

## ADR-0003 stage 1 DONE — RasterLayer synchronous extent
**When**: 2026-06-07 (b741d1d) — **By**: Claude Code Agent (Claude Opus 4.8 (1M context))

RasterLayer::initExtent() (called from ctor before the async load) reads the
reprojected geotransform + dimensions (no pixels) and applies the world
transform/pos synchronously; boundingRect() now returns the reprojected
dimensions immediately. imageReady() no longer re-applies the placement (with a
defensive fallback). No camp behavior change yet (fit-to-extent still reads the
bg until stage 3); camp2 layers now position the instant they're created. Build
+ 44 tests pass.

**Staging note (2/3 coupling):** stage 2 (Layers-tab Remove action) is coupled to
stage 3 (retire BackgroundRaster ownership). The displayed chart's RasterLayer is
currently owned by AVP (m_currentRasterLayer/m_currentBackground/m_depthRaster), so
a standalone Layer "Remove" would dangle those pointers. The clean order is stage 3
FIRST — backgrounds become independent layers owned by the Map model, AVP sheds the
single-current-background pointers — and the Remove action then falls out naturally
on those independent layers. So fold stage 2 into stage 3 (or do 3 then 2).

## Plan Authored — backgrounds-as-layers core (stages 3+2+5)
**Status**: complete
**When**: 2026-06-07 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-59/plan-backgrounds-core.md` at `8765adc`
**PR**: https://github.com/rolker/camp/pull/60 (existing #59 implementation PR)
**Phases**: 6 ordered steps in one PR; steps 1+6 land together (persistence gap),
3+4 are the large ones (geoToPixel collapse + BackgroundRaster delete)

### Open questions
- [ ] Persistence store: QSettings (matches camp2) vs config file? Lean QSettings.
- [ ] Depth-provider ordering across stacked charts pre-stage-4: load order? Lean yes.
- [ ] fit-to-extent re-fits on every chart open (not just first)? Lean yes (matches today).

## Step 1 (part 1/?) — depth provider list
**When**: 2026-06-07 (e881816) — **By**: Claude Code Agent (Claude Opus 4.8 (1M context))

m_depthRaster (single) -> std::vector<DepthRaster*>; openBackground appends,
deleteItem removes by filename (any chart), getDepth walks in load order (first
valid wins), hasDepth = any valid. DepthRaster gained filename(). Multiple loaded
charts now each contribute depth. Build + 44 tests pass.

**Remaining in step 1 (the entangled core, larger):** visual RasterLayer stacking
(retire single m_currentRasterLayer) + retire m_currentBackground/setCurrentBackground/
getBackgroundRaster. This interlocks with step 2 (fit-to-extent re-source from the
RasterLayer) and step 3 (collapse geoToPixel — getBackgroundRaster has consumers in
geoToPixel(AVP*) + ProjectView fit-to-extent + deleteItem), so the next buildable
increment necessarily spans 1+2+3-partial. Steps 1+6 still must land together
(persistence gap) before the mission file drops backgrounds.

## Step 1+2+3 (part 2) — fit-to-extent re-source + geoToPixel(AVP*) bg-free
**When**: 2026-06-07 (3228621) — **By**: Claude Code Agent (Claude Opus 4.8 (1M context))

ProjectView fit-to-extent now reads AVP::currentBackgroundExtent() (= the
RasterLayer's sceneBoundingRect, sync-valid via stage 1) instead of bg
boundingRect/pixelToGeo. geoToPixel(point, AVP*) delegates to the bg-free
overload. Ordering fix: setCurrentBackground (emits backgroundUpdated→fit) now
runs AFTER the RasterLayer is created. Build + 44 tests pass.

**getBackgroundRaster() remaining consumers** (all trivial/vestigial, go with the
BackgroundRaster deletion): the mission-tree node itself; the presence gates in
ProjectView beforeUpdateBackground + updateBackground `if(!bg)`; deleteItem's bg
branch. The hard georef consumer is gone.

**NEXT (the hardest, interlocked piece — best with fresh context):** remove the
BackgroundRaster from the mission tree (fixes the duplication) + Map-state
persistence (steps 1-final + 6, must land together) + collapse the remaining
geoToPixel overload + delete backgroundraster.{h,cpp} + Layers-tab Remove action.

**Open sim-verify gates:** fit-to-extent zoom-on-chart-load still correct (extent
source changed bg→RasterLayer); platform overlay already verified.

## Integrated Review
**Status**: complete
**When**: 2026-06-07 15:25 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**PR**: #60 at `7b40a06`
**Sources**: 1 live (Copilot review @ `7b40a06`, 4 inline comments) + local timeline (Pre-Push @ `0bc2007`, prior Integrated Review @ `f32ceb4`)
**Cross-source confirmations**: 0 at head SHA (1 lineage link — see finding 2)
**CI**: no real build/test CI on camp; the lone "copilot-pull-request-reviewer" check is the reviewer action firing (success), not a build

### Findings
- [ ] (low-med, Copilot) surveyarea: `generateAdaptiveTrackLines()` derefs `project = autonomousVehicleProject()` (`project->hasDepth()`) with no null guard; `autonomousVehicleProject()` returns nullptr for an unattached item (missionitem.cpp:37). Sibling `TrackLine::planPath` already guards (`!avp || … || !avp->hasDepth()`, trackline.cpp:265) — add matching guard for parity. Lineage: sibling of the f32ceb4 trackline.cpp:264 finding (accepted+fixed) — `src/camp/surveyarea.cpp:254`
- [ ] (low-med, Copilot) grid_map: `RCLCPP_WARN_STREAM_THROTTLE(…, clock, 2/2.0, …)` — throttle period is **ms**, so 2 ms disables throttling → log spam on malformed/empty GridMap messages; use 2000 — `src/camp2/ros/grids/grid_map.cpp:101,106`
- [ ] (low, Copilot) raster_layer: `GDALOpen(filename.toLatin1(), …)` — implicit QByteArray→const char* + temporary lifetime is correct (NOT a bug), but `toLatin1()` silently corrupts non-ASCII paths → GDALOpen null → raster silently fails to load; switch to `toUtf8()` at both sites — `src/camp2/raster/raster_layer.cpp:84,129`

### False positives
- (Copilot) test_color_map.cpp:76 "no main() → fails to link": disproven by build artifacts. `ament_add_gtest` used with no NO_MAIN, so gtest_main supplies main(); binary `ui_ws/build/camp/test_color_map` exists and its gtest XML shows 7 tests / 0 failures / 0 errors (ran 2026-06-07 14:51). Other test files defining their own main() also link (object-file main wins over the archive's). No "self-main mode" is configured.

## Review fixes applied (2026-06-07 15:55 -04:00)
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

Two of the three Integrated-Review findings fixed (build + 44 tests pass):
- `4a54408` surveyarea: null-guard `project` before `hasDepth()` in
  `generateAdaptiveTrackLines` (parity with TrackLine::planPath).
- `6663934` grid_map: `WARN_STREAM_THROTTLE` period 2/2.0 → 2000 ms.
- Deferred (low): raster_layer `toLatin1()` → `toUtf8()` encoding nit.

## Step 3 finish — geoToPixel collapse + last BackgroundRaster readers decoupled
**When**: 2026-06-07 16:05 -04:00 (`6349e42`) — **By**: Claude Code Agent (Claude Opus 4.8 (1M context))

Removed the `geoToPixel(point, AVP*)` and `geoToPixel(point, BackgroundRaster*)`
delegating overloads; migrated all ~40 call sites to the single bg-free
`geoToPixel(point)`. Redirected the two glyph-scale readers (Waypoint::shape,
GeoGraphicsMissionItem::drawArrow) from `getBackgroundRaster()->mapScale()` to
`AVP::mapScale()` — identical with a chart loaded (both track
ProjectView::scaleChanged), zoom-aware over OSM/WMTS instead of frozen at 1.0
(matches the existing avp->mapScale() in updateETE). Dropped AISContact's dead
`dynamic_cast<BackgroundRaster*>(parentItem())`. Deleted callerless
`findParentBackgroundRaster`. Build + 44 tests pass.

**Sim-verify gate (no chart loaded):** waypoint/arrow glyphs now scale with zoom
over OSM/WMTS rather than staying fixed — confirm this looks right (intended
chart-independent behavior, but a visible change in the no-chart case).

**NEXT — the interlocked core (steps 1-final + 6 + 4 + 5, land together):**
- step 1-final: `openBackground` appends a stacked `RasterLayer` to
  `m_map->topLevelLayers()` (no `BackgroundRaster` mission node); retire
  `m_currentBackground`/`m_currentRasterLayer`/`setCurrentBackground`/
  `getBackgroundRaster`; `currentBackgroundExtent`/`backgroundUpdated` re-sourced
  to the new layer.
- step 6 (MUST land with step 1): Map-state persistence — chart currently
  persists via the mission `.json` `BackgroundRaster` node (missionitem.cpp:184);
  removing that node strands the chart unless the loaded chart-layer list is
  persisted to QSettings (camp2 already persists per-layer settings by itemID()
  — add the *list*) and restored on startup.
- step 4: delete `backgroundraster.{h,cpp}`, `backgrounddetails.{h,cpp}`,
  `detailsview.cpp:110` bg branch, `missionitem.cpp:182` read dispatch,
  `BackgroundRasterType` enum entry, CMake refs. Entanglement to resolve first:
  `MeasuringTool` still reaches the AVP via `dynamic_cast<BackgroundRaster*>(parent())`.
- step 5: `Layer::contextMenu` base "Remove" action (detach via Map model +
  delete; for a chart layer also drop its DepthRaster provider).
- Heavy sim-verify: chart load/stack/remove, existing-`.json` project load (old
  BackgroundRaster node ignored, no crash), persistence round-trip across restart.

## BackgroundRaster retirement IMPLEMENTED — backgrounds-core sub-plan complete
**When**: 2026-06-07 17:05 -04:00 — **By**: Claude Code Agent (Claude Opus 4.8 (1M context))

The interlocked core landed in two commits; backgrounds-core sub-plan (ADR-0003
stages 3+2+5) is now fully implemented. Build + 44 tests pass at each.

- `73559df` **Retire BackgroundRaster** (steps 1-final + 6 + 4): chart load now
  appends a stacked, Map-owned `RasterLayer` (display) + `DepthRaster` (depth)
  instead of a mission-tree `BackgroundRaster`; charts STACK. Retired
  `m_currentBackground`/`m_currentRasterLayer`/`setCurrentBackground`/
  `getBackgroundRaster`; `currentBackgroundExtent` = last layer's extent;
  `hasBackground()` gates ProjectView center-save/fit. **Map-state persistence**:
  ordered chart-filename list → QSettings `backgrounds/files`, recreated at
  startup via `restorePersistedBackgrounds()` (MainWindow calls it after the
  background signals are wired); per-layer settings restore by itemID since
  layers rebuild in order. `backgroundUpdated`/`updatingBackground` are now
  parameterless (every consumer ignored the bg pointer). MeasuringTool parents
  to the Map scene-origin anchor + keeps the project for speed()/ETE. Dropped the
  DetailsView bg branch + BackgroundDetails, the MissionItem read dispatch, the
  dead `BackgroundRasterType` enum; deleted `backgroundraster.{h,cpp}` +
  `backgrounddetails.{h,cpp,ui}` + CMake/includes. Legacy mission `.json`
  BackgroundRaster nodes are ignored on load (no back-compat, per ADR-0003).
- `55ca7f2` **Layers-tab Remove** (step 5): base `Layer::contextMenu` gains a
  "Remove" (detach via Map model + drop from scene + delete). AVP listens to
  `m_map::rowsAboutToBeRemoved` (`onChartLayerRemoved`) to drop the matching
  DepthRaster + bookkeeping + re-persist — camp2 stays unaware of the project.

Also `6349e42` (step-3 geoToPixel collapse) + review fixes `4a54408`/`6663934`.

**Net since #60 head 7b40a06: 46 files, +405/−749.** `m_root`/`m_currentRasterLayer`
single-chart model gone; the whole `BackgroundRaster` class hierarchy removed.

### Sim-verify gates (Roland — no CI on camp; build+tests are the only auto gate)
1. Chart load over OSM-only: opens, fits to extent, renders.
2. Stack a 2nd chart: both present in Layers tab, view recenters (no jump).
3. Layers-tab **Remove** on a chart: disappears; depth-aware planning (shoal A*)
   no longer sees its soundings; gone after restart.
4. Persistence round-trip: load chart(s) → quit → relaunch → charts reappear
   (with their per-layer opacity/visibility/colormap).
5. Open an existing mission `.json` that had a BackgroundRaster node: loads with
   no crash, chart simply absent from the mission tree.
6. Glyph scale over OSM-only (waypoint/arrow) scales with zoom (step-3 change).
7. Middle-button MeasuringTool works over OSM-only (now anchor-parented).
