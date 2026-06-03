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
