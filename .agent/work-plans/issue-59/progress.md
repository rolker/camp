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
- [ ] (must-fix) PR3 won't build: retiring `backgroundraster`/`georeferenced` before overlays migrate breaks `GeoGraphicsItem::geoToPixel` callers — move retirement to PR6 or add a `geoToPixel`→`geoToMap` shim — `plan.md` PR3 / Approach
- [ ] (must-fix) PR2 mischaracterizes camp2: it's already a 2nd executable target (`CMakeLists.txt:237+`, own `main.cpp`) — reframe as share-modules-into-camp-target + retire standalone camp2 exe; add main/.qrc/target reconciliation — `plan.md` PR2
- [ ] (suggestion) Split PR3 into 3a scene/chart+shim+ADR, 3b tabbed UI, 3c depth-layer — `plan.md` PR3
- [ ] (suggestion) A* migration understated: pixel-grid iteration → multi-layer `getDepth(geo)` needs a defined planning grid — `plan.md` PR4
- [ ] (suggestion) Pin concrete tests (web_mercator round-trip, depth-order resolution) to PRs; camp has gtest `test/` — `plan.md` Files/Principles
- [ ] (note) review-issue not run on #59 (optional)
