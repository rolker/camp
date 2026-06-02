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
