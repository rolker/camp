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
- [ ] Confirm radar/costmap feeds `grids/grid.cpp` as `OccupancyGrid` vs `grid_map` (verify vs deployed launch before PR5)
- [ ] Where is `BackgroundRaster::getDepth()` consumed? Web-Mercator tiles carry no depth band — need replacement source or retained depth raster
- [ ] QSettings migration: per-background → per-`itemID()` — accept one-time reset or migrate keys?
- [ ] Worth an ADR in `camp` for the Web-Mercator scene + two-model (layers vs mission) split?
- [ ] `MeasuringTool`/`Orbit` rendering at scale under a global projection (currently scale via `bgr->mapScale()`)
