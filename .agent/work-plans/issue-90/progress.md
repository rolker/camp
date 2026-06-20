---
issue: 90
---

# Issue #90 — GGGS-tiled raster map layer: GPU display-time warp (bathy + sidescan)

## Plan Authored
**Status**: complete
**When**: 2026-06-20 00:55 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-90/plan.md` at `d7e4d6c`
**Branch**: feature/issue-90 at `d7e4d6c`
**Phases**: 4 slices (stacked PRs; Slice 1 GL plumbing + registered static tile → Slice 2 warp correctness/multi-tile → Slice 3 bands+colormap+watcher → Slice 4 live transport, deferred to I3)

### Open questions
- [ ] GL integration approach — recommend QOpenGLWidget viewport + beginNativePainting + raw QOpenGLShaderProgram (only Qt-5.15-viable path; QRhi is Qt6); risk = app-wide viewport swap + camp#98 interaction. Proceed?
- [ ] Colormap source — minimal 1-D LUT uniform now (reuse ColorMap stops), converge with camp#63 later, vs block on #63?
- [ ] Slicing — stacked PRs (Slice 1 first) vs one larger PR? Recommend stacked.
