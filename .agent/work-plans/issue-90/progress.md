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
- [x] GL integration → QOpenGLWidget viewport + beginNativePainting + raw QOpenGLShaderProgram (only Qt-5.15-viable; QRhi is Qt6). Resolved 2026-06-20.
- [x] Colormap → Slice 3 consumes camp#63's GPU colormap facility (marine_colormap GLSL + bake_lut); sequencing I4 Slice1 → camp#63 → I4 Slice3 (no interim LUT). Resolved 2026-06-20.
- [x] Slicing → stacked PRs on feature/issue-90, Slice 1 first. Resolved 2026-06-20.
