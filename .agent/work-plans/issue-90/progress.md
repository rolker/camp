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

## Plan Review
**Status**: complete
**When**: 2026-06-19 22:01 -04:00
**By**: Claude Code Agent (Claude Opus 4.8)

**Plan**: `.agent/work-plans/issue-90/plan.md` at `8ec0a35`
**PR**: PR-less
**Branch**: feature/issue-90
**Verdict**: approve-with-suggestions

### Findings
- [ ] (suggestion) ADR-0002 §D2/§D5 citations are the *unh_marine_autonomy* bathy-store ADR, not camp's local ADR-0002 (Web-Mercator scene); qualify the repo so a reader doesn't mis-resolve to `docs/decisions/0002` in camp — `plan.md:88-93`
- [ ] (suggestion) CMake: current `find_package(Qt5 5.15 COMPONENTS ...)` lacks `Gui`/`OpenGL`; `QOpenGLShaderProgram`/`QOpenGLFunctions` live in Qt5::Gui. Add `Gui` (and `OpenGL` if used) to COMPONENTS + `camp_map` link list, not just rely on transitive Widgets — `plan.md:74`
- [ ] (suggestion) Make the Y-flip explicit: scene uses `scale(s, -s)` (map_view.cpp:19), so the GL MVP MUST be derived from the live QPainter `worldTransform()` (as the plan says) rather than a hand-rolled ortho; call out depth-test off + premultiplied-alpha blend state for CPU QPainter layers compositing over the GL viewport — `plan.md:39`
- [ ] (suggestion) Slice 1 verification ("existing CPU layers still render") is qualitative; name the concrete check — load a KAP chart (workspace/13283) + OSM tiles + a marker, confirm registration unchanged, and capture a camp#98 zoom/pan memory baseline — `plan.md:31,99`
- [ ] (suggestion) geoToMap-parity unit test is the right gate but specify tolerance + sample spread (e.g. < 1e-6 m over φ ∈ [−85°,85°] incl. Massabesic latitude); the CPU mirror must call `web_mercator::geoToMap` directly so it can't silently drift from the shader — `plan.md:76,86`
