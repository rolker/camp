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

All 5 suggestions folded into the plan at `4e89b7b` before implementation.

## Implementation
**Status**: in-progress (Slice 1 code + unit tests complete; runtime registration verification pending)
**When**: 2026-06-20 01:35 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Branch**: feature/issue-90 at `914ec4a`
**Slice**: 1 of 4 — GL plumbing + GPU-warp tile layer + registered grayscale sidescan

**Built**:
- `MapView` viewport → `QOpenGLWidget` (`FullViewportUpdate`) so layers can issue native GL.
- `GggsTile` (`raster/gggs_tile.{h,cpp}`): GDAL native band-1 read (no warp VRT) + geotransform→geographic extent + NoData/data-range + lazy `R32F` `QOpenGLTexture`.
- `GggsTileLayer` (`raster/gggs_tile_layer.{h,cpp}`): `map::Layer` + `QOpenGLFunctions`; loads a tile directory, `boundingRect` = union of per-tile Web-Mercator extents via `web_mercator::geoToMap`; `paint()` → `beginNativePainting()` with an inline shader pair (vertex = geoToMap warp over a 16-strip latitude mesh, MVP from live `QPainter::worldTransform()`; fragment = auto-ranged grayscale, NoData 0 → discard; premult-alpha blend, depth-test off). GL teardown via `QPointer<QOpenGLWidget>` makeCurrent.
- `BackgroundManager`: "Open tile store" directory action → `GggsTileLayer`.
- CMake: `Qt5::Gui` added to COMPONENTS + `camp_map` link; new sources; `test_gggs_tile` registered.
- `GggsTileLayerType` item type.

**Verified**: builds clean (camp, only pre-existing warnings); **camp test suite 63 tests, 0 failures** incl. `test_gggs_tile` (extent-from-geotransform, NoData-excluded-from-range, all-NoData-crossed-range, missing-file-invalid, shader-warp==geoToMap parity over φ∈[−85°,85°] incl. Massabesic, rel tol <1e-6).

**Pending (Slice 1 close-out)**: runtime registration check — regenerate sidescan tiles from the #173 bag, open the tile store in CAMP, confirm tiles register vs a KAP chart (`workspace/13283`) + OSM tiles + a marker, and capture a camp#98 zoom/pan memory baseline. Needs a CAMP GUI session.

## Implementation — update (offscreen-FBO pivot + settings)
**Status**: Slice 1 code + headless verification complete; awaiting in-CAMP visual confirm
**When**: 2026-06-20 13:30 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Branch**: feature/issue-90 at `91cfd6e` (`cb225a1` render pivot, `91cfd6e` settings)

**Render approach revised** (Roland's call, plan §Approach updated): native GL in a QGraphicsView item gets **no current GL context** on this stack — Wayland composites via a software backingstore, not a GL surface; `beginNativePainting` → `QOpenGLFunctions` crashed on a null context (gdb-confirmed). Layer now warps tiles in its **own offscreen GL context** (`QOpenGLContext` + `QOffscreenSurface` + `QOpenGLFramebufferObject`) → `QImage` → `drawImage(boundingRect, img)`, cached by on-screen size. Portable (X11/Wayland/software); **`QOpenGLWidget` viewport reverted** (camp#98 risk moot).

**Verified headlessly** (no window): `test_gggs_render` warps a synthetic tile (asserts geometry + orientation) and, via `GGGS_TEST_STORE`, the **real** `~/data/stores/sidescan_backscatter/draft` store → a coherent **north-up** backscatter mosaic (pixel analysis: QImage row 0 = south → `drawImage` + MapView `scale(s,−s)` flip = north up). **camp tests 66, 0 fail, 1 skip** (env-gated real render).

**Settings persistence** (camp#90, folded in): the #59/#60 port wired view/window persistence only into the camp2 *test harness*, not deployed `MainWindow` → map pos/zoom + window geometry stopped saving. `closeEvent` now saves window geometry/state + `projectView` scale/center to QSettings; ctor restores geometry + (deferred to next event loop) map scale/center.

**Open**: (1) **in-CAMP visual confirm** of tile registration + the settings round-trip (close/reopen); (2) the 544 m Massabesic patch is sub-pixel at default world zoom — zoom in to see it; (3) Slice 2 = visible-region-only render + tessellation/seam tuning; eviction/large-survey memory.
