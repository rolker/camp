# Plan: Per-layer smooth interpolation toggle for data layers

## Issue

camp#132 — Make data-layer QPainter blit interpolation operator-selectable
(default Nearest/unblended for data layers; basemap/RGBA charts stay smoothed).

## Context

Post-#103 (this worktree branches from `feature/issue-103`), three data-layer
`paint()` methods unconditionally set `QPainter::SmoothPixmapTransform` when
blitting the GL-rendered QImage to the scene:

- `gggs_tile_layer.cpp` (GGGS tile stores)
- `raster_layer.cpp` (single-GeoTIFF scalar and palette/RGB charts)
- `sonar_live_cache_layer.cpp` (live sonar coverage)

The GL texture filter for Scalar data is already `Nearest` (camp#122), so
`SmoothPixmapTransform` is the remaining smoothing source. With the
viewport-sized FBO from #103, the blit is ~1:1, so the softening is sub-pixel
only — but it still affects cell-faithful QA rendering. The operator wants to
A/B Nearest vs smoothed on real data.

RGBA charts and basemap/OSM/WMTS imagery must stay smoothed unconditionally
(not data under QA). The colormap LUT texture stays Linear (it's a color ramp,
not data — correct as-is).

The PR targets `jazzy` (#103 / PR#173 merged 2026-07-24; this branch carries
the landed base via a forward-merge of jazzy).

## Approach

**Plan-review must-fix 1 (folded): the GL Scalar filter stays `Nearest`
ALWAYS.** Driving it from the toggle would reintroduce the camp#122 NoData
halo — Linear filtering blends the finite NoData sentinel with neighboring
data into fabricated values the shader's exact-equality discard can't catch
(the property `test_raster_gl_renderer.cpp:114-115` guards). The toggle
governs ONLY the QPainter blit hint; a true GL smooth mode (NoData→NaN at
upload) is out of scope.

1. **Add `smooth_interpolation_` member to each data layer** — default `false`
   (Nearest). Non-scalar `RasterLayer` (RGBA charts) always blits smooth
   regardless; the toggle and its persistence are Scalar-only.

2. **Condition `SmoothPixmapTransform` in each `paint()`** on the flag
   (`RasterLayer`: `smooth_interpolation_ || !is_scalar_`).

3. **Add context-menu toggle** — checkable "Smooth interpolation" action in each
   layer's `contextMenu()`. `RasterLayer` gates it behind the existing
   `if(!is_scalar_) return;` guard (same gate as Colormap/Range).

4. **Persist** `smooth_interpolation_` via `readSettings`/`writeSettings` under
   key `"smooth_interpolation"` in **each layer's EXISTING settings group**
   (plan-review must-fix 2): `GggsTileLayer`/`SonarLiveCacheLayer` use
   `MapItem/<settingsKey()>`; `RasterLayer` persists under `MapItem/<itemID()>`
   (raster_layer.cpp:558-559,608-609) — do NOT move it to settingsKey().

## Files to Change

| File | Change |
|------|--------|
| `src/camp_map/raster/gggs_tile_layer.h` | Add `bool smooth_interpolation_ = false` |
| `src/camp_map/raster/gggs_tile_layer.cpp` | Condition `paint()` hint; add context-menu toggle; read/write settings (settingsKey group) |
| `src/camp_map/raster/raster_layer.h` | Add `bool smooth_interpolation_ = false` |
| `src/camp_map/raster/raster_layer.cpp` | Condition `paint()` hint (`smooth_interpolation_ \|\| !is_scalar_`); add context-menu toggle (scalar-gated); read/write settings (itemID group) |
| `src/camp_map/ros/live_coverage/sonar_live_cache_layer.h` | Add `bool smooth_interpolation_ = false` |
| `src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp` | Condition `paint()` hint; add context-menu toggle; read/write settings (settingsKey group) |

Paths relative to `ui_ws/src/camp/`.

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Toggle is per-layer in the context menu the operator already uses; default Nearest preserves the faithful QA baseline; behavior is visible and reversible |
| Capture decisions, not just implementations | This plan explains why basemap/LUT stay Linear and why the flag lives on `RasterFieldItem` rather than being passed as a separate render arg |
| A change includes its consequences | Tests checked; `raster_gl_renderer.cpp` is the one filter-decision site — no duplicated logic |
| Only what's needed | No global setting added; no new widget; no new class; the toggle reuses the existing context-menu + QSettings pattern |
| Improve incrementally | Stacked on #103; single PR, small diff |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0007 (unified GL render path) | Yes | Filter decision stays in `RasterGlRenderer::renderToImage`, not in individual layers — consistent with the "ONE render path" intent |
| ADR-0008 (LUT stays identity-transfer) | No | LUT texture filter unchanged (Linear, correct — it's a color ramp) |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| GL filter behavior | NOTHING — the GL Scalar filter is untouched (must-fix 1); `test_raster_gl_renderer.cpp:114-115` (Nearest / no-bleed assertion) keeps passing unchanged | Yes |
| `paint()` in all three layers | No other callers of `cached_image_` | Yes — no other blit sites |
| Per-layer persistence | `readSettings`/`writeSettings` in each layer | Yes |

## Open Questions

- None. RGBA `RasterLayer` (palette/RGB charts) always blits smooth; the
  context-menu toggle is scalar-only, matching the existing Colormap gate.
- PR description must record the default-Nearest QA rationale (plan-review
  suggestion): interpolation fabricates values that aren't in the data and can
  mask the artifacts the operator is looking for; Nearest is the faithful
  default for data under QA, smoothing is the opt-in.

## Estimated Scope

Single PR targeting `jazzy` (~60-line diff + toggle wiring).
