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

The PR targets `feature/issue-103` (stacked).

## Approach

1. **Add `smooth` field to `RasterFieldItem`** — the flag travels from layer to
   renderer so the GL filter decision stays centralized in `RasterGlRenderer`.

2. **Add `smooth_interpolation_` member to each data layer** — default `false`
   (Nearest). Non-scalar `RasterLayer` (RGBA charts) always renders smooth
   regardless; the toggle and its persistence are Scalar-only.

3. **Update `RasterGlRenderer::renderToImage`** — for Scalar items, use
   `item.smooth ? Linear : Nearest` instead of always Nearest. Rgba path
   stays `Linear` as today.

4. **Set `item.smooth` in each layer's `items()` / `itemsIntersecting()`** so
   the renderer picks up the per-layer choice.

5. **Condition `SmoothPixmapTransform` in each `paint()`** on the same flag,
   so the QPainter blit matches the GL filter.

6. **Add context-menu toggle** — checkable "Smooth interpolation" action in each
   layer's `contextMenu()`. `RasterLayer` gates it behind the existing
   `if(!is_scalar_) return;` guard (same gate as Colormap/Range).

7. **Persist** `smooth_interpolation_` via `readSettings`/`writeSettings` under
   key `"smooth_interpolation"` in the existing `MapItem/<settingsKey()>` group.

## Files to Change

| File | Change |
|------|--------|
| `src/camp_map/raster/raster_field_source.h` | Add `bool smooth = false` to `RasterFieldItem` |
| `src/camp_map/raster/raster_gl_renderer.cpp` | Scalar filter: `item.smooth ? Linear : Nearest` |
| `src/camp_map/raster/gggs_tile_layer.h` | Add `bool smooth_interpolation_ = false` |
| `src/camp_map/raster/gggs_tile_layer.cpp` | Set `item.smooth` in `itemsIntersecting()`; condition `paint()` hint; add context-menu toggle; read/write settings |
| `src/camp_map/raster/raster_layer.h` | Add `bool smooth_interpolation_ = false` |
| `src/camp_map/raster/raster_layer.cpp` | Set `item.smooth` in `items()`; condition `paint()` hint (`smooth_interpolation_ \|\| !is_scalar_`); add context-menu toggle (scalar-gated); read/write settings |
| `src/camp_map/ros/live_coverage/sonar_live_cache_layer.h` | Add `bool smooth_interpolation_ = false` |
| `src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp` | Set `item.smooth` in `itemsIntersecting()`; condition `paint()` hint; add context-menu toggle; read/write settings |

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
| `RasterFieldItem` (add `smooth`) | `items()` callers in all three layers | Yes |
| `RasterGlRenderer::renderToImage` Scalar filter logic | `test_raster_gl_renderer.cpp` if it asserts filter state | Verify in step 3 — no existing filter-state test found; likely no change needed |
| `paint()` in all three layers | No other callers of `cached_image_` | Yes — no other blit sites |
| Per-layer persistence | `readSettings`/`writeSettings` in each layer | Yes |

## Open Questions

- None — approach is fully specified by orchestrator context. RGBA `RasterLayer`
  (palette/RGB charts) always gets `smooth = true` via `format_ == Format::Rgba`
  check; the context-menu toggle is scalar-only, matching the existing Colormap gate.
  No operator confirmation needed before implementation.

## Estimated Scope

Single stacked PR targeting `feature/issue-103`.
