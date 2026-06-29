# Plan: RasterFieldSource — unified source-agnostic render abstraction

## Issue

https://github.com/rolker/camp/issues/134

## Context

camp#121 Part B (live cache rendering through `SonarLiveCacheLayer`) is merged
(PR#139). The duplication this issue targets is confirmed:

- `raster/gggs_tile_layer.cpp` — GL shader (FBO + fragment shader, colormap LUT,
  NoData discard). GDAL-backed tiles, QtConcurrent off-thread load, atomic
  `pixelsLoaded()` publish.
- `ros/live_coverage/sonar_live_cache_layer.cpp` — **identical** shader, marked
  `// NOTE: shader duplicated; unify via RasterFieldSource camp#134`. In-memory
  tiles, all state on GUI thread.
- `raster/raster_layer.cpp` — **separate** QPainter/mipmap path, no GPU shader,
  no colormap LUT, no per-cell NoData discard (CPU `std::isnan` only).

NaN NoData bug: GLSL `v == u_nodata` is false when `v` is NaN. Chart and
backscatter stores use NaN as NoData — those cells render opaque today. The fix
(`if(v != v) discard;`) lands once in the unified shader.

camp#121 Part B merged; `marine_colormap` GPU-lib migration deferred (separable).
Single PR, full consolidation (operator-decided).

## Approach

1. **ADR** — Write `docs/decisions/0007-raster-field-source-interface.md`:
   `RasterFieldSource` API (`bands()`, `metadata(band)`, `items(band, extent)`,
   `dataRange(band)`), ownership model (textures owned by source, non-owning
   pointers valid for the duration of `paint()`), threading contract (`items()`
   called on GUI/GL thread; sources ensure texture validity before returning).
   Note `marine_colormap` deferral.

2. **Interface** — Add `src/camp_map/raster/raster_field_source.h`:
   ```cpp
   struct BandMeta { float nodata; bool has_nodata; QString units; };
   struct RasterFieldItem {
     QOpenGLTexture* texture;  // R32F, owned by source
     QList<QPointF> geo_vertices;  // Web-Mercator geo coords
     float nodata; bool has_nodata;
   };
   class RasterFieldSource {
   public:
     virtual QStringList bands() const = 0;
     virtual BandMeta metadata(const QString& band) const = 0;
     virtual QList<RasterFieldItem> items(const QString& band,
                                          const QRectF& extent) const = 0;
     virtual QPair<float,float> dataRange(const QString& band) const = 0;
   };
   ```

3. **Shared GL renderer** — Add `src/camp_map/raster/raster_gl_renderer.{h,cpp}`:
   Owns compiled shader program + LUT texture (per-GL context; caller makes
   context current before any call). Unified fragment shader:
   ```glsl
   if(v != v) discard;  // NaN NoData (GLSL 1.20-portable; isnan() requires 1.30+)
   if(u_has_nodata != 0 && v == u_nodata) discard;  // finite sentinel (sidescan/0)
   ```
   Methods: `render(items, data_min, data_max, mvp)`, `setColormap(cm)`,
   `ensureProgram()`, `ensureLut()`. Vertex tessellation (16 lat subdivisions,
   CPU-side geo→Web-Mercator warp, relative to extent origin) moves here from both
   layer files. Nearest texture filter set once in `render()` per item.

4. **Migrate `GggsTileLayer`** — Implement `RasterFieldSource` (or adapt the existing
   tile storage to satisfy the interface inline). Delete `kVertexShader`,
   `kFragmentShader`, `ensureProgram()`, `ensureLut()`, per-vertex tessellation
   from `gggs_tile_layer.cpp`. Replace `renderImage()` body with
   `renderer_.render(items(band_, extent), data_min_, data_max_, mvp)`. Keep
   QtConcurrent load + atomic `pixelsLoaded()` publish unchanged.

5. **Migrate `SonarLiveCacheLayer`** — Same pattern: implement `RasterFieldSource`,
   delete the duplicated shader block, delegate `renderImage()` to `renderer_`.
   All GUI-thread invariants unchanged (ROS callbacks still marshal to GUI thread).

6. **Migrate `RasterLayer`** — Replace QPainter mipmap path with `RasterGlRenderer`:
   Add `QOpenGLContext*` + `QOffscreenSurface*` + `QOpenGLFramebufferObject*` to
   class (matching GggsTileLayer pattern). In `loadAndReprojectFile()`, upload
   pixels as R32F texture (scalar path) or RGBA8 (palette/RGB path). In `paint()`,
   call `renderer_.render()` instead of `painter->drawPixmap()`. Implement
   `RasterFieldSource` returning the single-file item. Keep async load + abort flag.
   Remove mipmap pyramid (GPU path provides LOD via texture sampling). Verify
   visual parity: palette/RGB bands, scalar colormap, NoData cells transparent.

7. **Verify CPU range path consistency** — `GggsTileLayer` uses `isfinite` to skip
   NaN values when folding `data_min_`/`data_max_`. Confirm `SonarLiveCacheLayer`'s
   `foldAutoRange()` and `RasterLayer`'s range fold also exclude NaN, so the CPU
   range and shader discard are consistent.

8. **Tests** — Extend `test_gggs_render.cpp` with NaN NoData test (chart store,
   verify null cells render transparent). Add `test_raster_gl_renderer.cpp`:
   unit tests for renderer (NaN discard, finite-sentinel discard, LUT bake, Nearest
   filter). Extend `test_raster_layer_gdal_cleanup.cpp` (or add new test) for
   RasterLayer under the GPU path.

## Files to Change

| File | Change |
|------|--------|
| `src/camp_map/raster/raster_field_source.h` | **New** — abstract interface |
| `src/camp_map/raster/raster_gl_renderer.h/cpp` | **New** — shared GL renderer, unified shader + NaN fix |
| `docs/decisions/0007-raster-field-source-interface.md` | **New** — ADR |
| `src/camp_map/raster/gggs_tile_layer.h/cpp` | Remove own shaders; implement `RasterFieldSource`; delegate `renderImage()` |
| `src/camp_map/ros/live_coverage/sonar_live_cache_layer.h/cpp` | Delete duplicated shaders; delegate `renderImage()` |
| `src/camp_map/raster/raster_layer.h/cpp` | Replace QPainter/mipmap with GL path; implement `RasterFieldSource`; add GL context |
| `test/test_gggs_render.cpp` | Add NaN NoData render test (chart store) |
| `test/test_raster_gl_renderer.cpp` | **New** — renderer unit tests |
| `CMakeLists.txt` | Add new source files and test |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | Single PR, no speculative features; `marine_colormap` deferred |
| Improve incrementally | Steps 4→5→6 are ordered; each can be reviewed in sequence even within one PR |
| A change includes its consequences | Parity checklist + tests for all three adapters; NaN fix verified on real data stores |
| Capture decisions, not just implementations | ADR step 1 records interface design + threading contract |
| Test what breaks | New tests for NaN discard, finite sentinel, LUT correctness, RasterLayer GPU parity |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0001 (camp: Adopt ADRs) | Yes | New camp ADR `0007-raster-field-source-interface.md` |
| ADR-0002 (camp: Web-Mercator scene/layer model) | Yes | All three adapters remain Web-Mercator scene objects; geometry warp moves into renderer, not out |
| ADR-0008 (workspace: ROS 2 conventions) | No | Pure Qt/GL; no ROS message data in render path |
| ADR-0013 (workspace: progress.md vocabulary) | Yes | This plan + progress entry satisfy it |

## Consequences

| If we change... | Also update... | Included? |
|---|---|---|
| Shader discard logic | CPU range fold (NaN exclusion must match) | Yes — step 7 |
| `RasterLayer` from QPainter to GL | Mipmap pyramid removed; `QPixmap::scaledToWidth()` chain deleted | Yes — step 6 |
| Both GL layers lose own shader strings | `sonar_live_cache_layer.cpp` `// NOTE: shader duplicated` comment | Yes — deleted in step 5 |
| `marine_colormap` migration | Per-band colormap defaults | No — deferred (noted in ADR) |

## Open Questions

- `RasterLayer` RGB/palette bands: current QPainter path draws composite RGBA QImage.
  The GL renderer is single-band R32F. For multi-band color files, determine whether
  to (a) composite on CPU → upload RGBA8 texture (simplest, sidesteps per-band select),
  or (b) expose per-band R32F + channel compositor in the renderer. Recommendation: (a)
  for this PR; band-per-channel select is a separate future issue.

## Estimated Scope

Single PR. ~600–800 lines new (interface + renderer + ADR + tests);
~400 lines deleted (duplicate shaders, mipmap pyramid, QPainter path).
