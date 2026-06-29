# Plan: Adopt marine_colormap in camp (replace internal ColorMap)

## Issue

https://github.com/rolker/camp/issues/141

## Context

camp's internal `camp::map::ColorMap` (Grayscale/Viridis/Turbo) is used across 7 files
(`color_map.{h,cpp}`, `raster_gl_renderer`, `gggs_tile_layer`, `sonar_live_cache_layer`,
`raster_layer`, `grid_map`). `marine_colormap` is now merged (jazzy) and offers 6 palettes
(grayscale/bronze/thermal/viridis/turbo/quality) plus `bake_lut()` for LUT upload. rviz and rqt
already adopt it. camp#134 unified the render path into `RasterGlRenderer` + `RasterFieldSource`;
this issue swaps the colormap source, exposing the full 6-palette registry to camp operators.

**LUT integration decision (record in ADR-0008):** bake marine_colormap's ramps into the existing
RasterGlRenderer 256×1 LUT-texture path via `bake_lut(palette, TransferParams{}, 256)` — do NOT
swap in marine_colormap's GPU shader. The #134 render contract (NaN + finite-NoData discard,
Nearest, per-band range uniforms) is preserved unchanged in the fragment shader.

**Persisted-name migration:** existing settings use lowercase names (the current `ColorMap::name()`
already returns lowercase). shared names grayscale/viridis/turbo are identical in marine_colormap.
Add a case-insensitive fallback in `readSettings()` for any deployments that saved capitalized
names.

## Approach

1. **Write parity-baseline test** — add `test/test_color_map_parity.cpp`: hardcode expected
   LUT samples from the old ColorMap ramps for grayscale/viridis/turbo at t={0,0.5,1} (and a
   few interior points); compare against `marine_colormap::bake_lut()` output (≤2 per channel
   tolerance for rounding). Commit alongside old `color_map.cpp` so CI confirms parity before
   the old code is removed.

2. **Add marine_colormap dependency** — `package.xml`: add `<depend>marine_colormap</depend>`.
   `CMakeLists.txt`: add `find_package(marine_colormap REQUIRED)` and
   `target_link_libraries(camp_map PUBLIC marine_colormap)`.

3. **Migrate RasterGlRenderer** — replace `map::ColorMap colormap_` with
   `std::string colormap_name_{"grayscale"}`. Update `setColormap(const std::string& name)` /
   `colormap() const → const std::string&`. Replace `ensureLut()` body: call
   `marine_colormap::find_palette(colormap_name_)` → `bake_lut(pal, TransferParams{}, 256)`;
   copy the `Rgba8` vector to the existing 256×1 RGBA texture. If `find_palette` returns nullptr,
   fall back to "grayscale".

4. **Migrate RasterLayer, GggsTileLayer, SonarLiveCacheLayer** — update `setColormap` signature
   to `const std::string& name`. Rebuild context menus from
   `marine_colormap::palette_names()`. In `readSettings()`: read stored string; apply
   case-insensitive lowering before passing to `setColormap`; unknown names default to current
   ramp. In `writeSettings()`: write palette name string as-is.

5. **Migrate GridMap (CPU path)** — replace `map::ColorMap colormap_` with
   `std::string colormap_name_{"grayscale"}`. In `processGridMap` / `renderToData`, replace
   `colormap.colorNormalized(t)` with: `auto& pal = *marine_colormap::find_palette(colormap_name_);
   auto c = pal.sample(float(t)); QColor(int(c.r*255+0.5f), …, 255)`. Menu, settings, fallback
   same as step 4.

6. **Delete color_map.{h,cpp}** — remove from `CAMP_MAP_SOURCES` in CMakeLists.txt, delete files.
   Update `test_color_map.cpp` to cover: all 6 palettes opaque in-range, NaN transparent,
   endpoint values, `palette_names()` round-trip, case-insensitive settings fallback. Remove
   old parity test once it passes (its job is done at step 1).

7. **Write ADR-0008** — `docs/decisions/0008-adopt-marine-colormap-lut-bake.md`. Records: chose
   bake-into-existing-LUT over swapping GPU shader; rationale (preserves #134 NaN/NoData/Nearest
   contract, no shader rewrite risk); consequences (TransferParams must be identity at bake time;
   `marine_colormap_response` is NOT used in the shader).

## Files to Change

| File | Change |
|------|--------|
| `package.xml` | Add `<depend>marine_colormap</depend>` |
| `CMakeLists.txt` | find_package + link camp_map; remove `color_map.cpp`; update test |
| `src/camp_map/map/color_map.h` | Delete |
| `src/camp_map/map/color_map.cpp` | Delete |
| `src/camp_map/raster/raster_gl_renderer.h` | `std::string colormap_name_`; update sig |
| `src/camp_map/raster/raster_gl_renderer.cpp` | `ensureLut()` via `bake_lut()` |
| `src/camp_map/raster/gggs_tile_layer.h` | Update `setColormap` sig |
| `src/camp_map/raster/gggs_tile_layer.cpp` | Menu, settings, impl |
| `src/camp_map/ros/live_coverage/sonar_live_cache_layer.h` | Update `setColormap` sig |
| `src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp` | Menu, settings, impl |
| `src/camp_map/raster/raster_layer.h` | Update `setColormap` sig |
| `src/camp_map/raster/raster_layer.cpp` | Menu, settings, impl, default init |
| `src/camp_map/ros/grids/grid_map.h` | `std::string colormap_name_`; update sig |
| `src/camp_map/ros/grids/grid_map.cpp` | CPU sample path via `find_palette` |
| `test/test_color_map.cpp` | Replace with marine_colormap behavior + settings tests |
| `test/test_color_map_parity.cpp` | New: parity baseline (step 1, then retired) |
| `docs/decisions/0008-adopt-marine-colormap-lut-bake.md` | New ADR |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Full 6-palette menu exposed; per-band selection and defaults preserved; settings-key migration is backward-compatible |
| Capture decisions, not just implementations | LUT integration choice recorded in ADR-0008 |
| A change includes its consequences | Parity test, settings migration, and #63 wording reconciliation all in-scope |
| Only what's needed | Widget/range UI deferred to #142; no marine_colormap_response use in shader |
| Test what breaks | Parity test at step 1 locks in render equivalence before deletion |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0007 (RasterFieldSource) | Yes — modifies RasterGlRenderer's LUT path | bake_lut() replaces hand-coded LUT loop; shader contract unchanged |
| ADR-0001 (Adopt ADRs) | Yes — LUT integration is a design decision | New ADR-0008 records choice and rationale |
| ROS 2 conventions (ament) | Yes — new dependency | `<depend>` + `find_package` + `target_link_libraries(camp_map PUBLIC marine_colormap)` |

## Consequences

| If we change... | Also update... | Included? |
|---|---|---|
| `ColorMap::setColormap(Type)` API | All 4 call sites (gggs, sonar, raster_layer, grid_map) | Yes |
| Palette names in settings | Case-insensitive read fallback | Yes |
| camp#63 "camp-internal ColorMap" wording | Reconcile in ADR-0008 + issue comment | Yes (ADR reference) |
| `bake_lut` identity-params contract | Do NOT also apply `marine_colormap_response` in shader | Yes (ADR note) |

## Open Questions

- [ ] No open questions — scope decisions are explicit in the issue; plan is review-plan-ready.

## Estimated Scope

Single PR. ~17 files, all in the `camp` repo. Build + test on host only (container cannot build camp).
