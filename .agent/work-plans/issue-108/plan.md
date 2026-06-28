# Plan: Band Selection for GggsTileLayer (#108)

## Issue

https://github.com/rolker/camp/issues/108

## Context

`GggsTile` reads `GetRasterCount()` in the constructor (for the `< 1` guard) but
never stores it, and `loadPixels()` hardwires `GetRasterBand(1)`. Multi-band
GeoTIFFs (bathy: depth + uncertainty; backscatter: intensity + quality) are
treated as single-band only. The operator needs to pick which band a flat
`GggsTileLayer` renders, with the existing colormap LUT + context-menu pattern
(camp#90) already in place for range + ramp.

## Approach

1. **`GggsTile`: store band count and selected band** — In the constructor, store
   `band_count_ = dataset->GetRasterCount()` instead of just checking `< 1`.
   Add `int band_ = 1` member. Expose `bandCount()` getter. Add `setBand(int)`:
   sets `band_`, clears `data_`, resets `data_min_/data_max_` to crossed sentinel,
   stores `pixels_loaded_ = false` — GL texture is NOT touched here (that is the
   layer's responsibility). Modify `loadPixels()` to call
   `dataset->GetRasterBand(band_)` instead of the hardcoded `1`.

2. **`GggsTileLayer`: layer-level band state** — Add `int band_ = 1` member, plus
   `band()`, `bandCount()` (delegates to the first valid tile), and
   `setBand(int band)`. `setBand`: validate against `bandCount()`, abort+join any
   in-flight load, make GL context current and release all tile textures (reusing
   the existing per-tile `releaseGL()` pattern so only textures are reset, not
   shaders/FBO/LUT), call `tile->setBand(band)` for each tile, reset the layer
   range to crossed, invalidate `cached_image_`, re-kick `loadTiles()`, and call
   `update(boundingRect())`.

3. **Context menu: "Band" submenu** — In `contextMenu()`, add a "Band" submenu
   when `bandCount() > 1`. One `QAction` per band (1-indexed label), checkable,
   checked when equal to `band_`, connects to `setBand`. When
   `bandCount() == 1`, no submenu added (no visual noise for common single-band
   tile-sets).

4. **Persistence** — In `readSettings()`/`writeSettings()`, persist the selected
   band under the existing `MapItem/itemID()` settings group as key `"band"`.
   `readSettings()` reads with a default of 1; calls `setBand()` only if the
   persisted value differs (skips the abort+reload on the common no-change path).

5. **Tests** —
   - `test_gggs_tile.cpp`: write a 2-band GeoTIFF (different constant values per
     band); assert `bandCount() == 2`; load band 1, verify range; call
     `setBand(2)`, load pixels again, verify distinct range.
   - `test_gggs_band_select.cpp` (new): write a 2-band tile-set directory; create
     a `GggsTileLayer`, `waitForLoad()`, assert `bandCount() == 2`; call
     `setBand(2)`, `waitForLoad()`, assert the layer's `data_min_`/`data_max_`
     shifted. Skips (not fails) on environments without offscreen GL, using the
     same `offscreenGLAvailable()` guard as `test_gggs_render.cpp`.

## Files to Change

| File | Change |
|------|--------|
| `src/camp2/raster/gggs_tile.h` | Add `band_count_`, `band_`, `bandCount()`, `setBand(int)` |
| `src/camp2/raster/gggs_tile.cpp` | Store band count in ctor; use `band_` in `loadPixels()`; implement `setBand` |
| `src/camp2/raster/gggs_tile_layer.h` | Add `band_`, `band()`, `bandCount()`, `setBand(int)` |
| `src/camp2/raster/gggs_tile_layer.cpp` | Implement `setBand`, update `contextMenu`, `readSettings`/`writeSettings` |
| `test/test_gggs_tile.cpp` | Add multi-band construction + band-switch tests |
| `test/test_gggs_band_select.cpp` | New: layer-level band selection + reload test |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | No band-semantics metadata (deferred per #104); no per-band range normalization; band count from first tile only |
| A change includes its consequences | Settings persistence updated; tests cover both tile and layer levels |
| Improve incrementally | Single PR; no shader changes; reuses existing colormap/context-menu pattern |
| Human control and transparency | Band picker visible only when `bandCount() > 1`; persisted so session round-trips |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| camp ADR-0005 | Yes — extends flat `GggsTileLayer` | `setBand` operates on the existing flat layer; no store/catalog machinery changes |
| camp ADR-0002 | Informational | No new layer types; no scene/model structure changes |
| workspace ADR-0001 | No new design decision | Reusing camp#90's pattern; no novel ADR needed |

## Consequences

| If we change... | Also update... | Included? |
|---|---|---|
| `GggsTile::loadPixels()` signature (band param via member) | `test_gggs_tile.cpp` existing tests (band 1 default, no change needed) | Yes — default band stays 1 |
| `GggsTileLayer::setBand` (new public API) | `test_gggs_band_select.cpp` tests new method | Yes |
| `writeSettings()` adds "band" key | `readSettings()` reads it with default 1 | Yes |

## Open Questions

- None — band semantics (depth vs. uncertainty labels) are explicitly deferred per
  #104; operator identifies bands by 1-based index only.

## Estimated Scope

Single PR.
