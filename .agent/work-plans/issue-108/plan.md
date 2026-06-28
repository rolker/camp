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
   `dataset->GetRasterBand(band_)` instead of the hardcoded `1`. **Per-band NoData
   (review finding):** `loadPixels()` re-queries `GetNoDataValue()` from the band
   it actually reads (rather than reusing the ctor's band-1 value), so a switched
   band's range filter is correct. Done in `loadPixels()` — which already has the
   dataset/band open — rather than in `setBand()`, which does not open the dataset.

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
   `readSettings()` reads with a default of 1 and applies it only if the persisted
   value differs (skips the abort+reload on the common no-change path).
   *(camp#108 local-review refinement)* The band switch lives in a private,
   non-persisting `applyBand(int)` helper: `setBand()` calls `applyBand()` then
   `writeSettings()`, while `readSettings()` calls `applyBand()` directly so the
   read path is not coupled to a settings write — mirroring the inline colormap
   apply. `applyBand()` also skips (and WARNs once per switch on) any tile whose
   own `bandCount()` is below the requested band: such a tile keeps its prior band
   — its GL texture and pixels are left intact so it keeps rendering normally — and
   `tilesReady()` excludes it from the range fold (folding only tiles whose
   `band() == band_`) so its stale prior-band range can't pollute the new band's
   auto-range. *(camp#108 local-review round-2 refinement.)*

   *(camp#108 local-review round-3 fix — rescan interaction.)* The band switch in
   step 2/this step only re-points the tiles held at switch time; a tile that lands
   AFTER a switch and is picked up by `rescan()` (camp#102/#104) is freshly
   constructed at the default band 1. `rescan()`'s add loop therefore calls
   `tile->setBand(band_)` on each new tile so it inherits the layer's current band —
   otherwise a rescan after a band > 1 switch would render the wrong band scaled
   through the selected band's range and be permanently excluded from the
   `tilesReady()` fold (which folds only `band() == band_`). `setBand(1)` on a
   band-1 default is a no-op, so the common single-band path is unchanged.

5. **Tests** —
   - `test_gggs_tile.cpp`: write a 2-band GeoTIFF (different constant values per
     band); assert `bandCount() == 2`; load band 1, verify range; call
     `setBand(2)`, load pixels again, verify distinct range.
   - `test_gggs_band_select.cpp` (new): write a 2-band tile-set directory; create
     a `GggsTileLayer`, `waitForLoad()`, assert `bandCount() == 2`; call
     `setBand(2)`, `waitForLoad()`, assert the layer's `data_min_`/`data_max_`
     shifted. Skips (not fails) on environments without offscreen GL, using the
     same `offscreenGLAvailable()` guard as `test_gggs_render.cpp`.
   - `test_gggs_band_select.cpp` (round-3 add): `RescanInheritsSelectedBand` —
     switch a 2-band layer to band 2, then `rescan()` in a newly-landed tile and
     assert (via the GL-free `tileBands()` test seam) that EVERY tile, including the
     rescanned one, reads band 2. The per-tile band assertion needs no GL, so this
     test RUNS (not SKIPs) in-container and catches the must-fix; the render
     confirmation is `offscreenGLAvailable()`-guarded.
   - `test_gggs_persistence.cpp` (round-4 add): `BandRoundTrips` —
     `setBand(2)` on a 2-band tile-set → `writeSettings()` → a fresh layer over the
     same directory `readSettings()` restores band 2; `BandDefaultRoundTrips` — a
     layer that never sets a band writes/reads back band 1 (no spurious change), and
     reading a cleared group defaults to 1. The band integer round-trip is GL-free
     (a `TestableGggsTileLayer` exposes the protected hooks), so both tests RUN — not
     SKIP — in-container.

## Files to Change

| File | Change |
|------|--------|
| `src/camp2/raster/gggs_tile.h` | Add `band_count_`, `band_`, `bandCount()`, `setBand(int)` |
| `src/camp2/raster/gggs_tile.cpp` | Store band count in ctor; use `band_` in `loadPixels()` (re-queries NoData there); implement `setBand` |
| `src/camp2/raster/gggs_tile_layer.h` | Add `band_`, `band()`, `bandCount()`, `setBand(int)`, private `applyBand(int)`, test-only `tileBands()` |
| `src/camp2/raster/gggs_tile_layer.cpp` | Implement `applyBand`/`setBand` (read path non-persisting), update `contextMenu`, `readSettings`/`writeSettings`; `rescan()` propagates `band_` to new tiles; `applyBand()` marks GL failed on a makeCurrent failure |
| `test/test_gggs_tile.cpp` | Add multi-band construction + band-switch tests |
| `test/test_gggs_band_select.cpp` | New: layer-level band selection + reload test; rescan-after-switch band-inheritance regression test |
| `test/test_gggs_persistence.cpp` | Add band QSettings round-trip tests (`BandRoundTrips`, `BandDefaultRoundTrips`); GL-free, RUN in-container |

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

- Band semantics (depth vs. uncertainty labels) are explicitly deferred per #104;
  operator identifies bands by 1-based index only.
- **Shader `v <= 0.0` discard (camp#108 review finding).** The fragment shader
  (`gggs_tile_layer.cpp` `kFragmentShader`) discards `v <= 0.0`, relying on the
  mosaicker's convention that NoData is 0 and real returns are floored to >= 1.
  This holds for the depth/sidescan band (band 1). A switched band whose valid
  samples can legitimately be 0 or negative (e.g. an uncertainty band, or a
  signed-offset band) will have those samples discarded and the remainder
  mis-ranged — it renders blank or wrongly scaled. A correct fix distinguishes
  real NoData from valid 0/negative samples by plumbing the per-band NoData value
  into the shader (a uniform + sentinel test) instead of the hardcoded `<= 0.0`.
  Deferred: this slice does NOT rework the shader. The limitation is documented in
  a code comment beside the discard and acknowledged here. Per-band NoData is now
  re-queried on the CPU side (`GggsTile::loadPixels`), so the CPU auto-range is
  already correct for the selected band; only the GPU discard threshold is
  band-1-specific.

## Estimated Scope

Single PR.
