# Plan: WMS GetMap layer support + nowCOAST radar migration

## Issue

https://github.com/rolker/camp/issues/118

## Context

The camp map system supports xyz and WMTS tile sources via `MapTiles` + `CachedTileLoader`.
NOAA nowCOAST radar (`base_reflectivity_mosaic`) and GEBCO bathymetry are served WMS-only
(dynamic `GetMap` with per-request bbox), so they cannot reach the current z/x/y path.

Operator decisions (2026-07-24 checkpoint):
- **Granularity**: per-tile bbox `GetMap` (one WMS request per EPSG:3857 slippy tile),
  reusing the full `MapTiles` lifecycle — cache, #98 eviction, #99/#111 refresh/cache-buster.
- **Time dimension**: latest frame only for v1 — omit TIME parameter; preserve #99 5-min refresh.

`BackgroundManager::createTileLayer()` already has a `"wms"` stub returning `nullptr`.
`TileLayerPreset` already has `layer_id`; GEBCO preset already exists but is `enabled=false`.
`TileLayout::getUrl()` resolves TileMatrix/TileRow/TileCol; it needs a `WMS_BBOX` key that
computes the EPSG:3857 tile bbox from `TileAddress::topLeftCorner()`.

## Approach

1. **Add `WMS_BBOX` key to `TileLayout::getUrl()`** — when key is `"WMS_BBOX"`, compute
   the EPSG:3857 bbox (`minx,miny,maxx,maxy`) from `address.topLeftCorner()` +
   `level.scale * tile_width/height`. Uses `std::fixed` + 2 decimal places (meter precision).

2. **Add `src/camp_map/map_tiles/wms.h` + `wms.cpp`** — `generateWmsLayout(base_url, layer_id)`
   (analog to `osm::generateTileLayout`): builds the same zoom-level grid as OSM but substitutes
   a WMS 1.3.0 GetMap URL template with `WMS_BBOX` as the only variable key.
   URL form: `{base_url}?SERVICE=WMS&VERSION=1.3.0&REQUEST=GetMap&BBOX={WMS_BBOX}&CRS=EPSG:3857&WIDTH=256&HEIGHT=256&FORMAT=image/png&LAYERS={layer_id}&TRANSPARENT=TRUE`

3. **Wire `"wms"` in `BackgroundManager::createTileLayer()`** — construct a `MapTiles` with
   `wms::generateWmsLayout(preset.url.toStdString(), preset.layer_id.toStdString())`.
   `refresh_ms`, cache-busting, and the rest of the MapTiles lifecycle are unchanged.

4. **Update `tile_layer_presets.h`**:
   - Enable GEBCO preset (`enabled = true`; remove disabled note).
   - Replace IEM `nexrad_radar` preset with nowCOAST WMS variant
     (`type="wms"`, `url="https://nowcoast.noaa.gov/geoserver/wms"`,
     `layer_id="base_reflectivity_mosaic"`, same opacity/visible/refresh_ms as before).
   - Update schema comment to note `type` now also supports `"wms"`.

5. **Update camp ADR-0003 addendum** — add `wms` to the `type` values list.

6. **Write camp ADR-0012** (`docs/decisions/0012-wms-getmap-layer-type.md`) — record the
   per-tile bbox granularity decision, the latest-frame-only TIME scope for v1, and
   the nowCOAST/GEBCO provider choices.

7. **Add `test/test_wms_url_generation.cpp`** — pure-logic tests (no network):
   - `WMS_BBOX` key expands to a correctly-formatted `minx,miny,maxx,maxy` string
     for a known zoom-0 tile (verifiable against the OSM tile math).
   - `generateWmsLayout` produces a URL containing all required GetMap parameters.
   - Cache-busting (inherited from `MapTiles::setRefreshInterval`) produces distinct
     URLs across back-to-back refreshes (reuse the pattern from `test_map_tiles_refresh`).

8. **Register test in `CMakeLists.txt`** under `ament_add_gtest(test_wms_url_generation ...)`.

## Files to Change

| File | Change |
|------|--------|
| `src/camp_map/map_tiles/tile_layout.cpp` | Add `WMS_BBOX` key handler in `getUrl()` |
| `src/camp_map/map_tiles/wms.h` | New — `generateWmsLayout()` declaration |
| `src/camp_map/map_tiles/wms.cpp` | New — `generateWmsLayout()` implementation |
| `src/camp_map/background/background_manager.cpp` | Handle `"wms"` in `createTileLayer()` |
| `src/camp_map/background/tile_layer_presets.h` | Enable GEBCO; replace IEM radar preset with nowCOAST WMS |
| `docs/decisions/0003-backgrounds-as-layers-and-depth-tree.md` | Addendum: `type` includes `wms` |
| `docs/decisions/0012-wms-getmap-layer-type.md` | New camp ADR |
| `test/test_wms_url_generation.cpp` | New pure-logic test |
| `CMakeLists.txt` | Add `wms.cpp`; add `test_wms_url_generation` target |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | No TIME UI or WMS GetCapabilities — v1 scope is latest-frame only via fixed URL template |
| A change includes its consequences | Tests, ADR, and ADR-0003 addendum are in scope |
| Test what breaks | URL bbox computation and GetMap URL formation are pure logic — unit-testable without network |
| Improve incrementally | Builds directly on MapTiles/#98/#99/#111 lifecycle with minimal new code |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| camp ADR-0003 (backgrounds persistence) | Yes — new `wms` type variant | Addendum added; schema comment updated |
| camp ADR-0010 (bounded eviction) | Inherited — WMS tiles use same MapTiles lifecycle | No change needed |

## Consequences

| If we change... | Also update... | Included? |
|---|---|---|
| `tile_layout.cpp` (add WMS_BBOX key) | `tile_layout.h` (no public API change needed) | N/A |
| IEM radar preset → nowCOAST WMS | Existing users with persisted `nexrad_radar` as xyz continue to use IEM; re-add for nowCOAST | Accepted — no migration |
| GEBCO enabled | No other presets or ADRs reference GEBCO enablement | Yes |

## Open Questions

- [ ] No open questions — operator checkpoint decisions cover all scope; plan is review-plan-ready.
