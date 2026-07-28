# ADR-0012: WMS GetMap layers ride the per-tile MapTiles path

## Status

Accepted ([#118](https://github.com/rolker/camp/issues/118), 2026-07-24)

## Context

Some authoritative sources publish imagery **only** as WMS `GetMap` (dynamic
bbox render), never as tiles: NOAA nowCOAST serves its radar
(`weather_radar:base_reflectivity_mosaic`, MRMS-based, time-enabled) exclusively via WMS, and
GEBCO's global bathymetry is WMS-only (`wms.gebco.net/mapserv`,
`GEBCO_LATEST`). Until now camp's map system consumed only tiled sources
(slippy z/x/y and WMTS), which forced the [#99](https://github.com/rolker/camp/issues/99)
radar overlay onto IEM's third-party XYZ redistribution — a provenance and
freshness stopgap ([#111](https://github.com/rolker/camp/issues/111)) — and
left the GEBCO preset inert in the [#117](https://github.com/rolker/camp/issues/117)
preset table.

Two shapes were considered for WMS support:

1. **Per-tile bbox `GetMap`** — synthesize the Web-Mercator slippy grid and
   issue one `GetMap` per tile bbox.
2. **Full-viewport `GetMap`** — one dynamic request covering the current view.

## Decision

**Per-tile bbox `GetMap` on the OSM grid** (operator decision 2026-07-24).
`wms::generateWmsLayout()` builds the *identical* zoom-level grid as
`osm::generateTileLayout()` and swaps only the URL template, using a
`WMS_BBOX` variable key that `TileLayout::getUrl()` expands to the tile's
EPSG:3857 `minx,miny,maxx,maxy`. A WMS layer is therefore an ordinary
`MapTiles` layer and inherits, unchanged:

- the disk tile cache (`CachedTileLoader`),
- bounded LRU eviction ([#98](https://github.com/rolker/camp/issues/98)),
- periodic refresh + cache-busting
  ([#99](https://github.com/rolker/camp/issues/99) /
  [#111](https://github.com/rolker/camp/issues/111); the buster's `&` branch
  already handles URLs with a query string),
- blank-tile graceful degradation on fetch failure (`CachedFileLoader` — no
  crash, no UI block; the field-dependency posture for nowCOAST matches the
  IEM layer it replaces).

A full-viewport layer would have needed a new fetch/cache/refresh lifecycle
and refetched on every pan/zoom — heavier on marginal field links.

### Scope and hardcoded parameters

- **`VERSION=1.3.0` and `CRS=EPSG:3857` are hardcoded**, not schema fields:
  the per-tile path only works against the Web-Mercator tile grid, so the CRS
  is structural rather than configurable; 1.3.0 is the current WMS spec and
  both target endpoints accept it (verified live 2026-07-24). Axis order for
  EPSG:3857 in 1.3.0 is easting,northing — the natural x,y.
- **`WIDTH`/`HEIGHT` derive from `osm::tile_size`**, the same constant the
  bbox math uses, so raster size and bbox cannot drift apart.
- **Latest-frame only for time-enabled layers (v1)**: `TIME` is omitted, so
  the server returns the newest frame; the radar preset keeps its #99
  semantics (default-off, 0.65 opacity, 5-minute refresh). Frame
  navigation/animation is out of scope until a concrete need appears.
- The radar preset keeps the name `nexrad_radar` for persisted-state
  continuity (`BackgroundTileLayers/ids` + `MapItem/<settingsKey>`), although
  the MRMS mosaic is broader than NEXRAD proper.
- The Add-tile-layer dialog's **Custom** entry still offers xyz/wmts only; a
  custom WMS source would also need a `LAYERS` field — add it when a source
  outside the preset table actually needs it.

## Consequences

- The IEM stopgap is retired; ADR-0003's addendum gains `wms` in the `type`
  list, and the #117 GEBCO preset becomes live.
- WMS servers are asked for 256-px tiles at slippy scales; time-varying layers
  re-render per refresh exactly as the XYZ radar did.
- Live smoke check (2026-07-24): GEBCO `GetMap` returns a 117 KB PNG for the
  zoom-1 NE-quadrant bbox; nowCOAST requires the **workspace-qualified** layer
  name `weather_radar:base_reflectivity_mosaic` (the bare mosaic name returns
  `LayerNotDefined`) and then returns a 256×256 PNG.
- Pure-logic URL tests cover bbox expansion and template assembly
  (`test_wms_url_generation`); live-endpoint behavior is smoke-checked
  manually at implementation/review time (recorded in the PR), since CI must
  not depend on external services.

## References

- [#118](https://github.com/rolker/camp/issues/118) — this work
- [#117](https://github.com/rolker/camp/issues/117) / PR
  [#175](https://github.com/rolker/camp/pull/175) — preset table + persistence
  this builds on
- [#99](https://github.com/rolker/camp/issues/99),
  [#111](https://github.com/rolker/camp/issues/111) — radar overlay history
- [ADR-0003](0003-backgrounds-as-layers-and-depth-tree.md) — background
  persistence (addendum: `BackgroundTileLayers` schema)
