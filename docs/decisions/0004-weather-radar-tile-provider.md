# ADR-0004: Weather-radar tile provider (IEM NEXRAD N0Q)

## Status

Accepted

## Context

Issue [#99](https://github.com/rolker/camp/issues/99) adds an auto-refreshing
weather-radar overlay to the camp2 map system. The radar overlay is a network
tile source: it introduces a **new long-lived external dependency** on a
service that the deployed CAMP fetches imagery from at runtime, in the field.
That is an architectural decision worth recording — both for the choice itself
and for its field-operations consequences (what happens when the service is
slow, restructured, or unreachable on a boat with marginal connectivity).

The map system already has all the tile plumbing (`MapTiles` z/x/y + WMTS
layers, `CachedTileLoader`/`CachedFileLoader` HTTP-plus-disk-cache,
`BackgroundManager::createDefaultLayers()` registration). The only open question
for #99 was *which* radar provider to wire in, how its imagery is delivered, and
how its failure modes interact with field use.

**Key constraint discovered during endpoint verification (2026-06-18):** the
`MapTiles` layer consumes *tiled* sources (slippy/XYZ or WMTS — a `{z}/{x}/{y}`
address scheme). NOAA's own radar distribution does **not** provide tiled
radar:

- The legacy ArcGIS service
  (`nowcoast.noaa.gov/arcgis/.../radar_meteo_imagery_nexrad_time/MapServer/WMTS`)
  has been decommissioned — nowCOAST migrated to GeoServer, and the new host
  (`new.nowcoast.noaa.gov`) does not answer.
- The current nowCOAST GeoServer serves radar (`base_reflectivity_mosaic`,
  time-enabled MRMS, EPSG:3857) only via **WMS** (dynamic `GetMap` with an
  arbitrary bbox). Its GeoWebCache WMTS endpoint advertises only non-radar
  layers (bluetopo, stofs3d); the time-varying radar is deliberately not
  tile-cached.

Consuming nowCOAST radar would therefore require building **WMS support** into
CAMP (per-tile EPSG:3857 bbox `GetMap` + time-dimension handling, or a new
dynamic-WMS layer type) — materially more than the tile-plumbing the map system
already has.

Candidate providers considered:

- **NOAA nowCOAST (WMS).** Authoritative MRMS radar direct from NOAA, but
  WMS-only — does not fit the `MapTiles` tile path without new WMS code.
- **Iowa Environmental Mesonet (IEM) NEXRAD N0Q.** NOAA NEXRAD base-reflectivity
  data, redistributed by Iowa State University as **XYZ Web-Mercator
  (EPSG:3857) tiles** — a drop-in for the existing `MapTiles` slippy path. No API
  key, long-standing academic service, ~5-min cadence, CONUS coverage (includes
  the target operating area, Lake Massabesic, NH). The `n0q` product alias
  always serves the latest mosaic.
- **RainViewer / commercial tile APIs.** Global XYZ tiles, but the latest-frame
  path changes each cycle (requires fetching a JSON index before each refresh),
  and the data is third-party rather than NOAA-origin.

## Decision

Use **IEM NEXRAD N0Q** as the radar tile provider for the #99 overlay, wired as
an XYZ (slippy) `MapTiles` layer via `osm::generateTileLayout()` — the same path
as the OSM/OpenSeaMap basemap layers, *not* the WMTS path used for the NOAA
charts layer.

Endpoint (verified live 2026-06-18 — returns 256×256 `image/png` tiles in
EPSG:3857, NH-area tile confirmed):

```
https://mesonet.agron.iastate.edu/cache/tile.py/1.0.0/nexrad-n0q-900913/{z}/{x}/{y}.png
```

Rationale:

- **NOAA-origin data.** The imagery is NOAA NEXRAD base reflectivity; IEM is the
  redistributor, not the data author. Authoritative for operational use.
- **Fits the existing tile path.** XYZ Web-Mercator tiles drop straight into
  `MapTiles`/`generateTileLayout()` — no new protocol code, unlike nowCOAST WMS.
- **No API key.** Nothing to provision, rotate, or risk committing — consistent
  with the keyless OSM/NOAA-charts layers and the no-secrets workspace rule.
- **Always-latest frame.** The `nexrad-n0q` alias serves the most recent mosaic
  (no timestamp pinning), so the 5-minute refresh genuinely fetches fresh
  imagery (see Consequences).
- **US coverage** includes the target operating area.
- **Graceful offline degradation.** On any fetch error (DNS failure, timeout,
  404, offline), `CachedFileLoader::downloadFinished()` logs and drops the reply
  *without* emitting `dataLoaded`, so the tile's pixmap is never set — the tile
  stays blank. No crash, no UI block. The radar layer also ships **default-OFF**
  at ~0.65 opacity, so a missing or stale radar layer cannot degrade the
  operator's view of the basemap or chart.

## Consequences

- A new runtime network dependency exists (IEM, `mesonet.agron.iastate.edu`). If
  IEM changes URLs or goes down, the radar layer shows blank tiles; everything
  else is unaffected (default-OFF, independent layer). This ADR is the traceable
  record if the provider must be swapped (e.g. back to a NOAA-direct WMS feed if
  CAMP ever grows WMS support).
- The radar layer refreshes every ~5 minutes (`setRefreshInterval`, #99 Phase 2),
  re-fetching from IEM each cycle. Cadence is hardcoded for now; exposing it as a
  user setting is a possible follow-up.
- **Refresh yields a genuinely fresh frame** because `nexrad-n0q` always resolves
  to the latest mosaic. The disk-cache invalidation in `onRefreshTimer`
  (`map_tiles.cpp`) is what forces the network re-fetch — without it the on-disk
  PNGs would re-serve the previous frame for the same `z/x/y`. A future
  timestamp-pinned provider would break this assumption (static overlay); that
  caveat is noted at `onRefreshTimer`.
- **Enabling radar reintroduces #98-class within-cycle tile accumulation — on a
  second layer.** The refresh path bounds memory only AT each refresh boundary
  (`setLayout` resets the tile set); within a cycle, `paint()` still only hides
  (never deletes) tiles as the operator pans/zooms, exactly as the existing
  layers do. Default-OFF makes this *conditional* — it does not eliminate the #98
  leak, it just keeps the radar layer from contributing unless an operator turns
  it on. With radar enabled, an operator who pans/zooms heavily before a refresh
  accumulates tiles on the radar layer in addition to the basemap/chart layers.
  The bounded-memory test guards the refresh-boundary reset, not the within-cycle
  growth.
- Disk caching applies per-layer under
  `~/.CCOMAutonomousMissionPlanner/map_tiles/nexrad_radar/`; the refresh path
  invalidates that subdir each cycle so stale radar PNGs are not re-served.

## Alternatives considered

- **NOAA nowCOAST (WMS), direct.** Rejected for #99: radar is WMS-only, which
  does not fit the `MapTiles` tile path. Adopting it would mean building a WMS
  `GetMap` layer type (per-tile bbox + time dimension) — a larger feature than
  this overlay warrants. Revisit if CAMP grows general WMS support or if
  direct-from-NOAA provenance becomes a hard requirement.
- **RainViewer / commercial radar tiles.** Rejected as the default: the
  latest-frame path changes each cycle (extra JSON-index fetch before every
  refresh) and the data is third-party rather than NOAA-origin. Can be revisited
  if non-US coverage is ever needed.
- **No radar overlay (status quo).** Rejected: situational weather awareness on
  open water is operationally valuable, and the map system already had every
  piece needed except a periodic refresh.

## References

- Issue [#99](https://github.com/rolker/camp/issues/99) — the radar overlay
- [ADR-0002](0002-web-mercator-scene-and-layer-model.md) — Web-Mercator scene +
  library split (the radar layer lives in pure-Qt `libcamp_map`)
- [ADR-0003](0003-backgrounds-as-layers-and-depth-tree.md) — backgrounds as
  stackable layers (the radar is one such stacked, toggleable layer)
- [#98](https://github.com/rolker/camp/issues/98) — tile-lifecycle OOM; the
  refresh path's bounded-memory test (`test/test_map_tiles_refresh.cpp`) is the
  coordination gate (the radar is a QPixmap `MapTiles` layer, NOT the #96 GDAL
  raster path)
- IEM NEXRAD tile service:
  `https://mesonet.agron.iastate.edu/cache/tile.py/1.0.0/nexrad-n0q-900913/{z}/{x}/{y}.png`
  (NOAA NEXRAD base reflectivity, EPSG:3857, ~5-min cadence; verified 2026-06-18)
- `src/camp2/background/background_manager.cpp` — registration point
- `src/camp2/util/cached_file_loader.cpp` — the graceful-degradation fetch path
