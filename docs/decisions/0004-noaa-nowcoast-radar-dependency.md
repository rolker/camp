# ADR-0004: NOAA nowCOAST as the weather-radar tile provider

## Status

Accepted

## Context

Issue [#99](https://github.com/rolker/camp/issues/99) adds an auto-refreshing
weather-radar overlay to the camp2 map system. The radar overlay is a network
tile source: it introduces a **new long-lived external dependency** on a
third-party service that the deployed CAMP fetches imagery from at runtime, in
the field. That is an architectural decision worth recording — both for the
choice itself and for its field-operations consequences (what happens when the
service is slow, restructured, or unreachable on a boat with marginal
connectivity).

The map system already has all the tile plumbing (`MapTiles` WMTS layers,
`CachedTileLoader`/`CachedFileLoader` HTTP-plus-disk-cache,
`BackgroundManager::createDefaultLayers()` registration). The only open question
for #99 was *which* radar provider to wire in, and how its failure modes
interact with field use.

Candidate providers considered:

- **NOAA nowCOAST** (NEXRAD base-reflectivity, MRMS) — US government, authoritative,
  no API key, ArcGIS-hosted WMTS, ~2-5 min update cadence, permissive terms for
  operational use, covers the target operating area (Lake Massabesic, NH).
- **RainViewer / commercial tile APIs** — global coverage and friendly tiling,
  but require an API key (a credential to manage and not commit) and carry
  rate-limit / commercial-ToS constraints less suited to an open-source,
  government-adjacent autonomy stack.

## Decision

Use **NOAA nowCOAST** as the radar tile provider for the #99 overlay.

Provisional WMTS endpoint (mirrors the structure of the existing NOAA charts
WMTS block in `background_manager.cpp`):

```
https://nowcoast.noaa.gov/arcgis/rest/services/nowcoast/radar_meteo_imagery_nexrad_time/MapServer/WMTS
```

Rationale:

- **Authoritative source.** NOAA is the authoritative US weather-radar source;
  no quality/licensing ambiguity for operational use.
- **No API key.** Nothing to provision, rotate, or risk committing — consistent
  with the existing keyless NOAA charts layer and the no-secrets workspace rule.
- **US coverage** includes the target operating area.
- **Permissive ToS** for operational/derived use.
- **Graceful offline degradation.** On any fetch error (DNS failure, timeout,
  404, offline), `CachedFileLoader::downloadFinished()` logs and drops the reply
  *without* emitting `dataLoaded`, so the tile's pixmap is simply never set — the
  tile stays blank. No crash, no UI block. The radar layer also ships
  **default-OFF** and at ~0.65 opacity, so a missing or stale radar layer cannot
  degrade the operator's view of the basemap or chart.

The endpoint is marked **provisional / TODO-confirm** in the code: NOAA
periodically restructures its ArcGIS service paths, and the URL was not verified
live at implementation time. A 404 degrades gracefully (blank layer), so landing
with an unverified-or-TODO endpoint is acceptable; confirming it against live
nowCOAST docs is a follow-up.

## Consequences

- A new runtime network dependency exists. If nowCOAST changes URLs or goes down,
  the radar layer shows blank tiles; everything else is unaffected (default-OFF,
  independent layer). This ADR is the traceable record if the provider must be
  swapped.
- The radar layer refreshes every ~5 minutes (`setRefreshInterval`, #99 Phase 2),
  re-fetching from nowCOAST each cycle. Cadence is hardcoded for now; exposing it
  as a user setting is a possible follow-up.
- Disk caching applies per-layer under
  `~/.CCOMAutonomousMissionPlanner/map_tiles/NOAA_radar/`; the refresh path
  invalidates that subdir each cycle so stale radar PNGs are not re-served.
- The endpoint must be confirmed against live NOAA nowCOAST docs before this is
  relied on operationally (tracked as a TODO in `background_manager.cpp`).

## Alternatives considered

- **RainViewer / commercial radar tiles.** Rejected as the default: API-key
  management and commercial rate limits add operational friction with no benefit
  over an authoritative, keyless government source for the US operating area. Can
  be revisited if non-US coverage or a denser cadence is ever needed.
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
- `src/camp2/background/background_manager.cpp` — registration point
- `src/camp2/util/cached_file_loader.cpp` — the graceful-degradation fetch path
