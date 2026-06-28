# ADR-0006: Live tile cache — persistence contract + opt-in tile-stream subscription

## Status

Accepted

Implements CAMP issue #121 Part B (the live tile cache). Part A (the generic
`RasterFieldSource` render abstraction) is deferred to camp#134. Builds on
[ADR-0001](0001-topicbridge-and-executor-contract.md) (callback/executor
threading), [ADR-0002](0002-web-mercator-scene-and-layer-model.md) (the
Web-Mercator scene + layer model) and [ADR-0005](0005-stores-browser-and-flat-display-layers.md)
(browse/compose split). Consumes the boat-side anti-entropy transport
(uma ADR-0008: `SonarVisualizationTile` / `TileCatalog` / `TileRequest`) and the
payload-agnostic reconciler in `marine_tiled_raster_store`.

## Context

The boat publishes a live, GGGS-tiled sonar **display product**: per-tile,
per-band quantized rasters (`SonarVisualizationTile`, best-effort), a complete
periodic `TileCatalog` (transient-local + reliable), and accepts a `TileRequest`
"need" list (reliable). CAMP had no consumer: no live subscriber, no anti-entropy
reconciler, and no place to render incoming tiles. The gap is to land those tiles
in the existing GGGS GL render path **without** re-introducing the read/write race
a write-through-then-rescan-from-disk design would create.

Three forces shape the contract:

1. **Display-grade preview, not data of record.** These tiles are a lossy,
   quantized projection of the durable boat-side stores (bathy/backscatter). They
   carry no provenance and are always rebuildable from the stores. Losing the
   cache is *not* losing survey data — which is what licenses an in-memory,
   crash-safe-only persistence model rather than a durable store.

2. **Bandwidth (#71, slow cell links).** The large payload is `coverage_tiles`
   (best-effort, the full tile stream). The catalog is small and cheap. An
   operator on a constrained link must not pay for a tile stream they did not ask
   for, just because CAMP happened to see the topic appear.

3. **Single-thread reconciler (`TileCatalogReconciler` is not thread-safe).** All
   reconcile/markHave/drop/applyPatch mutation must happen on one thread.

## Decision

### D1 — In-memory render; disk is a crash-safe cache only

Received tiles are dequantized to in-memory Float32 (`SonarLiveTile`, one
`SonarLiveBand` per `VisualizationBand`) and rendered straight from memory through
a GL pipeline duplicated from `GggsTileLayer` — **no GDAL read on the hot path**.
The in-memory map is authoritative; the layer never reads back from disk during
operation. Write-through to disk exists only so a warm restart can repopulate
without waiting for a full re-push.

### D2 — Write-through: atomic temp+rename, all received bands

On each applied patch the affected tile is serialized off the GUI thread to a
Float32 GeoTIFF under
`<cache_dir>/<source_ns>/<level>_<row>_<col>.tif`, written to a `.tif.tmp`
sibling and `rename()`d into place (atomic on POSIX) so a crash mid-write can
never leave a half-tile a later warm-load would choke on. The georeferencing is
the GGGS north-up geotransform for the tile's level (the same convention
`marine_tiled_raster_store::saveTile` and `GggsTile` use), so a cached tile is a
valid GGGS GeoTIFF. **All received bands are written** (one Float32 GeoTIFF band
per band, band name carried as the GDAL band description), so a warm-load can
reconstruct whichever band the operator later selects — not just depth. (Operator
secondary-question default: write-through all bands.)

`cache_dir` defaults to `QStandardPaths::AppDataLocation + "/live_tile_cache"` and
is overridable via `QSettings LiveTileCache/cache_dir` (follows #117: a default,
never a hardcode the operator can't change).

### D3 — Warm-load before subscribing

When a source is (re)activated the layer reads any existing
`<level>_<row>_<col>.tif` tiles from its cache directory (GDAL, dequantized
Float32, all bands + band names recovered from band descriptions; GridIndex
recovered from the geotransform exactly as `marine_tiled_raster_store::loadTile`
does) into the in-memory map, and seeds the reconciler with `markHave(index, 0)`
for each. Version 0 means "I have *something* here" — strictly older than any real
catalog version, so the next catalog re-requests the tile if the boat has a newer
version, and the prune gate never deletes it spuriously (a held version of 0 is
older than any non-zero `generation_time`). This is what closes the
**downtime gap**: a CAMP that was off while coverage changed converges on
reconnect — requesting tiles it is missing/stale on, pruning tiles the boat no
longer holds.

### D4 — Anti-entropy via the reconciler, single-threaded

`TileCatalogReconciler` drives convergence. On each `TileCatalog`:
`reconcile()` → `{to_request, to_prune}`; publish a `TileRequest` for
`to_request`; for each `to_prune` delete the in-memory tile + its disk file +
`drop()`. On each `SonarVisualizationTile`: newest-wins (discard if the tile's
version is not newer than the held version), `applyPatch()`, `markHave()`,
schedule write-through, repaint.

**Threading invariant (ADR-0001).** The reconciler and tile map are **not**
thread-safe and are touched only on the GUI thread. ROS subscription callbacks do
the minimum — copy the message and emit a **queued** Qt signal — and never touch
`reconciler_` or the tiles directly. The GUI-thread slot does all reconcile /
markHave / drop / applyPatch / write-through-scheduling. Warm-load runs on the GUI
thread (at activation) before the subscriptions are created, so it cannot race a
callback.

**Version semantics — one int64.** Every version is **nanoseconds since epoch**
(`int64`, `marine_tiled_raster_store::TileVersion`). A tile's version is its
`header.stamp` flattened to ns; a catalog entry's version is its
`builtin_interfaces/Time` flattened to ns; the prune gate compares held versions
against the catalog `header.stamp` flattened to ns. The newest-wins check, the
`markHave` value, and the prune timestamp-gate are therefore all the same unit and
directly comparable. The single monotonic source clock is the boat (uma ADR-0008).

### D5 — Discovery is automatic; tile-stream subscription is **opt-in**

`SonarLiveCacheManager` auto-detects the `SonarVisualizationTile` topic (the
`GridManager` discovery pattern) and spawns a `SonarLiveCacheLayer` in a
**discovered-but-inactive** state. The inactive layer MAY subscribe to the small
`coverage_catalog` (transient-local, cheap) to populate availability, but it does
**NOT** subscribe to `coverage_tiles` and does **NOT** publish `TileRequest` until
the operator enables it. The layer shows in the Layers tree as
`Live Coverage [<source>]` with an `(available)` / `(live)` status and a
context-menu **"Enable live coverage"** toggle. Enabling subscribes to
`coverage_tiles`, warm-loads the disk cache, and starts the reconciler
(request/prune). Disabling unsubscribes from the tile stream (the catalog
subscription may remain). The per-source enabled/disabled state persists via
`QSettings` keyed on the source namespace, so an **enabled** source re-subscribes
on warm restart while a **never-enabled** one stays passive.

This is a conscious tension with **ADR-0005 §2**, which anticipated a *generic*
`CatalogSource` seam for topic-discovery consumers (camp#44/#68). That seam is not
yet built, so this PR discovers directly (the `GridManager` pattern) but gates the
expensive subscription behind an explicit operator action — serving #71 (no
surprise bandwidth on a slow link) without waiting on #44/#68. When the generic
`CatalogSource` lands, this manager should migrate onto it; the opt-in gate is the
behaviour to preserve through that migration.

### D6 — Operator presentation (ADR-0005 browse/compose)

The live layer appears in the **Layers tree** (compose side), not the Stores
catalog browser — it is ROS-discovered, not file-browsed. Its display name carries
the source and its `(available)`/`(live)` state so the operator can tell a passive
discovered source from an active one. Selected band + colormap persist per source
(same controls as `GggsTileLayer`).

## Consequences

- `camp_map_ros` gains a dependency on `marine_tiled_raster_store` (the
  reconciler) — added to `CMakeLists.txt` and `package.xml`.
- A new `map::SonarLiveCacheLayerType` enum value in `map/item_types.h` for
  `qgraphicsitem_cast`.
- The GL render path is **duplicated** from `GggsTileLayer` (shader constants,
  FBO, LUT) rather than shared. The duplication is explicit and temporary, marked
  with a `// NOTE: shader duplicated; unify via RasterFieldSource camp#134`
  comment. Unifying it is the whole point of Part A / camp#134.
- The new translation units live under `ros/live_coverage/` (not `raster/`):
  they depend on `marine_interfaces` + `marine_tiled_raster_store`, so they belong
  in the ROS-dependent `camp_map_ros` library — `camp_map` stays ROS-free, the
  invariant ADR-0002 / the CMake layering relies on. (This refines the plan's
  `raster/` placement for layering integrity.)
- No eviction beyond prune-on-absence. A long survey could accumulate many tiles;
  eviction-by-area is a deferred follow-up (`// TODO(camp): eviction-by-area`),
  not implemented here (operator secondary-question default).

## Alternatives considered

- **Write-through then rescan from disk** (reuse `GggsTileLayer::rescan()`
  wholesale): rejected — it re-reads tiles the writer is still flushing, the exact
  read/write race the in-memory-authoritative model avoids, and it pays a GDAL
  read on every live update.
- **Auto-subscribe on discovery** (the pure `GridManager` pattern, as the plan
  originally proposed): rejected for the tile stream — it spends bandwidth on a
  best-effort full-tile stream the operator never asked for (#71). Discovery stays
  automatic; only the tile-stream subscription is gated. See D5.
- **Durable store instead of crash-safe cache**: rejected — the tiles are a
  rebuildable preview, not data of record (D1); a durable store would imply a
  provenance/lifecycle contract these display tiles deliberately don't carry.
