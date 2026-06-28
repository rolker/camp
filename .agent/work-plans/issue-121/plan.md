# Plan: CAMP Live Tile Cache (Part B — anti-entropy, in-memory render)

## Issue

https://github.com/rolker/camp/issues/121

## Context

**Scope (operator-decided): Part B only.** Part A (RasterFieldSource abstraction)
is deferred to camp#134.

The transport is fully in place: `marine_interfaces` has `SonarVisualizationTile`,
`TileCatalog`, `TileCatalogEntry`, `TileIndex`, `TileRequest`, and
`VisualizationBand`; `marine_tiled_raster_store` has `TileCatalogBuilder` and
`TileCatalogReconciler` (merged via uma#230/PR#234–235); the boat-side producer
`cube_bathymetry` publishes on `~/coverage_tiles` (best_effort), `~/coverage_catalog`
(transient_local+reliable), and subscribes to `~/coverage_requests` (reliable)
(merged via cube#78).

Current CAMP state (post #108/#122/PR#124/PR#133):
- `src/camp_map/raster/gggs_tile_layer.{h,cpp}` — multi-band, colormap LUT,
  per-band NoData discard, Nearest sampling, per-layer QSettings persistence keyed
  by directory.
- `src/camp_map/raster/gggs_tile.{h,cpp}` — GDAL-backed tile; `loadPixels()` reads
  one band as Float32 into `data_` (off-thread safe); `texture()` uploads to GL.
- Rendering: offscreen FBO + vertex/fragment shader (GL 2.x); auto-range + colormap
  LUT; NoData discard in shader.

The gap: CAMP has no live tile subscriber or anti-entropy reconciler. The live cache
must render through the existing GL path without re-introducing the read/write race
that write-through-then-rescan would create.

## Approach

### Step 1 — Camp ADR-0006: Live Tile Cache Persistence Contract

Write `docs/decisions/0006-live-tile-cache-persistence.md`. Captures the
design decisions the review-issue entry flagged as needing an ADR:
- Display-grade preview vs. data of record (lossy/quantized; losing it ≠ losing
  survey data).
- In-memory render (no GDAL read on the hot path after warm-load).
- Write-through to disk: atomic temp+rename for crash safety only.
- Warm-load: on startup, read the disk cache dir using `GggsTile::loadPixels()`
  (GDAL reads dequantized Float32 GeoTIFFs) before subscribing.
- Anti-entropy: `TileCatalogReconciler::reconcile()` drives request+prune on each
  received `TileCatalog`.
- Cache directory: `QStandardPaths::AppDataLocation + "/live_tile_cache/<source_ns>/"`,
  configurable via QSettings (`LiveTileCache/cache_dir`). Follows #117
  no-hardcoded-defaults: default is `AppDataLocation`, operator can override.
- "Live/preview" operator label: the layer appears in the Layers tree with a
  `(live)` suffix in its display name; no store-browser entry (it is ROS-sourced,
  not file-browsed). Aligns with ADR-0005 browse/compose split.

### Step 2 — `SonarLiveTile`: in-memory tile carrying dequantized Float32 data

New `src/camp_map/raster/sonar_live_tile.{h,cpp}`.

Holds one GGGS tile worth of per-band Float32 data received over the network
(dequantized via `value = raw * scale + offset`). Does not use GDAL. Mirrors
`GggsTile`'s interface where the render path needs it:

```cpp
struct SonarLiveBand {
  std::string name;         // "depth" | "uncertainty" | "backscatter" | "intensity"
  std::vector<float> data;  // row-major Float32, tile_width*tile_height cells
  float nodata;
  bool has_nodata;
  float data_min, data_max; // auto-range over non-nodata cells
};

class SonarLiveTile {
public:
  explicit SonarLiveTile(const marine_interfaces::msg::TileIndex& index);
  void applyPatch(const marine_interfaces::msg::SonarVisualizationTile& msg);
  // Geographic extent from GGGS (gggs::levelSpec / GridIndex → lat/lon bounds)
  double minLon(), maxLon(), minLat(), maxLat();
  int width(), height();
  const SonarLiveBand* band(const std::string& name) const;
  int bandCount() const;
  // version for newest-wins / reconciler
  marine_tiled_raster_store::TileVersion version() const;
private:
  gggs::GridIndex index_;
  int width_, height_;
  std::map<std::string, SonarLiveBand> bands_;
  TileVersion version_{0};
};
```

`applyPatch()` dequantizes the dirty sub-window from `VisualizationBand::data`
(generic: dtype + scale + offset + nodata), patches `data` at
`[window_row..][window_col..]`, updates `data_min`/`data_max` incrementally,
and bumps `version_`.

### Step 3 — `SonarLiveCacheLayer`: render + ROS subscribe + write-through

New `src/camp_map/raster/sonar_live_cache_layer.{h,cpp}`.

Extends `ros::Layer` (which extends `map::Layer`). Holds the in-memory tile
map and drives anti-entropy.

**Rendering** — duplicates the GggsTileLayer GL pipeline (shader constants,
FBO, LUT texture) for the in-memory tiles. Duplication is explicit and
temporary: a `// NOTE: shader duplicated from GggsTileLayer; will unify
// via RasterFieldSource in camp#134` comment marks the intended consolidation.
Band selection and colormap controls (via context menu, same as GggsTileLayer)
apply. Selected band/colormap persist via QSettings, keyed on the source
namespace (settingsKey = "LiveTileCache/" + source_ns_).

**ROS subscriptions** — created with the `rclcpp::Node` from `ros::Layer`'s
parent `ros::Node`:

| Topic | Message | QoS | Direction |
|---|---|---|---|
| `<base>/coverage_tiles` | `SonarVisualizationTile` | best_effort 10 | subscribe |
| `<base>/coverage_catalog` | `TileCatalog` | transient_local reliable | subscribe |
| `<base>/coverage_requests` | `TileRequest` | reliable | publish |

`<base>` is the remapped source namespace (e.g. `/cube_bathymetry`), passed at
construction from `SonarLiveCacheManager`.

**Anti-entropy** — `TileCatalogReconciler reconciler_`. On each `TileCatalog`:
1. `reconciler_.reconcile(catalog)` → `{to_request, to_prune}`
2. Publish `TileRequest` for `to_request` tiles.
3. For each `to_prune`: delete in-memory tile, delete disk file, call
   `reconciler_.drop()`.

On each `SonarVisualizationTile`:
1. Newest-wins: check tile's `header.stamp` vs `reconciler_.versionOf(index)`;
   discard if stale.
2. `SonarLiveTile::applyPatch()` — patch the in-memory tile.
3. `reconciler_.markHave(index, version)`.
4. Schedule async write-through (see below).
5. Invalidate cached render, repaint.

**Write-through** — off the GUI thread via `QtConcurrent`:
- Serialize the updated tile's selected band(s) to Float32 GeoTIFF at a temp
  path (`<cache_dir>/<level>_<row>_<col>.tif.tmp`), using GDAL `GDALCreate` with
  the GGGS georeferencing from `gggs::levelSpec` for the tile's level. Band(s)
  written as `GDT_Float32` with per-band NoData set.
- Atomic rename to `<level>_<row>_<col>.tif`.
- Write is crash-safe only — the layer never reads from disk during operation.
  In-memory state is authoritative.

**Warm-load** — constructor reads existing `<level>_<row>_<col>.tif` files from
the cache directory using the existing `GggsTile::loadPixels()` (GDAL path).
Converts loaded `GggsTile` data to `SonarLiveTile` in-memory representation.
Seeds `reconciler_` with `markHave` for each loaded tile (version 0 = "have
something; reconciler will request update on next catalog").

**Operator presentation**:
- `objectName()` = `"Live Coverage"` (or `"Live Coverage [<source_ns>]"` for
  multi-source).
- `settingsKey()` = `"LiveTileCache/" + source_ns_hash`.
- `onRemovedFromMap()`: drops the source_ns from `QSettings LiveTileCache/sources`.
- Does NOT appear in the Stores tab catalog browser (it is ROS-discovered, not
  file-browsed — ADR-0005 §2, opt-in via topic discovery).

### Step 4 — `SonarLiveCacheManager`: topic discovery and layer lifecycle

New `src/camp_map/ros/sonar_live_cache_manager.{h,cpp}`.

Extends `tools::LayerManager`. Pattern: `TopicsManager` with
`setTypeFilter({"marine_interfaces/msg/SonarVisualizationTile"})`.

On `namesUpdated()`: for each discovered topic with active publishers, infer
`<base>` (strip `~/coverage_tiles` suffix), check if a layer for that base
already exists (dedup), spawn `SonarLiveCacheLayer` if not.

On re-connect (topic reappears): if the layer exists, it is already subscribed —
no new spawn; the layer's subscription reconnects via DDS discovery.

Wired in `src/camp_map/ros/node.cpp` alongside `GridManager` and `MarkersManager`.

Persists active source namespaces under `QSettings LiveTileCache/sources`
(a `QStringList`). `createDefaultLayers()` restores a layer per still-active source
(mirrors `GggsStoreSource::instantiate` + `GggsTileLayers/dirs` pattern).

**Discover vs. opt-in**: camp#44/#68 (shared discovery util / opt-in catalog
integration) are not yet implemented. For this PR, auto-spawn follows the
`GridManager` pattern (discover + immediately spawn). An open question flags
whether operator opt-in is required (see Open Questions).

### Step 5 — `CMakeLists.txt` additions

- Add `marine_tiled_raster_store` to `find_package` and `ament_target_dependencies`.
- Add new source files to the `camp_map` library target.
- Add `test_sonar_live_cache.cpp` to the test target.

### Step 6 — Test: downtime-gap acceptance scenario

New `test/test_sonar_live_cache.cpp`.

Runs headless (no Qt event loop, no ROS node, no live boat):

1. **Warm-load test**: write synthetic Float32 GeoTIFFs to a temp dir; construct a
   `SonarLiveTile` from those tiles via the GDAL warm-load path; verify tile count
   and data range.
2. **Patch-apply test**: call `SonarLiveTile::applyPatch()` with a synthetic
   `SonarVisualizationTile` (sub-window patch); verify the patched region is
   dequantized correctly and `data_min`/`data_max` updated.
3. **Downtime-gap reconcile test** (the acceptance scenario from the issue):
   - Seed `TileCatalogReconciler` with tiles A, B, C (simulates "before downtime").
   - Receive a `TileCatalog` with tiles A, B, D (C replaced by D — boat was
     down, some coverage changed).
   - Call `reconcile()` → assert `to_request = {D}` (missing), `to_prune = {C}`
     (absent and version < generation_time).
   - Call `markHave(D, ...)`, `drop(C)` — model the consumer acting.
   - Receive a second catalog with A, B, D — assert `to_request = {}`,
     `to_prune = {}` (converged).
4. **Prune timestamp-gate test**: ensure a tile with version > catalog
   `generation_time` is NOT pruned even if absent (ADR-0008 D4b).

## Files to Change

| File | Change |
|------|--------|
| `docs/decisions/0006-live-tile-cache-persistence.md` | New ADR |
| `src/camp_map/raster/sonar_live_tile.h` | New in-memory tile type |
| `src/camp_map/raster/sonar_live_tile.cpp` | Patch-apply, dequantize, auto-range |
| `src/camp_map/raster/sonar_live_cache_layer.h` | New ROS layer |
| `src/camp_map/raster/sonar_live_cache_layer.cpp` | GL render, subscribe, reconcile, write-through, warm-load |
| `src/camp_map/ros/sonar_live_cache_manager.h` | New topic-discovery manager |
| `src/camp_map/ros/sonar_live_cache_manager.cpp` | TopicsManager + layer lifecycle |
| `src/camp_map/ros/node.cpp` | Wire SonarLiveCacheManager (alongside GridManager) |
| `CMakeLists.txt` | Add `marine_tiled_raster_store`, new sources, new test |
| `test/test_sonar_live_cache.cpp` | Downtime-gap + reconcile tests |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | GL shader code is duplicated from GggsTileLayer rather than extracting a shared renderer; that extraction belongs to Part A / camp#134. |
| Capture decisions, not just implementations | ADR-0006 records the persistence contract and the preview-vs-data-of-record distinction. |
| A change includes its consequences | CMakeLists.txt gains `marine_tiled_raster_store`; node.cpp wires the manager; test covers the downtime-gap scenario. |
| Test what breaks | Tests target the reconciler logic (request + prune, timestamp gate) and dequantize path — the failure modes hardest to catch in the field. |
| Improve incrementally | Part B only; Part A's abstraction deferred; single PR. |
| Human control and transparency | Operator sees a `(live)` suffix on the layer; write-through and warm-load are transparent to the user. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| camp ADR-0005 (browse/compose split) | Yes | Live cache layer appears in Layers tree (compose side), NOT in the Stores browser; spawned via ROS topic discovery, not file browsing. |
| camp ADR-0001 (TopicBridge executor contract) | Yes | Subscription callbacks are lightweight (data copy + signal emit); GL render is always on the GUI thread. |
| camp ADR-0002 (Web-Mercator scene + layer model) | Yes | Layer renders into the Web-Mercator scene via the same FBO+shader path as GggsTileLayer. |
| uma ADR-0008 (live sonar transport + render) | Yes | Subscribes to the D1/D3/D4 messages; applies newest-wins patch ordering; prune-on-absence with timestamp gate. |
| workspace ADR-0013 (progress.md vocabulary) | Yes | This plan is committed; progress.md updated. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Add `marine_tiled_raster_store` dep | `CMakeLists.txt` + `package.xml` | Yes |
| New layer type in Layers tree | `item_types.h` (add `SonarLiveCacheLayerType`) | Yes (in `sonar_live_cache_layer.h`) |
| `node.cpp` wires manager | `node.h` forward-declares the manager | Yes |
| GggsTile shader duplicated | Comment pointing to camp#134 consolidation | Yes |

## Open Questions

- [ ] **Discover vs. opt-in**: camp#44/#68 (shared topic-discovery + opt-in catalog) are not implemented. Should the live cache auto-spawn when the topic is detected (GridManager pattern), or require explicit operator opt-in (catalog browser entry)? Auto-spawn is proposed; confirm with operator before implementation.
- [ ] **Multi-band write-through**: The write-through step must decide which bands to persist (depth only? all received bands?). Writing all bands is safest for warm-load fidelity; writing only depth keeps the cache small. Propose: write all received bands (one Float32 band per VisualizationBand), so warm-load can reconstruct any operator-selected band.
- [ ] **Cache size limits**: `TileCatalogReconciler` has no built-in eviction; prune-on-absence handles normal shrinkage. A large survey could accumulate thousands of tiles. An eviction-by-area policy is possible follow-up. Flag if the operator expects this to be bounded.

## Estimated Scope

Single PR. ~600–900 lines net (new files: ADR, sonar_live_tile, sonar_live_cache_layer,
sonar_live_cache_manager, test; changes: node.cpp, CMakeLists.txt, item_types.h).
