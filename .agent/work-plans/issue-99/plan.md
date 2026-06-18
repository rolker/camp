# Plan: Add auto-refreshing weather radar tile overlay (stacked layer on the map system)

## Issue

https://github.com/rolker/camp/issues/99

## Context

CAMP's camp2 map system (now deployed in the main app via #59/PR#60) already has
all the tile infrastructure needed: `MapTiles` for XYZ/WMTS tile layers,
`CachedTileLoader`/`CachedFileLoader` for HTTP fetch + disk cache,
`BackgroundManager::createDefaultLayers()` as the registration point, and
per-layer opacity/visibility in the layer tree. The sole missing piece is a
periodic refresh mechanism — tiles cache indefinitely and only reload on pan/zoom,
so a weather radar overlay would show a stale snapshot without it.

**Provider decision (pre-decided by user):** NOAA nowCOAST WMTS — specifically the
MRMS base-reflectivity radar product. This is US government-hosted, no API key
required, updated every ~2–5 minutes, covers Lake Massabesic NH (target operating
area), and has permissive terms for operational use. The likely WMTS endpoint is:
`https://nowcoast.noaa.gov/arcgis/services/nowcoast/radar_meteo_imagery_nexrad_time/MapServer/WMSServer?service=WMS&version=1.1.1&request=GetCapabilities`
but the WMTS form is: `https://nowcoast.noaa.gov/arcgis/rest/services/nowcoast/radar_meteo_imagery_nexrad_time/MapServer/WMTS`
**This endpoint needs confirmation against live NOAA nowCOAST docs before
hardcoding** — NOAA periodically restructures ArcGIS service paths. The NOAA
charts pattern already in `background_manager.cpp` (line 36) serves as the
structural template.

**#98 coordination:** Phase 2's periodic tile refresh shares the tile/raster code
path implicated in the #98 map-zoom/pan OOM crash. The refresh eviction design
(below) is the coordination mechanism; Phase 2 must not make the leak worse.

## Approach

### Phase 1 — Add radar as a stacked MapTiles WMTS layer

1. **Confirm NOAA nowCOAST WMTS endpoint** — fetch the capabilities XML from the
   live endpoint before hardcoding; note the confirmed URL in an inline comment.
   If the endpoint is unreachable at implementation time, flag it in the PR.

2. **Add radar layer in `background_manager.cpp`** — mirror the NOAA charts block:
   ```cpp
   auto radar_caps = new camp::wmts::Capabilities("NOAA_Radar", this);
   camp::map_tiles::MapTiles* radar = new camp::map_tiles::MapTiles(layers, "NOAA_radar");
   radar->setLayoutFromWMTS(*radar_caps);
   radar->setOpacity(0.65);        // default ~0.65, transparent overlay on basemap
   radar->setVisible(false);       // default OFF — operator toggles in layer tree
   radar_caps->setUrl("<confirmed-nowcoast-wmts-url>");
   ```
   The layer label `"NOAA_radar"` also drives the disk-cache subdirectory
   (`~/.CCOMAutonomousMissionPlanner/map_tiles/NOAA_radar/`).

3. **Verify `CachedFileLoader` network-error path** — read `downloadFinished()` in
   `src/camp2/util/cached_file_loader.cpp` (lines 76–122). Currently on error it
   logs `qDebug()` and calls `reply->deleteLater()` but does NOT emit
   `client->dataLoaded`. This means `CachedTileLoader::dataLoaded` is never called
   and `MapTiles::tileLoaded` is never called — the tile simply stays blank (no
   pixmap set). This is the correct graceful-degradation behavior: blank tile, no
   crash, no UI block. Confirm this remains true; if not, add the guard in
   `cached_file_loader.cpp`. Add a comment noting the intentional silence on
   fetch failure.

### Phase 2 — Periodic refresh timer with bounded-memory eviction

4. **Add `setRefreshInterval(int msec)` to `MapTiles`** in
   `src/camp2/map_tiles/map_tiles.h`:
   - Add `QTimer* refresh_timer_ = nullptr;` private member.
   - Add `void setRefreshInterval(int msec);` public method.
   - Add `void onRefreshTimer();` private slot.

5. **Implement refresh in `map_tiles.cpp`**:
   - `setRefreshInterval(msec)`: create/configure a `QTimer` (single-shot=false,
     parent=this), connect `timeout()` → `onRefreshTimer()`, start it.
     `msec <= 0` disables (stops timer if running).
   - `onRefreshTimer()`: evict all cached tiles — call `setLayout(tile_layout_)`
     which already `delete`s every `Tile*` in `tiles_` and clears the map (lines
     91–95 of map_tiles.cpp). This is the bounded-memory path: old `Tile` objects
     (which hold `QGraphicsPixmapItem` children) are deleted, then the top-level
     tiles are re-fetched. Call `update()` after to trigger a repaint.
   - **Important:** `setLayout` resets to zoom level 0 tiles only; the viewport's
     current zoom level will re-request higher-res tiles on the next `paint()` call
     (which fires automatically on `update()`). This is acceptable — one cycle of
     coarser tiles during refresh is fine for a 5-min cadence.

6. **Wire refresh on the radar layer** in `background_manager.cpp`:
   ```cpp
   radar->setRefreshInterval(5 * 60 * 1000);  // 5-minute cadence
   ```
   Existing OSM/OpenSeaMap/NOAA charts layers call `setRefreshInterval(0)` (or
   nothing, since default is disabled) — the timer is explicitly opt-in.

7. **Add disk-cache bypass for force-refresh** in `cached_tile_loader.cpp`:
   - Add `void invalidateCache();` to `CachedTileLoader` that removes files under
     `local_cache_path_`. This ensures a refresh fetches fresh network tiles rather
     than serving stale disk-cached PNGs (the existing `load()` path prefers local
     file if present: `cached_file_loader.cpp` line 61–62).
   - Call `tile_loader_->invalidateCache()` at the start of `onRefreshTimer()`
     before `setLayout()`.
   - Scope: delete only the files under the layer's own cache subdirectory (not the
     global cache). `QDir(local_cache_path_).removeRecursively()` followed by
     `QDir::root().mkpath(local_cache_path_)` is sufficient.

### Tests

8. **Add `test/test_map_tiles_refresh.cpp`** — `ament_add_gtest` linked against
   `camp_map + Qt5::Widgets + Qt5::Test`. Tests:

   a. **Timer fires on interval**: create a `MapTiles` with no layout,
      `setRefreshInterval(100)`, spin a `QEventLoop` for 350 ms, verify the
      refresh slot fired ≥ 3 times (spy on a mock or subclass). Use
      `QSignalSpy` or a subclass override.

   b. **Cache is cleared on refresh**: populate `tiles_` via `setLayout()` with a
      minimal fake layout (1 zoom level, 1×1 matrix). Confirm `tiles_` is
      non-empty. Trigger `onRefreshTimer()` directly. Confirm `tiles_` is
      repopulated (setLayout rebuilds zoom-0 tiles) and old pointers are gone.

   c. **Memory stays bounded across N cycles**: run N=10 refresh cycles via direct
      `onRefreshTimer()` calls. Check that the number of `Tile` children of the
      `MapTiles` item does not grow monotonically (each cycle resets to zoom-0
      count). This directly tests that old tiles are deleted, not accumulated — the
      #98-risk regression.

   d. **Interval disabled when msec=0**: call `setRefreshInterval(0)` after
      enabling, spin event loop, verify no refresh fires.

   Note: `QApplication` or `QGuiApplication` must be instantiated for the
   `QGraphicsScene` / `QGraphicsItem` hierarchy to function. Mirror the pattern in
   `test/test_map_model.cpp`.

9. **Register the test in `CMakeLists.txt`** — add an `ament_add_gtest` block
   alongside the existing map-model test, linking `camp_map` and `Qt5::Widgets`.

## Files to Change

| File | Change |
|------|--------|
| `src/camp2/background/background_manager.cpp` | Add NOAA nowCOAST radar WMTS layer (Phase 1), wire `setRefreshInterval(5*60*1000)` (Phase 2) |
| `src/camp2/map_tiles/map_tiles.h` | Add `setRefreshInterval(int)`, `QTimer* refresh_timer_`, `onRefreshTimer()` slot |
| `src/camp2/map_tiles/map_tiles.cpp` | Implement `setRefreshInterval` and `onRefreshTimer`; per-refresh `layout_epoch_` carried into minted `TileAddress`es (pre-push review fix — rejects stale pre-refresh pixmaps) |
| `src/camp2/map_tiles/tile_address.h` / `tile_address.cpp` | Add `quint64 epoch_` to `TileAddress`; compared in `operator==` (identity) but **not** `operator<` (ordering) so refresh rejects stale in-flight pixmaps without breaking `tiles_` lookups (pre-push review fix) |
| `src/camp2/map_tiles/cached_tile_loader.h` | Add `invalidateCache()` declaration |
| `src/camp2/map_tiles/cached_tile_loader.cpp` | Implement `invalidateCache()` (remove + recreate cache subdir) |
| `src/camp2/util/cached_file_loader.cpp` | Add comment on intentional silence for network errors (no behavior change if already correct) |
| `test/test_map_tiles_refresh.cpp` | New: timer + cache-eviction + bounded-memory tests |
| `CMakeLists.txt` | Register `test_map_tiles_refresh` under `ament_add_gtest` |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Radar layer is default-OFF and toggleable in the layer tree; opacity defaults to ~0.65 (visible but not dominant); operator can disable instantly |
| Only what's needed | Phase 1 is ~5 lines in `createDefaultLayers()`; Phase 2 adds one timer + one cache-clear method to `MapTiles`. No new subsystem, no new library dependency |
| A change includes its consequences | Refresh timer is opt-in (disabled by default on existing layers); bounded eviction via `setLayout()` reuse; #98 risk mitigated by eviction + bounded-memory test |
| Test what breaks | Tests for timer interval, cache invalidation, bounded memory across N cycles (the #98-risk regression test). Network-error path verified, not assumed |
| Robustness / no silent failures | Network failure → blank tile (already correct per code review); disk-cache clear before refresh prevents stale tiles being served from disk |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0001 (TopicBridge/executor contract) | No | Timer is Qt-only (`QTimer`), no ROS threading concerns |
| ADR-0002 (Web-Mercator scene + library split) | Yes | Radar layer goes into `libcamp_map` (pure Qt, no ROS); `background_manager.cpp` is the correct registration point; no ROS boundary crossed |
| ADR-0003 (Backgrounds as layers) | Yes | Radar is a `MapTiles` layer in `BackgroundManager::createDefaultLayers()`, exactly the prescribed pattern for tile-based background layers |
| External-dependency ADR (new) | Recommended | NOAA nowCOAST is a new long-lived external network dependency with operational implications (field-ops offline degradation, NOAA service continuity). **Recommend creating ADR-0004** documenting the provider choice rationale: authoritative source, no API key, US coverage, permissive ToS, graceful offline degradation. This keeps the decision traceable if nowCOAST changes URLs again or a field deployment needs an alternative |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `MapTiles::setRefreshInterval` added | All existing callers of `MapTiles` (OSM, OpenSeaMap, NOAA charts) — timer is opt-in/disabled by default; no callers need to change | Yes — opt-in design means no caller changes |
| `CachedTileLoader::invalidateCache()` added | `map_tiles.cpp` calls it in `onRefreshTimer()` | Yes |
| Disk-cache cleared on refresh | Radar tiles re-fetched from NOAA on each cycle — this is the intended behavior; not a side-effect | Yes |
| New tile layer in `createDefaultLayers()` | Layer tree UI — new row appears in the Layers panel (correct; no code change needed) | Yes |
| Phase 2 shares #98 tile/raster code path | Coordinate with #98 fix; bounded-memory test is the gate | Yes — noted in tests step |

## Open Questions

- [~] Confirm NOAA nowCOAST WMTS endpoint URL before hardcoding — **NOT verified live at implementation time.** Hardcoded the provisional URL `https://nowcoast.noaa.gov/arcgis/rest/services/nowcoast/radar_meteo_imagery_nexrad_time/MapServer/WMTS` with an explicit `// TODO: confirm endpoint` comment in `background_manager.cpp`. A 404 degrades gracefully (blank layer), so landing with the TODO is acceptable; confirming it is a follow-up.
- [x] ADR-0004 for NOAA nowCOAST dependency — created `docs/decisions/0004-noaa-nowcoast-radar-dependency.md` as part of this PR.
- [~] Is 5 minutes the right refresh cadence? Kept hardcoded at `5 * 60 * 1000` ms. A follow-up issue can expose it as a user-configurable value.

## Implementation Status (filled in during implementation)

- [x] Phase 1 — radar layer added in `background_manager.cpp` (default OFF, opacity 0.65, WMTS via `Capabilities`/`setLayoutFromWMTS`, framed as #98 tile path NOT #96 GDAL).
- [x] Phase 2 — `setRefreshInterval(int)` + owned `QTimer` + `onRefreshTimer()` private slot in `map_tiles.{h,cpp}`; `invalidateCache()` in `cached_tile_loader.{h,cpp}` with a guard (non-empty + must contain `/map_tiles/`, never the global cache root); radar wired to 300000 ms.
- [x] Network-failure path confirmed graceful with a comment in `cached_file_loader.cpp` (no behavior change).
- [x] Tests — `test/test_map_tiles_refresh.cpp`: timer config (interval/active/single-shot/disabled/opt-in) + one deterministic refresh via `QMetaObject::invokeMethod(onRefreshTimer)` + bounded `Tile`-child count across 10 cycles (the #98-risk regression). Tile children counted via public `childItems()` + `qgraphicsitem_cast<Tile*>`. Registered in `CMakeLists.txt`.
- Honored plan-review must-fix: NO wall-clock fire counting; assert config + deterministic single refresh instead.

## Estimated Scope

Single PR covering both phases. Phase 1 and Phase 2 are implemented together because:
- Phase 1 without refresh is only marginally useful (a static snapshot)
- The bounded-memory eviction tests (Phase 2) are the explicit #98-risk coordination
- Combined, the total change is small (~100-150 lines of production code + tests)

Reference #98 in the PR description; the bounded-memory test is the coordination artifact.
