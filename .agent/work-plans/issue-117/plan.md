# Plan: Remove hard-coded tile layers; add preset UI + QSettings persistence

## Issue

https://github.com/rolker/camp/issues/117

## Context

`BackgroundManager::createDefaultLayers()` unconditionally mints four hard-coded
tile/WMTS layers (OSM, OpenSeaMap, NOAA Charts, NEXRAD radar) every session.
GGGS tile-sets and rasters in the same method already follow a persistence model
(restored from `GggsTileLayers/dirs` / `GggsRasters/files` in QSettings). This
issue extends that model to tile layers: remove the hard-coded blocks, add a
`BackgroundTileLayers` QSettings key that persists operator-created layers, restore
them on startup, and provide a UI with a preset list to create new ones.

Checkpoint resolutions (operator, 2026-07-24):
- **First-run/upgrade seed**: seed OSM only, whenever `BackgroundTileLayers` key
  is absent (covers both fresh install and upgrade from a hard-coded-layers build).
- **GEBCO/WMS**: include now as an inert data entry in the preset table
  (type=WMS, hidden/greyed-out until #118 lands the WMS layer type).

## Approach

1. **Define preset table** — new `tile_layer_presets.h` in `background/` with
   `TileLayerPreset` struct and `builtinPresets()` function. Carries all per-preset
   attributes (name, type, URL, opacity, visible, refresh_ms, enabled). WMS entries
   have `enabled=false` until #118.

2. **Define QSettings schema** — per-layer **keyed groups**, not an index-addressed
   array (plan-review suggestion: keyed removal must not rewrite the whole array;
   mirrors GGGS `settingsKey()`):
   - `BackgroundTileLayers/ids` — QStringList of layer names, creation order.
   - `BackgroundTileLayers/<percent-encoded-name>/` — **construction parameters
     only**: `type` ("xyz" | "wmts"), `url`, `refresh_ms`, and for WMTS the
     non-default `layer_id` / `tile_matrix_set`.
   - Presentation state (opacity, visibility) is NOT duplicated here — the
     existing `MapItem/<settingsKey()>` mechanism (`Layer::read/writeSettings`)
     already round-trips it. At add time the preset's default opacity/visible are
     written into that group so the deferred `readSettings()` (itemConstructed
     timer) applies them instead of the 1.0/true defaults; thereafter the standard
     mechanism owns them.
   WMS presets are never written (no constructible type until #118).

3. **Update `createDefaultLayers()`** — replace the four hard-coded `new MapTiles`
   calls with:
   - If the `BackgroundTileLayers/ids` key is absent, call
     `seedDefaultTileLayers()` to write the OSM preset to QSettings.
   - Restore loop over `ids`, instantiating each saved entry as a `MapTiles`
     layer (XYZ via `osm::generateTileLayout`, WMTS via `Capabilities`), deduped
     against already-live MapTiles of the same name.
   - WMTS restore replicates the async ordering/ownership of the current
     hard-coded block (plan-review suggestion): `Capabilities` parented to
     `this`, then `MapTiles` + `setLayoutFromWMTS(*caps, layer_id,
     tile_matrix_set)`, then `caps->setUrl(url)` — the URL fetch is what fires
     `ready()`, so it must come last.
   This mirrors the existing GGGS restore block in the same function.

4. **Add `seedDefaultTileLayers()`** — private method that writes a single OSM
   entry to the `BackgroundTileLayers` QSettings array. Called only when the key
   is absent.

5. **Add `persistTileLayer()`** — private method that adds a layer's name to
   `BackgroundTileLayers/ids` and writes its construction group. Called by
   `addTileLayer()` after the layer is successfully created.

5b. **De-persist on removal (plan-review must-fix)** — removal happens via the
   layer's own Layers-tree "Remove" action (ADR-0003 §4), not BackgroundManager,
   so `MapTiles` overrides `Layer::onRemovedFromMap()` (the GGGS pattern,
   `gggs_tile_layer.cpp:778`): drop `objectName()` from
   `BackgroundTileLayers/ids` and remove its construction group. A MapTiles that
   was never persisted (name not in `ids`) no-ops, keeping the hook safe for
   non-persisted uses of the generic class. Covered by a dedicated test.

6. **Add "Add tile layer" context menu action** in `BackgroundManager::contextMenu()`
   alongside "Open raster".

7. **Implement `addTileLayer()` slot** — opens `AddTileLayerDialog`; on accept,
   instantiates the `MapTiles` layer (XYZ or WMTS), calls `persistTileLayer()`.

8. **New `AddTileLayerDialog`** (`add_tile_layer_dialog.h/.cpp`) — Qt dialog that
   shows the built-in preset list (WMS entries greyed-out, tooltip: "requires #118"),
   plus a "Custom" option revealing URL + type fields. Uses standard
   `QDialogButtonBox` (OK/Cancel). Returns a `TileLayerPreset` struct on accept.

9. **Update `CMakeLists.txt`** — add `add_tile_layer_dialog.cpp` to
   `CAMP_MAP_SOURCES`.

10. **Add tests** — new `test/test_background_persistence.cpp` (with a test
    org/app name in `main()` like `test_gggs_persistence.cpp:399-400`, so runs
    never touch the developer's real QSettings — plan-review suggestion):
    - Fresh QSettings → `createDefaultLayers()` seeds OSM, one layer present.
    - Key already present → no re-seeding, existing entries restored (deduped).
    - Persist round-trip: add → restart (new Map) → layer restored with its
      construction params; preset defaults (opacity/visible) applied.
    - **De-persist on remove (must-fix)**: `removeFromMap()` on a persisted
      MapTiles drops it from `ids` + removes its group; next Map does not
      recreate it.

11. **Fix `test_map_model.cpp` QSettings leakage (plan-review suggestion)** —
    its `main()` sets no org/app name, so the new seed would write into the
    developer's real QSettings; set `camp_test`/`test_map_model` like the other
    suites. Its top-3-prefix assertion (`test_map_model.cpp:262`) tolerates the
    1-layer OSM seed by design; verify it passes.

12. **ADR** — add a short addendum to camp ADR-0003 documenting the
    `BackgroundTileLayers` schema (ids + per-name construction groups,
    presentation state delegated to `MapItem/<settingsKey>`) and the OSM-only
    key-absent seed strategy (plan-review suggestion; GGGS's equivalent key is
    documented in ADR-0005).

## Files to Change

| File | Change |
|------|--------|
| `src/camp_map/background/background_manager.cpp` | Replace 4 hard-coded `new MapTiles` blocks with QSettings restore loop; add seed/persist logic; add `addTileLayer()` slot; extend `contextMenu()` |
| `src/camp_map/background/background_manager.h` | Add `addTileLayer()` private slot; add `seedDefaultTileLayers()` and `persistTileLayer()` private methods |
| `src/camp_map/background/tile_layer_presets.h` | New: `TileLayerPreset` struct + `builtinPresets()` function |
| `src/camp_map/background/add_tile_layer_dialog.h` | New: `AddTileLayerDialog` class header |
| `src/camp_map/background/add_tile_layer_dialog.cpp` | New: `AddTileLayerDialog` implementation |
| `src/camp_map/map_tiles/map_tiles.h/.cpp` | Override `onRemovedFromMap()` to de-persist (must-fix) |
| `CMakeLists.txt` | Add `add_tile_layer_dialog.cpp` to `CAMP_MAP_SOURCES`; add `test_background_persistence` test target |
| `test/test_background_persistence.cpp` | New: fresh-start seed, upgrade-path, persistence round-trip, de-persist-on-remove tests |
| `test/test_map_model.cpp` | Set test org/app name in `main()` (QSettings isolation) |
| `docs/decisions/0003-*.md` | Addendum: `BackgroundTileLayers` schema + seed strategy |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Moves from invisible hard-code to operator-controlled, QSettings-persisted layers. OSM seed is written to QSettings immediately, so the operator can remove it from the first restart. |
| Only what's needed | Dialog is minimal (preset list + custom URL). No user-extensible saved presets for v1. GEBCO is a data-only entry (no new code paths until #118). |
| A change includes its consequences | Persistence round-trip tests included. Upgrade path covered by key-absent check in step 3 (not a version check). |
| Test what breaks | Tests cover the failure modes that matter: accidental re-seeding on upgrade, persistence mismatch, duplicate-on-restart. |
| Capture decisions, not just implementations | QSettings schema and seed strategy are non-obvious; plan records them explicitly. An ADR addendum to camp ADR-0003 may be warranted after implementation. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| camp ADR-0003 §4 | Yes — directly | `BackgroundTileLayers` QSettings array completes the app-level persistence pattern §4 describes; hard-coded layers were the historical exception. |
| camp ADR-0006 D2 | Informational | ADR-0006 already cites #117 as the "defaults, never a hardcode" precedent — direction confirmed. |
| workspace ADR-0001 | Watch | QSettings key schema + seed strategy are design decisions; consider a camp ADR addendum to ADR-0003 after implementation if the choices are non-obvious to future agents. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `createDefaultLayers()` removes hard-coded layers | Existing users (no `BackgroundTileLayers` key) get OSM seed on first launch | Yes — key-absent check covers upgrade path |
| Add `add_tile_layer_dialog.cpp` | `CMakeLists.txt` `CAMP_MAP_SOURCES` | Yes — step 9 |
| Add test file | `CMakeLists.txt` test target | Yes — step 10 |
| #99 NEXRAD behavior | Preserved via the NEXRAD preset; behavior reproduced when operator adds it | Yes — preset table carries opacity/refresh_ms |

## Open Questions

- [ ] No open questions — all operator checkpoints resolved (2026-07-24). Plan is review-plan-ready.

## Estimated Scope

Single PR. ~5 new/modified files, ~300-400 lines net added.
