# Plan: GGGS store layer — lazy/async tile load + default-off tile-sets + persisted visibility

## Issue

https://github.com/rolker/camp/issues/102

## Context

The GGGS store layer (camp#90, merged on `jazzy`) loads **every** tile
synchronously in the `GggsTile` constructor: a full-band GDAL `RasterIO` on the
GUI thread (`gggs_tile.cpp:51-57`). For the small Massabesic store that's fine,
but a store over *all* existing data would freeze the UI at startup, and every
tile-set loads whether the operator wants it or not. The fix generalizes the
pattern `RasterLayer` already uses per camp ADR-0003 §3: cheap extent/metadata
in the constructor, the pixel read deferred to a `QtConcurrent` worker with an
abort/join dtor contract.

Five cohesive changes toward one goal — open a large store without blocking, and
read pixels only for layers the operator turns on. Visible-region LOD render,
producer pyramids, and live ROS transport stay out of scope (separate issues).

## Approach

1. **Split `GggsTile` into extent-read (ctor) + deferred pixel-read.** The ctor
   opens the dataset, reads the geotransform → extent, dimensions, and NoData
   only (no `RasterIO`); `valid()` becomes true on dimensions/geotransform alone.
   A new `loadPixels()` does the band `RasterIO` and computes `data_min_/data_max_`.
   `texture()` keeps its current-context upload but is only reachable once pixels
   are present. The colormap auto-range (`data_min_/data_max_`) therefore becomes
   **incremental**: it is unknown until a tile's pixels load, so the layer's
   `data_min_/data_max_` accumulate as tiles complete (not at `loadDirectory`
   time). Keep `texture()` and `releaseGL()` on the render thread only — never
   call them off-thread (OQ1).

2. **Async pixel load on `GggsTileLayer`, mirroring `RasterLayer`.** Add a
   `QFutureWatcher<...>` + `abort_flag_`/`abort_flag_mutex_` to the layer (not the
   tile — one watcher per layer drives that layer's tile reads). A `loadTiles()`
   slot launches `QtConcurrent::run` over the layer's not-yet-loaded tiles, each
   honoring the abort flag between tiles; an `tilesReady()` slot folds the new
   ranges into `data_min_/data_max_`, invalidates `cached_image_`, and calls
   `update()`. Worker does **GDAL `RasterIO` only**; `texture()` upload stays on
   the paint path (OQ1). Port the dtor contract **verbatim** from `RasterLayer`
   (`raster_layer.cpp:38-44`, `103-123`): set `abort_flag_` under mutex +
   `waitForFinished()` in the dtor and before re-launching, so a layer destroyed
   mid-load can't outlive `this` (OQ4).

3. **Trigger load only when the layer is visible/drawn.** `loadDirectory` keeps
   building the cheap-extent tile list (so `boundingRect()`/`sceneBounds()` are
   valid immediately) but launches **no** pixel reads. The first `paint()` (which
   QGraphicsView calls only for visible items) lazily kicks `loadTiles()` if not
   already loaded/loading; an in-flight tile is skipped in `renderImage` (its
   `texture()` returns null) and the layer repaints on `tilesReady()` (OQ2). This
   piggybacks on Qt's existing "don't paint hidden items" rather than adding a
   visibility-change observer MapItem doesn't have.

4. **Default tile-set leaves OFF, persisted.** Override `readSettings()` on
   `GggsTileLayer` to read `visible` with a **`false`** fallback (the auto-range
   `data_min_/data_max_` and the colormap already round-trip via the existing
   override). A ctor `setVisible(false)` would be clobbered: `MapItem::itemConstructed()`
   runs `readSettings()` via `QTimer::singleShot(0,…)` after the ctor
   (`map_item.cpp:26,180-183`), and `Layer::readSettings` defaults `visible` to
   `true` (`layer.cpp:101-102`). So the off-default must live in the leaf's
   `readSettings` fallback, **not** a ctor call and **not** a base-class change
   (which would wrongly default charts/AIS off) (OQ3). `GggsStoreLayer` grouping
   nodes are **not** forced off — only the renderable `GggsTileLayer` leaves.
   Note: `Map::setData` toggles `setVisible` but does **not** call
   `writeSettings` (`map.cpp:110-112`); visibility persists at app-quit
   (`MapItem::applicationQuitting` → `writeSettings`), as today — the leaf
   override only changes the *default*, the persistence path is unchanged.

5. **`QFileSystemWatcher` on the store for new tiles/epochs.** `GggsStoreLayer`
   watches its directory subtree; on a change it (a) discovers newly-landed
   tile-set dirs and adds `GggsTileLayer`/`GggsStoreLayer` children (default-off
   per change 4), and (b) for an existing tile-set, signals the leaf to re-scan
   and re-read changed tiles. A half-written tile that fires mid-write degrades
   to `valid()==false` (or a crossed range) and is simply skipped — producer-side
   atomic-write safety is the separate uma#189; the consumer only tolerates it
   (OQ5). Watch granularity: add one watch per discovered directory (Qt watches
   are non-recursive); reload semantics are **incremental add**, not a full
   rebuild, so already-loaded tiles and operator on/off choices are preserved.

6. **Update stale docs.** Fix the `gggs_tile.h:15-20` class comment (ctor no
   longer reads CPU samples) and the `background_manager.cpp:70-73` TODO (the
   lazy/async path now exists). Record the GL-upload-vs-async-GDAL split + the
   default-off-for-tile-sets decision as a PR/issue note (small enough that a
   note suffices; not a new ADR — it generalizes ADR-0003 §3, and §4 persists
   app-state visibility, which default-off respects).

## Files to Change

| File | Change |
|------|--------|
| `src/camp2/raster/gggs_tile.h` | Add `loadPixels()`; `pixelsLoaded()`; ctor reads extent/metadata only; fix class doc comment |
| `src/camp2/raster/gggs_tile.cpp` | Move band `RasterIO` + range computation from ctor into `loadPixels()`; ctor stops at geotransform/dimensions/NoData |
| `src/camp2/raster/gggs_tile_layer.h` | Add `QFutureWatcher<void>` + `abort_flag_`/mutex, `loadTiles()`/`tilesReady()` slots + `loadTilesWorker()`, `load_started_` state, `readSettings()` extend (false `visible` fallback), verbatim abort/join dtor. Also added `waitForLoad()` (test seam) and `rescan()` (watcher hook) — see Deviations |
| `src/camp2/raster/gggs_tile_layer.cpp` | `loadDirectory` extent-only (range fold removed — now incremental); async `loadTiles`/`loadTilesWorker` (RasterIO only, abort between tiles); `tilesReady` incremental `data_min_/data_max_` fold + `pixelsLoaded()` gate; lazy kick from `paint`; skip not-`pixelsLoaded()` tiles in `renderImage`; `readSettings` `visible` false fallback; verbatim abort/join dtor + re-launch guard in `loadTiles`; `rescan()` for new tiles |
| `src/camp2/raster/gggs_store_layer.h` | Add `QFileSystemWatcher*` + `onDirectoryChanged()` slot + `scan()`; `tileset_children_`/`group_children_` sets to make re-scan incremental |
| `src/camp2/raster/gggs_store_layer.cpp` | `build`→`scan`; install one watch per visited directory; on change, discover new children (default-off) + call `rescan()` on the changed existing leaf; incremental, not rebuild; watch-count cap deferred (review #4) |
| `src/camp2/background/background_manager.cpp` | Update the stale Slice-2 TODO comment now that lazy/async lands |
| `test/test_gggs_tile.cpp` | Add `ExtentKnownBeforePixelsLoad` (ctor valid extent + crossed range *before* `loadPixels()`; range populated after); update `NoDataExcludedFromRange`/`AllNoDataHasCrossedRange` to call `loadPixels()` first |
| `test/test_gggs_visibility.cpp` (new) | Default-off + persisted-visibility round-trip on a tile-set leaf (non-GL state machine per review #3; render path covered by existing `test_gggs_render`, now driven via `waitForLoad()`) |
| `test/test_gggs_render.cpp` | Drive + await the async load (`waitForLoad()`) before `renderImage` (pixels are no longer synchronous) |
| `CMakeLists.txt` | Wire `test_gggs_visibility`; gggs tests already link `camp_map` (Qt5::Concurrent is PUBLIC on it) |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Robustness / do-it-completely | Port the abort/join dtor *verbatim* (no "good enough" partial join); tolerate half-written tiles by degrading the tile, not crashing; tests cover the risky concurrency + visibility-default logic |
| Verify against source, not assumptions | Plan pins exact precedents: `RasterLayer` ctor/dtor/`loadFile`/`imageReady`, `MapItem::itemConstructed` → `readSettings` ordering, `Map::setData` not calling `writeSettings` |
| Tests are the only gate (camp has no CI/pre-commit) | New gtests for (a) extent-before-pixels and (b) default-off persistence round-trip — the two highest-risk behaviors |
| Atomic commits | Sequence: (1) tile split + test, (2) layer async + dtor, (3) lazy-on-visible, (4) default-off + test, (5) watcher, (6) doc/TODO cleanup |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| camp ADR-0003 §3 (sync extent / async pixels) | Yes | Directly generalizes the `RasterLayer` extent-in-ctor + async-pixel-on-worker contract to `GggsTile`/`GggsTileLayer`; same abort/join dtor |
| camp ADR-0003 §4 (backgrounds persist as app/Map state: visibility/order/opacity/colormap) | Yes | Default-off is a *default* change only; visibility still persists via the existing app-state path. PR note records that default-off-for-tile-sets is an intentional refinement of §4's app-state visibility, not a departure from its persistence mechanism |
| camp ADR-0002 (Web-Mercator scene, layer model) | No (unchanged) | Tile geometry/warp untouched; only load timing + visibility default change |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `GggsTile` ctor no longer reads pixels | `gggs_tile.h` class doc comment; any caller assuming `dataMin/Max` valid post-ctor (the layer — now incremental) | Yes |
| Lazy/async path now exists | `background_manager.cpp:70-73` Slice-2 TODO | Yes |
| Default tile-sets OFF | Operator UX (must turn layers on); persistence is via existing QSettings path | Yes (documented in PR note) |
| New `QFileSystemWatcher` | Watch-count limits over a deep store (per-dir, non-recursive) — noted as a bounded, incremental-add design | Yes |

## Open Questions

- **OQ-A (watcher scale):** per-directory watches over a deep multi-epoch store
  could hit the OS inotify watch limit on a very large store. Plan caps risk with
  incremental-add semantics, but is a watch-count ceiling (e.g. watch only the
  store root + active epoch dirs) wanted now, or deferred until stores get big?
- **OQ-B (in-flight render test):** a headless gtest for "skip in-flight tile,
  repaint on completion" may need GL/event-loop scaffolding the existing
  `test_gggs_render` harness doesn't have. If it's not cleanly headless-testable,
  is asserting the *non-GL* state machine (loading/loaded flags + range folding)
  sufficient, with the render path covered manually?

## Estimated Scope

Single PR, ~6 atomic commits (tile split, layer async+dtor, lazy-on-visible,
default-off+test, watcher, doc cleanup). Self-contained within `src/camp2/raster/`
plus one TODO-comment touch in `background_manager.cpp` and test additions.

## Deviations (recorded during implementation)

- **`GggsTileLayer::waitForLoad()` added** (not in the original Files-to-Change).
  The existing `test_gggs_render` calls `renderImage()` directly, outside the
  QGraphicsView paint loop that normally kicks + awaits the load via signals. A
  small public test seam (`load if not started → waitForFinished → tilesReady`)
  keeps that test valid without GL/event-loop scaffolding (review #3 intent).
- **`GggsTileLayer::rescan()` added.** The plan said "signal the leaf to re-scan
  and re-read changed tiles"; this is the concrete API the store's
  `onDirectoryChanged` calls — it aborts/joins any in-flight load, appends
  extent-only entries for newly-landed tiles (a half-written tile degrades to
  `valid()==false` and is skipped), and re-kicks the load if already loaded.
- **`QFutureWatcher<void>`** (not `<LoadResult>`): the worker mutates each shared
  `GggsTile` in place (`loadPixels()`), so there is no per-future result struct
  to marshal — the watcher only signals completion; `tilesReady()` reads the
  tiles. This still ports RasterLayer's abort/join + re-launch contract verbatim.
- **New test file `test_gggs_visibility.cpp`** (vs. extending `test_gggs_render`)
  — the visibility round-trip needs only `QApplication` + `QSettings`, no GL, so
  a dedicated file is cleaner. Uses a tiny subclass to reach the protected
  `readSettings()`/`writeSettings()` without the deferred `itemConstructed()` timer.
- **Watcher inotify watch-count cap DEFERRED** (review #4 / OQ-A): per-directory
  non-recursive watches with incremental-add semantics, sized for the
  Massabesic-scale store; a root+active-epoch ceiling is premature. Noted in a
  code comment in `gggs_store_layer.h` and the PR.
