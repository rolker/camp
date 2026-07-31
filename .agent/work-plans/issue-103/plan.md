# Plan: LOD level-selection + demand-driven loading for GggsTileLayer (#103 — LOD half)

## Issue

https://github.com/rolker/camp/issues/103

## Context

The visible-region half landed in PR#173 (camp ADR-0011). The issue stays open for
the LOD half, now unblocked by unh_marine_autonomy#188 (`build_sidescan_overviews`,
ADR-0011 producer: 1012 fine L13 tiles → 479 overview tiles to a single L0 apex;
flat `overviews/<level>_<row>_<col>.tif` consumer contract).

**Two problems this plan closes:**

1. **Eager full-store load** — `loadTilesWorker()` reads every tile's pixels at
   construction (3.6 GB observed open cost on the sidescan store). The fix: demand-driven
   — only load pixels for tiles at the selected LOD level that intersect the current
   viewport.

2. **No level selection** — at fit-zoom the renderer draws all fine L13 tiles (too many
   / too small). The fix: pick the GGGS level whose cell size best matches the
   viewport's metres-per-pixel, reading coarse tiles from the `overviews/` sidecar.

The scope also includes generic native multi-level wiring (operator checkpoint,
2026-07-31): the level-selection function must serve both (a) the `overviews/` sidecar
case and (b) natively multi-level layers (chart ENC scale ladder), even though no camp
chart layer exists yet.

## Approach

### 1. Parse tile level from filename — `gggs_tile_util.h`

Add `tileLevel(const QString& filename) → int` (name unified with camp#180's identical helper at the jazzy merge) using the existing anchored regex
but with a capture group for the first digit sequence. Returns -1 on mismatch. Pure
function, unit-testable, no new dependencies.

### 2. Store level in GggsTile — `gggs_tile.h / .cpp`

Add `int level_ = -1` set in the ctor from `tileLevel(QFileInfo(path).fileName())` (shared with camp#180's getElevation caching).
Expose `level()` accessor. Add `resetPixels()` (clears `data_`, crosses the range
sentinel, stores `pixels_loaded_ = false` with RELEASE) to release CPU memory when
switching levels — mirrors `setBand()`'s existing clear path. **Invariant (review
suggestion): every `resetPixels()` call site must pair it with `releaseGL()` for
the same tile** — after a `texture()` upload `data_` is already freed while
`pixels_loaded_` stays true and `texture_` is non-null, so a CPU-only clear leaves
a stale texture that shadows the re-loaded pixels and leaks the new `data_`.
Documented on the `resetPixels()` declaration.

### 3. Level-selection pure function — new `lod_level_selector.h`

```cpp
/// Return the finest element of @p available_levels that is ≤ the GGGS level
/// matching @p ground_metres_per_pixel (fromCellSize). Falls back to finest if
/// all available levels are finer than ideal, or to coarsest if all are coarser.
inline int selectLodLevel(
  double ground_metres_per_pixel, const std::vector<int>& available_levels);
```

Uses `gggs::Level::fromCellSize()` (include `<marine_autonomy/gggs.h>`; camp already
links marine_autonomy). Note `fromCellSize()` returns a `gggs::Level` *object* —
the int comes from `.level()`. The function is **`inline`** (header-only, included
by both `gggs_tile_layer.cpp` and the test TU — the `gggs_tile_util.h` pattern;
without `inline` it is an ODR multiple-definition link error). No Qt, no globals —
fully testable in `test_gggs_tile.cpp`. This is the generic multi-level wiring: a
future chart layer calls the same function with its ENC scale-ladder levels.

**Input basis (review finding):** the caller must pass **true ground metres per
pixel**, not raw Web-Mercator scene metres — Mercator inflates by ~sec(latitude)
(≈1.37 at Massabesic 43°N, biasing selection up to ~half a level coarser). The
`paint()` caller converts: `mercator_m_per_px * cos(lat_at_viewport_centre)`,
with the latitude recovered from the viewport-centre scene y (inverse Mercator).
ADR-0013 records this basis.

### 4. GggsTileLayer: multi-level tile set — `gggs_tile_layer.h / .cpp`

**State changes:**

- `selected_level_ = -1` (int, GUI-thread only) — the currently active LOD.
- `available_levels_` (std::vector<int>, populated in `loadDirectory()`) — sorted
  ascending list of levels present in this layer.
- `load_viewport_` (QRectF, set before kicking each worker, read by the worker) —
  snapshot of `clip.scene` so the off-thread worker can filter tiles spatially.

**`loadDirectory()` changes:**

After scanning fine tiles from `directory_/`, also scan
`directory_ + "/overviews/"` for tiles using the same `isValueTile()` filter.
All tiles (fine + overview) go into `tiles_`; their `level_` distinguishes them.
Build `available_levels_` as the deduplicated sorted level list across all tiles.
`scene_bounds_` unions the FINEST-level tile extents only (as-built; ADR-0013
§Extent semantics): overview tiles are padded to their coarse GGGS grid cell,
so uniting them would balloon boundingRect / fit-to-extent far beyond the data
footprint.

**`paint()` changes:**

After deriving `clip` via `deriveViewportClip()`:

```
// Ground metres per pixel: Mercator scene metres × cos(lat at viewport centre)
double lat = inverseMercatorLat(clip.scene.center().y());
double metres_per_pixel = (clip.scene.width() / clip.size.width()) * cos(lat);
int target = selectLodLevel(metres_per_pixel, available_levels_);
bool level_changed = (target != selected_level_);
if (level_changed) {
  selected_level_ = target;
  cached_image_ = QImage();
  // As-built (field-verified fix, 2026-07-31): NO eager release here. The
  // outgoing level's tiles stay resident and draw as the backdrop — see
  // itemsIntersecting (stale levels coarse→fine UNDER the selected level) —
  // until the new level's visible set loads; tilesReady() then releases them
  // (resetPixels()+releaseGL() paired, the gggs_tile.h invariant). The
  // original eager release blanked the layer for the whole load — visible
  // zoom flicker in the real-store field verify (ADR-0013 §Progressive
  // refinement).
}
// Snapshot the viewport BEFORE any kick so the worker never reads stale bounds
// (review must-fix: the original draft assigned after loadTiles()).
load_viewport_ = clip.scene;
// Re-kick on level change OR when a pan/zoom exposes intersecting tiles at the
// selected level that have no pixels yet (review must-fix: pure pan previously
// never re-kicked → permanently blank panned-in regions). The unloaded-visible
// check is a cheap O(tiles) scan and doubles as the debounce: no unloaded
// visible tiles → no kick → no abort+join churn during pan storms.
if (load_started_ && (level_changed || hasUnloadedVisibleTiles(clip.scene)))
  loadTiles();
```

`hasUnloadedVisibleTiles(viewport)`: any `tile` with `level() == selected_level_`
∧ extent intersects `viewport` ∧ `!pixelsLoaded()`.

**`loadTilesWorker()` changes:**

Filter predicate: `tile->level() == selected_level_` AND tile extent intersects
`load_viewport_` — **with headless defaults (review must-fix):**
`selected_level_ == -1` means "no level filter" and a null/invalid
`load_viewport_` means "no spatial filter". Both fields are only set by
`paint()`; the ~6 existing headless tests (`test_gggs_render.cpp`,
`test_gggs_band_select.cpp`) drive `waitForLoad()` + `renderImage()` without
ever calling `paint()` — `waitForLoad()` does NOT set `selected_level_` (the
original draft claimed it did; that was wrong). With the no-filter defaults the
headless path loads everything, exactly as today, and every existing test passes
unchanged. Workers abort and re-kick via the existing abort+join mechanism.

**`itemsIntersecting()` changes:**

Add `selected_level_ != -1 && tile->level() != selected_level_` → skip (before
the spatial intersection test, cheapest path). This stops off-level tiles from
reaching the renderer even if they still carry loaded pixels from a prior level;
the `-1` guard keeps the headless/no-selection path unfiltered.

**`tilesReady()` changes:**

Skip range-folding for tiles whose `band() != band_` (existing) OR whose
`level() != selected_level_` when `selected_level_ != -1` (new) — off-level
tiles must not pollute the auto-range. Note (review suggestion): a level
*switch* does not reset `data_min_/data_max_`; the incremental fold only widens
the auto-range across levels. Usually benign (overview values ⊆ fine range by
MEAN-fold construction) — documented in ADR-0013 rather than reset-on-switch.

### 5. Camp ADR-0013 — `docs/decisions/0013-lod-level-selection-demand-driven-load.md`

Record: the level-selection algorithm (`gggs::Level::fromCellSize` → closest
available coarser level); the `overviews/` sidecar consumer contract (ADR-0011 in
unh_marine_autonomy); the demand-driven load pattern (paint → snap viewport →
worker loads only intersecting tiles at selected level); the #172 hook shape
(GggsTileLayer's demand-driven pattern is the template for SonarLiveCacheLayer
evicted-tile reload, which this PR deliberately enables but does not implement).

### 6. Tests

Extend `test_gggs_tile.cpp` (already registered, no CMake change):

- Level-parse coverage: unified into camp#180's `TileLevelParse` at the jazzy merge (valid names + all reject cases).
- `ParseTileLevel_Rejects`: companion tiles, malformed names → -1.
- `SelectLodLevel_PicksCoarsestAvailable`: given levels {0, 7, 13} and a large
  metres_per_pixel (fit-zoom), returns 0.
- `SelectLodLevel_PicksFinestWhenZoomedIn`: metres_per_pixel < L13 cell size →
  returns 13.

Extend `test_gggs_render.cpp` (already registered):

- `OverviewSidecarLoadsAtCoarseLevel`: synthetic two-level store (`dir/13_r_c.tif`
  fine + `dir/overviews/0_0_0.tif` coarse); construct `GggsTileLayer`; assert
  `available_levels_` contains both {0, 13}; fit-zoom selects level 0.
- `DemandDrivenLoadsOnlySelectedLevel`: same synthetic store; force
  `selected_level_ = 13` (via a test-seam method or by constructing with a
  fine-zoom viewport size); `waitForLoad()`; assert the L0 overview tile did NOT
  load pixels (test seam: expose `pixelsLoadedCount(int level) → int`).
- `HeadlessDefaultsLoadEverything` (review must-fix guard): synthetic two-level
  store, NO `paint()` and no test-seam level — `waitForLoad()` then assert tiles
  at BOTH levels loaded pixels (`selected_level_ == -1` ⇒ no filter). This is
  the regression test for the headless no-filter defaults that keep the ~6
  existing tests meaningful.
- `UnloadedVisibleTilesTriggerRekick` (via test seam on
  `hasUnloadedVisibleTiles()`): with level selected and one visible tile's
  pixels reset, the predicate is true; with all visible tiles loaded, false —
  the pan re-kick condition in one unit test.

Keep all existing clip, resolution, and render tests passing (no change to their
paths — guaranteed by the -1/no-viewport no-filter defaults).

## Files to Change

| File | Change |
|------|--------|
| `src/camp_map/raster/gggs_tile_util.h` | Add `tileLevel()` (unified w/ camp#180) |
| `src/camp_map/raster/gggs_tile.h` | Add `level_`, `level()`, `resetPixels()` |
| `src/camp_map/raster/gggs_tile.cpp` | Implement `level_` init, `resetPixels()` |
| `src/camp_map/raster/lod_level_selector.h` | New — `selectLodLevel()` pure function |
| `src/camp_map/raster/gggs_tile_layer.h` | Add `selected_level_`, `available_levels_`, `load_viewport_`; add `pixelsLoadedCount(int)` test seam |
| `src/camp_map/raster/gggs_tile_layer.cpp` | `loadDirectory()` scans `overviews/`; `paint()` ground-metres level selection + level-switch release + viewport-delta re-kick (`hasUnloadedVisibleTiles()`); `loadTilesWorker()` demand-driven filter with -1/no-viewport headless defaults; `itemsIntersecting()` level filter; `tilesReady()` range-fold guard |
| `docs/decisions/0013-lod-level-selection-demand-driven-load.md` | New camp ADR-0013 |
| `test/test_gggs_tile.cpp` | Level-parse + `selectLodLevel()` tests |
| `test/test_gggs_render.cpp` | Overview sidecar + demand-driven load tests |
| `CMakeLists.txt` | `camp_map` links `marine_autonomy::marine_autonomy` PUBLIC (the selector calls `gggs::Level`; previously only `camp_map_ros` had it) |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Status text set to "(loading…)" during demand-driven load (existing mechanism); level selection is automatic but traceable — ADR-0013 documents the math |
| Only what's needed | No eviction loop, no watchlist, no #172 reload for SonarLiveCacheLayer — only the GggsTileLayer demand-driven path |
| A change includes its consequences | `tilesReady()` + `itemsIntersecting()` both updated; all callers of `loadTilesWorker()` guard with existing abort+join; tests extended |
| Improve incrementally | Closes the 3.6 GB open cost + fit-zoom blank; eviction and #172 reload remain clean follow-ups |
| Test what breaks | Level parse, LOD selection math, overview sidecar enumeration, and demand-driven guard are all new test cases |
| Capture decisions | ADR-0013 captures level-selection algorithm and #172 hook shape |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| camp ADR-0007 (RasterFieldSource seam) | Yes | `itemsIntersecting()` adds a level filter; no renderer API change |
| camp ADR-0010 (SonarLiveCacheLayer eviction pyramid) | No new impact | GggsTileLayer pattern is independent; SonarLiveCacheLayer unchanged in this PR |
| camp ADR-0011 (viewport-clip convention) | Yes | `paint()` level selection plugs into the ADR-0011 clip derivation; demand-driven worker snapshots the clip |
| workspace ADR-0001 (capture decisions) | Yes | New ADR-0013 |
| workspace ADR-0002 (worktree isolation) | Satisfied | camp worktree feature/issue-103 already exists |

## Consequences

| If we change… | Also update… | In plan? |
|---|---|---|
| `loadTilesWorker()` filters by level | `tilesReady()` range-fold guard must also filter | Yes |
| `itemsIntersecting()` filters by level | Existing headless tests never call `paint()`, so `selected_level_` stays -1 — the -1 no-filter default keeps them passing unchanged (verify) | Yes — headless defaults in §4 |
| `loadDirectory()` scans `overviews/` | `rescan()` optionally re-scans overviews/ too (follow-up, not in this PR) | No — follow-up |
| `resetPixels()` added to GggsTile | `setBand()` can delegate the clear to it | Yes — `resetPixels()` is the shared clear body |
| `#172` hook shape | ADR-0013 documents the hook cleanly | Yes — ADR-0013 §Consequences |

## Open Questions

- [ ] Should `rescan()` also scan `overviews/` for newly-landed coarse tiles? The initial
  `loadDirectory()` scan picks up overviews that exist at open time; tiles that arrive
  after open require a rescan. Accepted for this PR: `rescan()` covers fine tiles only
  (the `overviews/` sidecar is built by `build_sidescan_overviews` at processing time,
  not live-updated). Follow-up if needed.

## Estimated Scope

Single PR ("Closes #103"). ~250–350 lines across 10 files. One CMake change
(as-built correction): `camp_map` gains the `marine_autonomy::marine_autonomy`
PUBLIC link — the selector calls `gggs::Level::fromCellSize` and only
`camp_map_ros` carried that dependency before. Tests extend already-registered
files.
