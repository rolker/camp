---
issue: 121
---

# Issue #121 — Unified raster render abstraction + CAMP live tile cache (anti-entropy)

## Issue Review
**Status**: complete
**When**: 2026-06-28 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #121
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: needs-splitting

### Actions
- [ ] Split into two PRs: Part A (RasterFieldSource interface + adapters) and Part B (live tile cache) — Part B depends on #230 (anti-entropy transport) and #44/#68 (discover/opt-in); verify those are merged before starting Part B.
- [ ] Verify that the `RasterFieldSource` interface doesn't merely rename existing `GggsTileLayer` internals — the abstraction should emerge from needing a second implementation (live cache), not be introduced speculatively ahead of it. Consider whether the interface belongs in the same PR as Part A or only in Part B's PR.
- [ ] Write a camp project ADR for Part B's persistence contract: in-memory render + write-through to disk + anti-entropy reconcile + "display-grade preview vs. data of record" distinction. This is a significant design decision for the project.
- [ ] Clarify how the live cache layer presents to the operator: does it appear in the Layers tree? How is its "live/preview" status visually distinguished? Align with ADR-0005's browse/compose split.
- [ ] For Part B acceptance: add a test or documented scenario for the simulated-downtime gap (request + prune) that can run without a live boat connection.

## Plan Authored
**Status**: complete
**When**: 2026-06-28 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-121/plan.md` at `edf9946`
**Branch**: feature/issue-121 at `edf9946`
**Phases**: single

### Open questions
- [ ] Discover vs. opt-in: camp#44/#68 not yet implemented; auto-spawn (GridManager pattern) proposed — confirm with operator before implementing.
- [ ] Multi-band write-through: which bands to persist (all received vs. depth-only)? Propose: all received bands.
- [ ] Cache size limits: no built-in eviction beyond prune-on-absence; flag if operator expects a bounded cache.

## Plan Review
**Status**: complete
**When**: 2026-06-28 17:55 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-121/plan.md` at `edf9946`
**PR**: PR-less (--issue mode)
**Verdict**: approve-with-suggestions

The plan is well-grounded in the actual codebase (reconciler API, `ros::Layer`/`Node`,
`GggsTile::loadPixels`, manager wiring, QoS choices all verified) and addresses every
review-issue finding. The one must-fix is a completeness gap: the Files-to-Change table
omits two files (`package.xml`, `item_types.h`) that the plan's own Consequences table
says must change. Suggestions concern threading clarity, the discover-vs-opt-in blocker,
and minor convention/version-semantics points.

### Findings
- [ ] (must-fix) Files-to-Change table omits `package.xml` (needs `<depend>marine_tiled_raster_store</depend>` — confirmed absent) and `item_types.h` (needs a new `SonarLiveCacheLayerType` in its `enum ItemType` — the Consequences row says "in sonar_live_cache_layer.h" but the enum lives in `src/camp_map/map/item_types.h`). Reconcile the two tables. — `plan.md:214-227, 252-257`
- [ ] (suggestion) Consequences row "node.h forward-declares the manager" is unnecessary: existing `MarkersManager`/`GridManager` are created in `node.cpp` (`new XManager(this)`) and are not forward-declared in `node.h`. Drop the row or confirm it's actually needed. — `plan.md:256`
- [ ] (suggestion) Resolve Open Question #1 (auto-spawn vs. operator opt-in) with the operator *before* implementation — it determines `SonarLiveCacheManager`'s spawn behavior, a structural decision, not a detail to defer. Also note this bypasses ADR-0005 §2's generic `CatalogSource` seam (which explicitly anticipated topic-discovery consumers like camp#44/#68); the deferral is defensible but should be stated as a conscious ADR-0005 tension in ADR-0006. — `plan.md:182, 261`
- [ ] (suggestion) `TileCatalogReconciler` is explicitly **not thread-safe** (tile_catalog.hpp:46). The plan should state that `reconcile()`/`markHave()`/`drop()`/`applyPatch()` all run on a single thread (warm-load constructor + ROS callbacks marshalled to the GUI thread via queued signal per ADR-0001), so the ROS callback must only copy+emit, never touch `reconciler_` directly. — `plan.md:122-135, 245`
- [ ] (suggestion) Specify version semantics consistency: newest-wins compares `header.stamp` against `versionOf()` (`TileVersion` = int64), and prune is gated on catalog `generation_time` (also int64). State the single conversion (e.g. stamp→nanoseconds) used for `markHave`, the stale check, and the timestamp gate so they're comparable. — `plan.md:128-130, 211-213`
- [ ] (suggestion) New manager file placement: existing managers live in `ros/` subdirectories (`ros/grids/`, `ros/markers/`, `ros/geometry/`); the plan puts `sonar_live_cache_manager.{h,cpp}` directly in `ros/`. Consider a subdir for consistency (cosmetic). — `plan.md:161, 223-224`

## Implementation
**Status**: complete
**When**: 2026-06-28 00:00 +00:00
**By**: Claude Opus

**Issue**: #121 (Part B — CAMP live tile cache; Part A / RasterFieldSource deferred to camp#134)
**Branch**: feature/issue-121
**Scope**: Part B only, with the operator-decided **opt-in tile-stream** activation model.

### Files added
- `docs/decisions/0006-live-tile-cache-persistence.md` — ADR: in-memory render + crash-safe write-through + warm-load + anti-entropy, and the **discovery-automatic / tile-stream-opt-in** decision (D5) stated as a conscious ADR-0005 tension.
- `src/camp_map/ros/live_coverage/sonar_live_tile.{h,cpp}` — in-memory dequantized Float32 tile (`applyPatch` generic dequantize `value = raw*scale+offset`, GGGS-cell→north-up flip, incremental auto-range), GDAL warm-load (`loadFromGeoTiff`/`loadCacheDir`, all bands + names + GridIndex-from-geotransform) and write-through (`writeToGeoTiff`), plus the node-boundary conversions (`toNanoseconds`, `gridIndexFromTileIndex`/`tileIndexFromGridIndex`, `toReconcilerCatalog`).
- `src/camp_map/ros/live_coverage/sonar_live_cache_layer.{h,cpp}` — `ros::Layer` subclass: GL render **duplicated from GggsTileLayer** (marked `// NOTE: shader duplicated; unify via RasterFieldSource camp#134`), reconcile/request/prune, write-through (QtConcurrent, atomic temp+rename), warm-load, opt-in **Enable/Disable live coverage** toggle, per-source QSettings (colormap/band/enabled).
- `src/camp_map/ros/live_coverage/sonar_live_cache_manager.{h,cpp}` — `TopicsManager`-filtered discovery (`SonarVisualizationTile`), spawns one inactive layer per source base namespace (deduped).
- `test/test_sonar_live_cache.cpp` — headless (no Qt loop / no ROS node / no boat): warm-load round-trip, patch-apply/dequantize, downtime-gap reconcile (seed A,B,C → catalog A,B,D → request={D}, prune={C} → converge), prune timestamp-gate.

### Files changed
- `src/camp_map/map/item_types.h` — **(must-fix)** added `SonarLiveCacheLayerType` to `enum ItemType`.
- `package.xml` — **(must-fix)** added `<depend>marine_tiled_raster_store</depend>`.
- `CMakeLists.txt` — `find_package(marine_tiled_raster_store)`; new sources in `CAMP_MAP_ROS_SOURCES`; `marine_tiled_raster_store` + `${GDAL_LIBRARY}` on `camp_map_ros`; new `test_sonar_live_cache` target.
- `src/camp_map/ros/node.cpp` — wired `SonarLiveCacheManager` alongside `GridManager`.
- `.agent/work-plans/issue-121/plan.md` — synced to the opt-in model, must-fix table additions, and `ros/live_coverage/` placement; Open Questions resolved.

### Operator decision applied (overrides plan auto-spawn)
**Discover automatically; subscribe to `coverage_tiles` + publish `TileRequest` only on opt-in.** Discovery spawns a *discovered-but-inactive* `Live Coverage [<src>]` layer (status `(available)`); the inactive layer subscribes only to the cheap transient-local `coverage_catalog`. "Enable live coverage" warm-loads the disk cache, subscribes to the best-effort tile stream, starts request/prune, and persists `live_enabled` per source (settingsKey on source ns) → an enabled source re-subscribes on warm restart; a never-enabled one stays passive. Serves #71.

### Must-fix + suggestions addressed
- must-fix: `package.xml` + `map/item_types.h` reconciled with the plan's Consequences. ✔
- Thread-safety invariant (ADR-0001/ADR-0006 D4): ROS callbacks only copy the message + `QMetaObject::invokeMethod(..., Qt::QueuedConnection)` to the GUI thread; the GUI-thread slot does all reconcile/markHave/drop/applyPatch/write-through; warm-load runs GUI-thread pre-subscribe. Documented in the layer header. ✔
- Version semantics: one int64 = **nanoseconds since epoch** for markHave, the newest-wins stale check, and the prune timestamp-gate (single `toNanoseconds`). ✔
- Dropped the unnecessary "node.h forward-declares the manager" step (managers are `new XManager(this)`). ✔
- Manager (and, for ROS-free-core integrity, the tile + layer) placed under `ros/live_coverage/`. ✔

### Secondary OQ defaults applied
- Write-through **all received bands** (band name = GDAL band description). ✔
- **No eviction** beyond prune-on-absence; `// TODO(camp): eviction-by-area follow-up` note left in the layer. ✔

### Build + test (verbatim)
- Lower layers built first (system-available deps from `/opt/ros/jazzy`): `colcon build --packages-up-to marine_tiled_raster_store marine_ais_msgs marine_nav_interfaces marine_nav_tasks` → all finished (only pre-existing `marine_autonomy` warnings).
- `./ui_ws/build.sh camp` → `Summary: 1 package finished [1min 59s]` (clean; only pre-existing camp warnings).
- `./ui_ws/test.sh camp` → `Summary: 129 tests, 0 errors, 0 failures, 4 skipped` (the 4 skips are the offscreen-GL render tests that skip in-container by design).
- `test_sonar_live_cache` → `[ PASSED ] 4 tests` (PatchApplyDequantize, WarmLoadRoundTrip, DowntimeGapReconcile, PruneTimestampGate).

### Notes / deviations
- New TUs live in `ros/live_coverage/` (not `raster/` as the plan first listed): they depend on `marine_interfaces` + `marine_tiled_raster_store`, so they belong in `camp_map_ros`; `camp_map` stays ROS-free (ADR-0002 / CMake layering). Recorded in ADR-0006 + plan.
- Warm-load reads the multi-band GeoTIFF directly via GDAL (recovering band names from descriptions + GridIndex from the geotransform, the `marine_tiled_raster_store::loadTile` convention) rather than literally calling the single-band `GggsTile::loadPixels()`, since write-through-all-bands needs all bands + names in one pass.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-28 19:14 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-121 at `470d2c9`
**Mode**: pre-push
**Depth**: Deep (reason: ~1700 LOC new C++ + new ADR + concurrency/file-I/O across the ROS↔GUI boundary)
**Must-fix**: 1 | **Suggestions**: 2
**Round**: 1 | **Ship**: continue — one genuine concurrency-correctness must-fix (same-tile write-through race) warrants the fix + a re-read

### Findings
- [ ] (must-fix) Write-through launches uncoalesced concurrent `QtConcurrent` workers that race on a shared per-tile `<stem>.tif.tmp` (every patch calls `setFuture` unconditionally; deviates from the `GggsTileLayer` isRunning-guard / `GridMap` coalesce convention; defeats atomic temp+rename; dtor joins only the latest future) — `src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp:380-384`
- [ ] (suggestion) Stale auto-range after prune: prune path runs `recomputeBounds`+`foldAutoRange` but not `resetAutoRange`, so a pruned tile's extreme lingers in `data_min_/data_max_` — `src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp:365-371`
- [ ] (suggestion) Reset `tile_sub_`/`catalog_sub_` at the top of the dtor (before `waitForFinished`) for teardown symmetry / to close the executor-thread TOCTOU window — `src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp:131-137`

### Notes
- Governance clean: ADR-0006 well-formed (Deep ADR-add trigger); ADR-0001/0002/0005 + #117/#71 all Pass; consequence updates (`package.xml`, `CMakeLists.txt`, `item_types.h`, `node.cpp`) all done. Plan-review must-fixes resolved.
- Unbounded in-memory/on-disk tile growth is a consciously deferred, ADR-0006-documented, TODO'd follow-up (eviction-by-area) — acceptable for a display-grade preview cache; not counted as a finding.
- Static analysis: camp configures no `ament_lint`/`cpplint`; build + 129 tests reported clean at implementation. New files match camp conventions. No new lint findings.
