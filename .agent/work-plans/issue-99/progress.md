---
issue: 99
---

# Issue #99 — Add auto-refreshing weather radar tile overlay (stacked layer on the map system)

## Issue Review
**Status**: complete
**When**: 2026-06-18 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet 4.6)

**Issue**: #99
**Comment**: https://github.com/rolker/camp/issues/99#issuecomment-4742140067
**Scope verdict**: well-scoped

### Actions
- [ ] Add unit/integration tests for Phase 2 refresh timer: verify interval fires, cache is invalidated, and memory stays bounded across N refresh cycles (addresses #98 risk).
- [ ] Document tile provider choice (RainViewer vs. NOAA nowCOAST) in the PR with ToS/offline/coverage rationale; consider an ADR if introducing a long-lived external dependency.
- [ ] Verify `CachedTileLoader` network-error path degrades gracefully (no crash/block) when radar tile server is unreachable (field ops concern).
- [ ] Phase 2 PR should be explicitly coordinated with / blocked on #98 (OOM crash) fix — periodic refresh shares the tile/raster code path implicated in the crash.

## Plan Authored
**Status**: complete
**When**: 2026-06-18 14:30 +00:00
**By**: Claude Code Agent (Claude Sonnet 4.6)

**Plan**: `.agent/work-plans/issue-99/plan.md` at `48c63c5`
**Branch**: feature/issue-99 at `48c63c5`
**Phases**: single PR (Phase 1 = NOAA radar WMTS layer; Phase 2 = refresh timer + eviction — both in same PR)

### Open questions
- [ ] Confirm NOAA nowCOAST WMTS endpoint URL before hardcoding — `https://nowcoast.noaa.gov/arcgis/rest/services/nowcoast/radar_meteo_imagery_nexrad_time/MapServer/WMTS` needs live verification at implementation time.
- [ ] Create ADR-0004 for NOAA nowCOAST external dependency — recommend including in the same PR as Phase 1.
- [ ] 5-minute refresh cadence: confirm this is appropriate or make it configurable (hardcoded default for now, follow-up issue to expose as user setting).

## Plan Review
**Status**: complete
**When**: 2026-06-18 00:00 +00:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-99/plan.md` at `48c63c5`
**PR**: PR-less (--issue / dispatched sub-agent mode)
**Verdict**: approve-with-suggestions

### Findings
- [ ] (must-fix) Test (a) "timer fires >=3 times in 350ms wall-clock" is timing-flaky under CI load — assert on timer config (`isActive()`, `interval()`) and a single deterministic fire via direct slot/QMetaObject invocation instead of counting wall-clock fires — `plan.md:110-112`
- [ ] (suggestion) Tests (b)/(c) inspect private `tiles_` ("confirm tiles_ repopulated", count Tile children) but `tiles_`/`setLayout`/`onRefreshTimer` are all private in `map_tiles.h:50-61`. Count via public `QGraphicsItem::childItems()` + `qgraphicsitem_cast<Tile*>`, and invoke `onRefreshTimer` via `QMetaObject::invokeMethod` or a test friend/subclass. Plan should name the access mechanism — `plan.md:113-121`
- [ ] (suggestion) Eviction claim is accurate but incomplete: `setLayout()` deletes all `Tile*` and resets to zoom-0, bounding memory *at each refresh boundary*. Within a cycle, `paint()` still only hides (setVisible(false)) non-visible tiles, never deletes them — so `tiles_` still grows with pan/zoom between refreshes exactly as today. The refresh bounds nothing the existing code didn't; it just resets periodically. State this so the #98 coordination isn't oversold — `plan.md:74-82,152`
- [ ] (suggestion) `invalidateCache()` data-loss risk is bounded: `CachedTileLoader::local_cache_path_` is set to the absolute per-layer subdir `~/.CCOMAutonomousMissionPlanner/map_tiles/<label>` (`map_tiles.cpp:26`), so `QDir(local_cache_path_).removeRecursively()` is correctly scoped. BUT add an explicit guard that `local_cache_path_` is non-empty AND ends in the expected map_tiles subpath before removeRecursively — an empty/misconfigured path would target an unexpected root. Note the actual fetch goes through the global `CachedFileLoader` singleton (cache root `~/.CCOMAutonomousMissionPlanner/`); confirm the relative-vs-absolute path interplay so removal and re-fetch agree — `plan.md:91-100`, `cached_file_loader.cpp:55-63`
- [ ] (suggestion) Network-error graceful degradation confirmed correct: `cached_file_loader.cpp:119-121` logs + deleteLater, never emits dataLoaded → blank tile, no crash/block. Plan's claim is accurate; the "add comment" step is fine but verify no behavior change. — `plan.md:53-60`
- [ ] (suggestion) Plan conflates the #98/#96 path. Radar is a `MapTiles` (Qt QPixmap) layer, NOT a GDAL `RasterLayer` — so it does not touch the #96 GDAL leak path at all, and only shares #98's tile lifecycle. Say "tile path (#98), not GDAL raster (#96)" so the coordination scope is precise — `plan.md:30,173`
- [ ] (suggestion) ADR-0004 for the NOAA dependency is appropriate, not overkill — it's a new long-lived external network dependency with field-ops (offline) implications; matches review-issue's "capture decisions" flag. Next ADR number 0004 is correct (existing: 0001-0003) — `plan.md:163,178`

### Decision notes
- **Both phases in one PR**: acceptable, no hard blocker. Phase 1 alone is a static snapshot (low value) and the bounded-memory tests ARE the #98 coordination artifact, so combining is defensible. The total is ~100-150 LOC. The only risk is that Phase 2 touches the #98-implicated tile lifecycle while #98 is still OPEN — but the eviction + bounded-memory test is the gate, and the radar layer is default-OFF, so it can't worsen a live crash unless an operator enables it. The user's both-in-one decision stands.
- **NOAA endpoint**: correctly flagged as needs-live-verification; graceful failure on 404 is confirmed (blank layer). Acceptable to land with a verified-or-TODO comment.

## Implementation
**Status**: complete
**When**: 2026-06-18 12:30 +00:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Branch**: feature/issue-99
**Commits** (oldest→newest):
- `f07961f` feat(map_tiles): periodic refresh timer + cache invalidation (#99 Phase 2)
- `cf66f3e` feat(background): NOAA nowCOAST weather-radar WMTS overlay (#99 Phase 1)
- `fd2d326` test(map_tiles): refresh-timer config + bounded-memory regression (#99)
- `defc566` docs(adr): ADR-0004 NOAA nowCOAST radar tile-provider dependency (#99)

### What was implemented
- **Phase 1** — NOAA nowCOAST NEXRAD radar layer in `background_manager.cpp::createDefaultLayers()`, mirroring the NOAA charts WMTS block (`wmts::Capabilities` + `setLayoutFromWMTS`). Default `setVisible(false)`, `setOpacity(0.65)`, `setRefreshInterval(5*60*1000)`.
- **Phase 2** — `MapTiles::setRefreshInterval(int msec)` + owned `QTimer` + private `onRefreshTimer()` slot in `map_tiles.{h,cpp}`. `onRefreshTimer` calls `tile_loader_->invalidateCache()` then `setLayout(tile_layout_)` + `update()`. `msec<=0` disables. `CachedTileLoader::invalidateCache()` added to `cached_tile_loader.{h,cpp}` with a guard (path non-empty AND under `/map_tiles/` before `removeRecursively()`; never the global cache root). Graceful network-error path confirmed with a comment in `cached_file_loader.cpp` (no behavior change).
- **Tests** — `test/test_map_tiles_refresh.cpp` (registered in `CMakeLists.txt`): 5 tests — timer config (interval/active/single-shot=false/disabled-on-0/opt-in), one deterministic refresh via `QMetaObject::invokeMethod(onRefreshTimer)`, and bounded `Tile`-child count across 10 refresh cycles (the #98 regression). Tile count via public `childItems()` + `qgraphicsitem_cast<Tile*>`. Honored the plan-review must-fix: no wall-clock fire counting.
- **ADR** — `docs/decisions/0004-noaa-nowcoast-radar-dependency.md`.

### Build / test
- Built `camp` via `./ui_ws/build.sh camp` — **success** (only pre-existing codebase warnings; one initial test-compile error from a missing `map/layer_list.h` include was fixed).
- `test_map_tiles_refresh` ran via `colcon test` — gtest XML shows **5 tests, 0 failures, 0 errors**.

### Residual risks / TODOs
- **NOAA endpoint NOT verified live** — hardcoded `https://nowcoast.noaa.gov/arcgis/rest/services/nowcoast/radar_meteo_imagery_nexrad_time/MapServer/WMTS` with an explicit `// TODO: confirm endpoint` in `background_manager.cpp`. A 404 degrades gracefully (blank layer), so landing with the TODO is acceptable; confirm before operational reliance.
- Refresh cadence (5 min) is hardcoded; a follow-up could expose it as a user setting.
- Honest scope note (per plan review): `setLayout()` bounds memory only AT each refresh boundary; within-cycle pan/zoom tile accumulation is unchanged from today.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-18 14:06 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: changes-requested

**Branch**: feature/issue-99 at `703c74f`
**Mode**: pre-push
**Depth**: Deep (reason: new ADR + tile-lifecycle/concurrency surface coordinating with the #98 OOM path)
**Must-fix**: 0 | **Suggestions**: 5

Scope: diff vs `origin/jazzy` only (src/camp2 + test + ADR + work-plans). Static
analysis: no camp lint profile (no pre-commit/CI in repo) — content review only.
Claude Adversarial: 2 passes (Lens A logic + Lens B systemic). Copilot: off (default).
Host pre-verified: colcon build OK, test_map_tiles_refresh 5/5.

No blocking must-fix. The implementation closely follows plan.md, commit identity is
correct (all `Claude Code Agent`), ADR-0004 is well-formed and correctly numbered,
atomic commits. Verdict is changes-requested only to land the should-fixes below
(honesty/defense-in-depth) before push, per the Quality Standard — none require redesign.

### Findings
- [ ] (should-fix) Stale-pixmap on refresh: `onRefreshTimer` re-applies the SAME `tile_layout_` object, so `TileAddress::operator==` (compares `layout_` pointer) does NOT reject an in-flight pre-refresh pixmap landing on the rebuilt same-address tile — a stale radar frame can paint for up to one cycle until the re-requested fetch overwrites it. Self-corrects within the cycle; narrow window at 5-min cadence. Consider a per-refresh layout generation/epoch so the guard rejects pre-refresh replies — `src/camp2/map_tiles/map_tiles.cpp:154,168-176`, `src/camp2/map_tiles/tile_address.cpp:44`
- [ ] (should-fix) Honesty gap: ADR + `onRefreshTimer` comment claim re-fetching each cycle yields time-varying radar, but that depends on the (unverified, TODO) time-aware nowCOAST endpoint resolving to "latest". If the WMTS template pins a timestamp, refresh re-serves the same frame. State this dependency alongside the existing endpoint TODO — `docs/decisions/0004-noaa-nowcoast-radar-dependency.md` (Consequences), `src/camp2/map_tiles/map_tiles.cpp:145-151`
- [ ] (should-fix) `invalidateCache()` guard is a free substring (`contains("/map_tiles/")`) not a verified-root prefix. Current call sites pass a hardcoded literal label so practical risk is nil, but a prefix check vs the resolved cache root (`QDir(cache_root).absolutePath()+"/map_tiles/"`, startsWith) is a cheap permanent hardening per the Quality Standard ("field configs change under pressure") — `src/camp2/map_tiles/cached_tile_loader.cpp:65`
- [ ] (suggestion) ADR-0004 Consequences should note that ENABLING radar reintroduces #98-class within-cycle tile accumulation on a second layer (default-OFF makes this conditional, not eliminated); the `MemoryStaysBounded...` test name/assertion overstates coverage — it guards the refresh-boundary reset, not the within-cycle `paint()` accumulation (honestly caveated in the test header + code comment, but the assertion message says "the #98 leak") — `test/test_map_tiles_refresh.cpp:149-164`, `docs/decisions/0004-noaa-nowcoast-radar-dependency.md`
- [ ] (suggestion) Minor: `setRefreshInterval` disable comment says "tear down the timer" but only `stop()`s (correct behavior, retains for re-enable — fix comment); `invalidateCache` ignores `mkpath()` return (self-heals via `downloadFinished` re-mkpath, but inconsistent with checked mkpath elsewhere) — `src/camp2/map_tiles/map_tiles.cpp:128`, `src/camp2/map_tiles/cached_tile_loader.cpp:76`

## Implementation (Pre-Push Review Fixes)
**Status**: complete
**When**: 2026-06-18 14:20 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Branch**: feature/issue-99
Addresses the 5 should-fix / honesty findings from the Local Review (Pre-Push)
entry above. No redesign.

**Commits** (oldest→newest):
- `fc15d44` fix(map_tiles): per-refresh epoch rejects stale pre-refresh pixmaps (#99)
- `b4d97c6` fix(map_tiles): harden invalidateCache with resolved-root prefix check (#99)
- `b7af79b` docs(adr)+test: honesty caveats on refresh freshness and #98 scope (#99)

### What was fixed
1. **Stale-pixmap on refresh (epoch)** — Added a `quint64 layout_epoch_` to
   `MapTiles`, incremented in `setLayout()`, and carried into every `TileAddress`
   minted in `paint()` and `setLayout()`. `TileAddress` gains a `quint64 epoch_`
   ctor arg (default 0); it is compared in `operator==` (identity) but deliberately
   NOT in `operator<` (ordering). Effect: across a refresh, `tiles_.find()`
   (`operator<`, epoch-agnostic) still locates the rebuilt same-position tile, but
   the `tileLoaded` guard `tiles_[addr]->address() == addr` (`operator==`, now
   epoch-aware) rejects the pre-refresh in-flight pixmap. Non-refreshing layers
   keep epoch 0 → matching unchanged for OSM/NOAA. Files: `tile_address.{h,cpp}`,
   `map_tiles.{h,cpp}`.
2. **Honesty caveat on time-varying radar** — Added to the `onRefreshTimer` comment
   (`map_tiles.cpp`) and ADR-0004 Consequences: "fresh frame each cycle" depends on
   the unverified, TODO nowCOAST endpoint resolving to LATEST; a timestamp-pinned
   template re-serves the same frame. Tied to the existing endpoint TODO.
3. **`invalidateCache()` prefix hardening** — Replaced `contains("/map_tiles/")`
   with a strict `startsWith(<resolved cache root>/.CCOMAutonomousMissionPlanner/map_tiles/)`
   prefix check; bails safely (no-op + qDebug) otherwise; non-empty check retained.
   File: `cached_tile_loader.cpp`.
4. **ADR Consequences note** — ADR-0004 now states enabling radar reintroduces
   #98-class within-cycle (pan/zoom) tile accumulation on a second layer; default-OFF
   makes it conditional, not eliminated.
5. **Test assertion honesty** — Renamed `MemoryStaysBoundedAcrossManyRefreshes` →
   `RefreshBoundaryResetsTileSet` and softened the assertion message to reflect that
   it guards the refresh-boundary reset, not within-cycle `paint()` growth.

Not in scope (the 5th *review* suggestion — "tear down" comment + unchecked
`mkpath()` return) was not in the assigned fix list; left as-is.

### Build / test
- `./ui_ws/build.sh camp` — **success** (only pre-existing codebase warnings).
- `test_map_tiles_refresh` gtest XML (`ui_ws/build/camp/test_results/camp/test_map_tiles_refresh.gtest.xml`):
  **5 tests, 0 failures, 0 errors** (regenerated this run; includes the renamed test).
- Local uncrustify-0.78 mass-fail not run/not relevant (documented false positive;
  verified via gtest XML per instructions).

### For re-review to double-check
- Epoch identity semantics: confirm `operator<` intentionally excludes `epoch_`
  (map lookup must stay epoch-agnostic) while `operator==` includes it. The two
  must disagree only on epoch — that's the whole mechanism.
- `quint64` availability in `tile_address.h` / `map_tiles.h` (pulled in via QPoint
  → QtGlobal); build confirms it compiles.
- `invalidateCache` resolved-root prefix matches how `MapTiles` builds the path
  (`QDir::home().filePath(".CCOMAutonomousMissionPlanner/map_tiles/"+label)`).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-18 14:40 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: approved

**Branch**: feature/issue-99 at `51c9297`
**Mode**: pre-push (re-review of fixes `3873f63..HEAD`: `fc15d44`, `b4d97c6`, `b7af79b`, `51c9297`)
**Depth**: Deep (reason: tile-lifecycle/concurrency surface coordinating with the #98 OOM path; ADR-touching)
**Must-fix**: 0 | **Suggestions**: 2

Focused re-review of the 4 fix commits that addressed the prior changes-requested
entry. Scope: diff vs `origin/jazzy` only (camp2 map_tiles + cached loaders + ADR +
test + work-plans). Static analysis: no camp lint profile (no pre-commit/CI in repo)
— content review only. Host pre-verified: colcon build OK; `test_map_tiles_refresh`
gtest XML 5/5 (timestamp 14:14:50, confirmed this run). IDE/clang "QObject/QPoint not
found" are LSP Qt-path false positives — ignored.

### Prior findings — all 5 genuinely resolved
1. **Stale-pixmap epoch fix** — RESOLVED. `quint64 layout_epoch_` bumped in every
   `setLayout()` (`map_tiles.cpp:100`), carried into every `TileAddress` minted in
   `paint()` (`:75`) and `setLayout()` (`:108`). `operator==` now includes `epoch_`
   (`tile_address.cpp:53-54`); `operator<` deliberately excludes it (`:57-62`).
   (a) Internally consistent: `operator<` was ALREADY layout-agnostic pre-fix (the
   `tiles_` map has keyed on zoom/y/x only since before this work — see the existing
   `tileLoaded` comment); the epoch addition extends that established split, so
   strict-weak-ordering of `std::map<TileAddress,Tile*>` is unchanged and lookups are
   not broken. (b) Guard works: `tileLoaded` does `find()` (epoch-agnostic → locates
   the rebuilt same-position tile) then `address() == tile_address` (epoch-aware →
   pre-refresh reply with stale epoch compares unequal and is rejected),
   `map_tiles.cpp:181-186`. (c) Non-refresh layers unaffected: matching is relative
   within a layout generation, so OSM/NOAA behave exactly as before.
2. **Honesty caveat (freshness)** — RESOLVED. ADR-0004 Consequences gains an explicit
   "fresh frame only if endpoint is time-aware (latest), not timestamp-pinned" note
   tied to the endpoint-confirmation TODO; mirrored at `onRefreshTimer`
   (`map_tiles.cpp:157-162`).
3. **invalidateCache() guard** — RESOLVED. Free `contains("/map_tiles/")` replaced
   with a resolved-root prefix check: `allowed_prefix = QDir(home/.CCOMAutonomous-
   MissionPlanner/map_tiles).absolutePath()+"/"`, `normalized.startsWith(allowed_prefix)`
   (`cached_tile_loader.cpp:70-78`). Verified the resolved root matches how `MapTiles`
   builds the path (`map_tiles.cpp:27`): legitimate `.../map_tiles/<label>` passes; the
   bare global root and `.../map_tiles` (no trailing slash) are refused. No
   legitimate-path rejection, no real-file miss.
4. **ADR Consequences (#98 on second layer)** — RESOLVED. ADR-0004 states enabling
   radar reintroduces #98-class within-cycle pan/zoom accumulation on a second layer;
   default-OFF is conditional, not eliminated.
5. **Test honesty** — RESOLVED. `MemoryStaysBoundedAcrossManyRefreshes` →
   `RefreshBoundaryResetsTileSet` (`test/test_map_tiles_refresh.cpp:157`); header +
   assertion message reworded to claim only the refresh-boundary reset, not within-cycle
   growth. gtest XML reflects the renamed test, 5/5.

### Findings (this round)
- [ ] (suggestion) No dedicated unit test exercises the new epoch stale-pixmap path
  (the `operator==`-epoch reject in `tileLoaded`). The mechanism is verified by code
  reasoning + a clean build, but a regression test would need to simulate an in-flight
  pre-refresh reply landing after a `setLayout` swap. Acceptable to defer (hard to test
  the async race deterministically), but worth a follow-up — `test/test_map_tiles_refresh.cpp`
- [ ] (suggestion) Minor plan drift: `plan.md` "Files to Change" doesn't list
  `tile_address.{h,cpp}` or mention the epoch mechanism, since those arose in the
  pre-push review round (post-plan). Fully captured in this progress.md timeline; could
  add a one-line note to the plan's Files-to-Change for completeness — `.agent/work-plans/issue-99/plan.md:133-144`

### Governance / regressions
- Commit identity clean: all 14 branch commits authored by `Claude Code Agent`
  (`roland+claude-code@ccom.unh.edu`).
- Atomic commits: epoch / guard-hardening / ADR+test / progress are each one logical
  change.
- No scope creep: diff is confined to the radar overlay surface; no wider-workspace edits.
- No regressions introduced by `fc15d44..51c9297`: `operator<` unchanged, lookups intact;
  the prior 5th *review* item (the "tear down" comment + unchecked `mkpath()` return) was
  explicitly out of the assigned fix list and is a pre-existing cosmetic nit, not a
  regression.

**Verdict: approved.** The 4 fix commits resolve all 5 prior findings without papering
over them, the mechanism is sound and internally consistent, and the two new suggestions
are non-blocking. Clear to push.
