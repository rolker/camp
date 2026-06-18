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
