---
issue: 118
---

# Issue #118 — WMS GetMap layer support + nowCOAST radar migration

## Issue Review
**Status**: complete
**When**: 2026-07-24 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #118
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Actions
- [ ] Settle the WMS layer granularity design choice (per-tile bbox `GetMap` vs single full-viewport `GetMap`) before implementation — these branch significantly in architecture; record the decision in a new camp ADR.
- [ ] Clarify time-dimension handling for nowCOAST MRMS: fetching available times (e.g. via `GetCapabilities` or MRMS time service) is more complex than a URL parameter and must be scoped explicitly in plan-task.
- [ ] Extend `TileLayerPreset` / persistence schema (ADR-0003 addendum) for WMS-specific fields (at minimum `wms_version`, `crs`; confirm `layer_id` serialization path for WMS type in `persistTileLayer`).
- [ ] Add test coverage for the new WMS layer type: time-dimension fetching, graceful degradation (blank tile on network failure), and the nowCOAST migration path.
- [ ] Confirm the nowCOAST `base_reflectivity_mosaic` runtime field dependency and graceful-degradation story matches the existing `CachedFileLoader` blank-tile fallback contract before plan-task closes scope.

### Checkpoint resolutions (operator, 2026-07-24)

- **WMS granularity**: **per-tile bbox GetMap** — synthesize the slippy-tile
  grid, one GetMap per EPSG:3857 tile bbox, reusing the full MapTiles
  lifecycle (disk cache, #98 eviction, #99/#111 refresh + cache-buster).
- **Time dimension**: **latest frame only** for v1 — omit/default TIME so the
  server returns the newest frame; preserve #99 semantics (5-min refresh,
  cache-busted). No capabilities-time parsing / frame navigation.

## Plan Authored
**Status**: complete
**When**: 2026-07-24 12:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-118/plan.md` at `e90d541`
**Branch**: feature/issue-118 at `e90d541`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-07-24 21:21 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-118/plan.md` at `e90d541`
**PR**: PR-less (`--issue` mode; gh unauthenticated — issue context from the review-issue + checkpoint entries above)
**Verdict**: approve-with-suggestions

All code claims verified against source: the `"wms"` stub returns `nullptr`
(`background_manager.cpp:177`); GEBCO preset already exists as `type="wms"`,
`enabled=false` (`tile_layer_presets.h:124`); `getUrl()` key-dispatch
(`tile_layout.cpp:32`) and `TileAddress::topLeftCorner()`/`scale()` support the
`WMS_BBOX` computation; the cache-buster already joins with `&` when the URL
has a query (`cached_tile_loader.cpp:134`), so refreshing WMS URLs cache-bust
correctly with no new work; `layer_id` already round-trips through
`persistTileLayer` (`background_manager.cpp:277`). Scope, file targeting, and
ADR compliance all Good. No must-fix findings.

### Findings
- [ ] (suggestion) ADR-0012 should record why WMS `version`/`crs` are hardcoded (`VERSION=1.3.0`, `CRS=EPSG:3857`) rather than added as schema fields, per review-issue item 3 — `plan.md:32,45`
- [ ] (suggestion) State the inherited blank-tile graceful-degradation coverage (CachedFileLoader fallback) explicitly instead of silently dropping review-issue items 4/5 — `plan.md:51`
- [ ] (suggestion) Add an implementation-time manual `GetMap` smoke check for both live endpoints (nowCOAST GeoServer, GEBCO mapserv accepting WMS 1.3.0 + EPSG:3857) — not coverable by the pure-logic tests — `plan.md:32`
- [ ] (suggestion, minor) URL template hardcodes `WIDTH/HEIGHT=256` while bbox math uses `tile_width/height` (both 256 via `osm.h` `tile_size`); derive together or note the coupling — `plan.md:32`
- [ ] (suggestion, minor) Note that preset name `nexrad_radar` is retained for persisted-state continuity even though nowCOAST `base_reflectivity_mosaic` is MRMS-based, not NEXRAD — `plan.md:40`

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-24 21:55 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-118 at `1c19451`
**Mode**: pre-push
**Depth**: Deep (reason: new ADR docs/decisions/0012 — Deep promotion trigger)
**Must-fix**: 0 | **Suggestions**: 4
**Round**: 1 | **Ship**: recommended — no must-fix findings; suggestions are low-severity or already-accepted tradeoffs
**Specialists**: Static Analysis (cppcheck + ament_cpplint), Governance, Plan Drift, Claude Adversarial ×2 (Lens A+B, Deep). Local Adversarial skipped (no Ollama server at localhost:11434); Copilot off (default).

### Findings
- [ ] (suggestion) Pre-existing persisted `nexrad_radar` (xyz) keeps retired IEM source on restore — already accepted/documented in plan.md Consequences (informational) — `tile_layer_presets.h:93`, `background_manager.cpp:73`
- [ ] (suggestion) `layer_id`/`base_url` injected unencoded into WMS query; safe (trusted presets, Custom offers xyz/wmts only) — add "presets-only" note before a future Custom-WMS entry — `wms.cpp:26`, `background_manager.cpp:178`
- [ ] (suggestion) No guard for empty `layer_id` → `LAYERS=` malformed request → blank tile; not currently reachable (presets set it; persist skips empty) — `wms.cpp:24`, `background_manager.cpp:178`
- [ ] (suggestion, minor) cppcheck: `generateWmsLayout` `base_url`/`layer_id` could be `const&` (matches osm::generateTileLayout by-value convention) — `wms.cpp:11`

**Note**: ROS 2 build/tests not compiled in this review (heavy colcon build); new tests are pure-logic and reasoned through. Plan adherence full; positive deviation — radar layer_id workspace-qualified (`weather_radar:base_reflectivity_mosaic`) per live smoke check, documented in ADR-0012.
