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

## Integrated Review
**Status**: complete
**When**: 2026-07-24 18:08 -04:00
**By**: Claude Code Agent (Claude Fable 5)

**PR**: #176 at `a1a9b77`
**Sources**: 3 (Copilot R1 @ `a1a9b77`, Local Review (Pre-Push) R1, CI rollup)
**Cross-source confirmations**: 1
**CI**: build-and-test pending at triage time; copilot check pass

### Findings
- [x] (cross-confirmed: Copilot + Local Review R1 suggestion) No guard for empty
  `layer_id` in the WMS construction path — a persisted entry read back without
  the key constructs `LAYERS=` requests that always fail (repeated pointless
  traffic on refresh layers). Refuse to construct WMS with empty `layer_id` —
  `src/camp_map/background/background_manager.cpp:188`
- [x] (valid, Copilot) WMS 1.3.0 GetMap template omits the spec-required
  `STYLES` parameter (may be empty); both live endpoints tolerate its absence
  (smoke-checked) but stricter servers reject. Add `&STYLES=` + update the
  required-params test — `src/camp_map/map_tiles/wms.cpp:30`
- [x] (valid-minor, Copilot) `wms.h` uses `std::string` in its public API via a
  transitive `<string>` include from `tile_layout.h`; include `<string>`
  directly — `src/camp_map/map_tiles/wms.h:4`
- [x] (valid-minor, Copilot) `test_wms_url_generation.cpp` calls `sscanf`
  without `<cstdio>` (compiles via transitive includes today) — add the
  include — `test/test_wms_url_generation.cpp:9`

### False positives
- (none — all four Copilot comments verified against local code as real,
  spec-supported, or hygiene-valid)

## Implementation
**Status**: complete
**When**: 2026-07-27 19:19 +00:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-118 at `092318b`
**Addressed**: Integrated Review (When 2026-07-24 18:08 -04:00, PR #176 @ `a1a9b77`)
**Commits**: `c8fb5e5`, `c098a6a`, `ac8818d`, `092318b`

### Actions
- [x] Refuse to construct a WMS layer with empty `layer_id` — early `return nullptr` before build, so a corrupt restore can't spew failing `LAYERS=` GetMap traffic on refresh — `src/camp_map/background/background_manager.cpp:178` (`c8fb5e5`)
- [x] Add spec-required `STYLES=` (empty) to the WMS 1.3.0 GetMap template + extend the required-params test — `src/camp_map/map_tiles/wms.cpp:30`, `test/test_wms_url_generation.cpp:71` (`c098a6a`)
- [x] Include `<string>` directly in `wms.h` (public API uses `std::string`, previously via transitive `tile_layout.h`) — `src/camp_map/map_tiles/wms.h:4` (`ac8818d`)
- [x] Include `<cstdio>` for `sscanf` in `test_wms_url_generation.cpp` (was compiling via transitive includes) — `test/test_wms_url_generation.cpp:9` (`092318b`)

### Checks
- `ament_cpplint` on the four touched files: my added includes introduce no new `include_order`/`line_length` errors; the remaining categories (`legal/copyright`, `build/include_subdir`) are pre-existing repo-wide conventions present on unrelated files (e.g. `osm.h`), not from this pass.
- ROS 2 colcon build/tests not compiled (heavy build; changes are pure-logic hygiene + a guard clause and a test-param addition, reasoned through) — matches the Local Review's convention for this diff.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-27 19:32 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-118 at `ed71cba`
**Mode**: pre-push
**Depth**: Deep (reason: new ADR docs/decisions/0012 — Deep promotion trigger)
**Must-fix**: 0 | **Suggestions**: 2
**Round**: 2 | **Ship**: recommended — no must-fix; R1 integrated-review findings (empty layer_id guard, STYLES=, <string>, <cstdio>) all verified fixed in code; remaining suggestions are known/accepted.
**Specialists**: Static Analysis (cppcheck + ament_cpplint), Governance, Plan Drift, Claude Adversarial ×2 (Lens A+B, Deep). Local Adversarial skipped (no Ollama server at localhost:11434); Copilot off (default).

### Findings
- [ ] (suggestion) `base_url`/`layer_id` injected unencoded into the WMS query; not reachable today (builtin presets only; Custom dialog offers xyz/wmts; persist skips empty ids) — add a presets-only note before a future Custom-WMS entry — `src/camp_map/map_tiles/wms.cpp:26`
- [ ] (suggestion, minor) cppcheck: `generateWmsLayout` `base_url`/`layer_id` could be `const&` (kept by-value to match `osm::generateTileLayout` convention) — `src/camp_map/map_tiles/wms.cpp:11`

**Note**: cpplint findings on `wms.h`/tests (legal/copyright, build/header_guard, build/include_subdir) match the established `osm.h` sibling convention exactly and are repo-wide, not this-PR defects — correctly silenced. ROS 2 colcon build/tests not compiled (heavy build; changes are pure-logic URL assembly + a guard clause, reasoned through and tested pure-logic) — matches the R1 convention for this diff. Plan adherence full; documented positive deviation: radar `layer_id` workspace-qualified (`weather_radar:base_reflectivity_mosaic`) per live smoke check (ADR-0012).

## Integrated Review
**Status**: complete
**When**: 2026-07-27 16:11 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #176 at `4ab991d`
**Sources**: 3 (Copilot R2 @ `4ab991d`, Local Review (Pre-Push) @ `ed71cba`, CI rollup @ `4ab991d`)
**Cross-source confirmations**: 0
**CI**: all-pass (`build-and-test` success, `copilot-pull-request-reviewer` success)

### Findings
- [x] (valid-minor, Copilot R2) Test duplicates the Web-Mercator half-circumference as a
  13-digit literal (`kHalfEarth = 20037508.3427892`) instead of deriving it from
  `web_mercator::earth_radius_at_equator * M_PI` — the same constant
  `osm::generateTileLayout()` uses. Fix: include `map_view/web_mercator.h` (compiles as-is —
  `camp_map` links `Qt5::Positioning` PUBLIC) and make `kHalfEarth` a derived `constexpr`.
  — `test/test_wms_url_generation.cpp:21`

### Carried forward (informational, single-source Local Review @ `ed71cba`, no action this round)
- (suggestion) `base_url`/`layer_id` injected unencoded into the WMS query; not reachable
  today (builtin presets only; Custom dialog offers xyz/wmts; empty `layer_id` now refused)
  — add a presets-only note before a future Custom-WMS entry — `src/camp_map/map_tiles/wms.cpp:26`
- (suggestion, minor) cppcheck: `generateWmsLayout` `base_url`/`layer_id` could be `const&`
  — kept by-value to match `osm::generateTileLayout` convention — `src/camp_map/map_tiles/wms.cpp:11`

### Resolved since R1 (verified in code at `4ab991d`)
- (Copilot R1) Missing spec-required `STYLES=` — present in the template (`wms.cpp:30`) and
  asserted by the test's required-parameter list (`test_wms_url_generation.cpp:72`).
- (Copilot R1 + Local Review, cross-confirmed) Empty `layer_id` not refused — guarded at
  `background_manager.cpp:185-186`.
- (Copilot R1) `wms.h` transitive `<string>` — now included directly (`wms.h:4`).
- (Copilot R1) Test `sscanf` without `<cstdio>` — now included (`test_wms_url_generation.cpp:8`).

### False positives
- (none this round)

## Implementation
**Status**: complete
**When**: 2026-07-28 00:00 +00:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-118 at `ec92b6d`
**Addressed**: Integrated Review (When 2026-07-27 16:11 -04:00, PR #176 @ `4ab991d`)
**Commits**: `ec92b6d`

### Actions
- [x] Derive the WMS test's `kHalfEarth` from `web_mercator::earth_radius_at_equator * M_PI` — the same constant `osm::generateTileLayout()` uses — instead of the repeated 13-digit literal `20037508.3427892`, so the test can't drift from the layout it verifies. Added `#include "map_view/web_mercator.h"` (resolves via the test's `src/camp_map` include dir; `QGeoCoordinate` available since `camp_map` links `Qt5::Positioning` PUBLIC) and `#include <cmath>` for `M_PI` — `test/test_wms_url_generation.cpp:23` (`ec92b6d`)

### Checks
- Numeric equivalence confirmed: `6378137 * M_PI` = `20037508.3427892` (diff ≈ 4.5e-8, well inside the tests' `EXPECT_NEAR` 0.01 tolerance).
- `ament_cpplint` on the touched file: only the pre-existing repo-wide `legal/copyright` convention (also on `osm.h`, noted in prior reviews) — the new includes introduce no `include_order` or other errors.
- ROS 2 colcon build/tests not compiled (heavy build; change is a pure-logic `constexpr` derivation + two direct includes, reasoned through) — matches this diff's established review convention.

### Carried forward (not this round — informational bullets in the source Integrated Review, no `- [ ]` action)
- `base_url`/`layer_id` unencoded in the WMS query (presets-only note deferred to a future Custom-WMS entry) and `generateWmsLayout` by-value params (kept to match `osm::generateTileLayout`) remain single-source suggestions carried by the Integrated Review as plain bullets — no open checkbox, so out of scope for this pass.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-28 12:44 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-118 at `a89006c`
**Mode**: pre-push
**Depth**: Deep (reason: new ADR docs/decisions/0012 — Deep promotion trigger)
**Must-fix**: 0 | **Suggestions**: 3
**Round**: 3 | **Ship**: recommended — no must-fix; this round's only code delta (kHalfEarth derived constant) verified correct; one fresh low-severity systemic suggestion + two accepted carry-forwards remain.
**Specialists**: Static Analysis (cppcheck + ament_cpplint), Governance, Plan Drift, Claude Adversarial ×2 (Lens A+B, Deep). Local Adversarial skipped (no Ollama server at localhost:11434); Copilot off (default).

### Findings
- [ ] (suggestion) WMS error responses (HTTP 200 + XML ServiceExceptionReport) cache-poison non-refreshing layers: `CachedFileLoader` caches any body on `error()==NoError` with no decode gate, so a transient GEBCO backend blip writes an XML body to `<z>/<x>/<y>.png`; display stays blank (graceful) but GEBCO sets no refresh_ms so it never `invalidateCache()`s — persists until #98 LRU eviction. Radar self-heals via 5-min refresh. Cheap mitigation: add `&EXCEPTIONS=BLANK` to the GetMap template; fuller fix (decode-gate the cache write) is cross-cutting — track as follow-up — `src/camp_map/map_tiles/wms.cpp:30`, `src/camp_map/util/cached_file_loader.cpp:97`
- [ ] (suggestion, carried) `base_url`/`layer_id` injected unencoded into the WMS query; not reachable today (Custom dialog offers xyz/wmts only, no wms/layer_id field — Lens B traced; builtin presets only) — add a presets-only note before a future Custom-WMS entry — `src/camp_map/map_tiles/wms.cpp:26`
- [ ] (suggestion, minor, carried) cppcheck: `generateWmsLayout` `base_url`/`layer_id` could be `const&` — kept by-value to match `osm::generateTileLayout` convention — `src/camp_map/map_tiles/wms.cpp:11`

**Note**: This round's code delta vs R2 (`ed71cba`) is a single line — `kHalfEarth` derived from `web_mercator::earth_radius_at_equator * M_PI` (the R2 Integrated-Review action). Verified: numerically correct (6378137·π = 20037508.3427892) and the transitive `<QGeoCoordinate>` include resolves via `camp_map`'s PUBLIC `Qt5::Positioning` link. The Lens B cache-poisoning finding is new (not surfaced in R1/R2 or the two Copilot integrated rounds); verified end-to-end (refreshTiles→invalidateCache at map_tiles.cpp:287 vs GEBCO refresh_ms=0). cpplint findings on changed files (legal/copyright, header_guard, include_subdir, include_order) match the repo-wide osm.h sibling convention — pre-existing, correctly silenced. ROS 2 colcon build/tests not compiled (pure-logic diff; reasoned through) — matches this diff's established convention. Plan adherence full.
