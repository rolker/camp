---
issue: 117
---

# Issue #117 — Remove hard-coded tile layers; add preset UI + QSettings persistence

## Issue Review
**Status**: complete
**When**: 2026-07-24 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #117
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Scope Assessment

The issue proposes three tightly-coupled components: (1) remove the four
hard-coded XYZ/WMTS layers from `BackgroundManager::createDefaultLayers()`,
(2) add an "Add background layer" UI action with a preset list, and (3) persist
operator-created layers to QSettings, restoring them on startup exactly as GGGS
tile-sets and rasters already do. These components are intentionally
co-delivered — removing the hard-coded layers without the UI would leave the
operator with no way to add basemaps. The addendum adds bathymetry presets
(NOAA BlueTopo WMTS, GEBCO WMS pending #118), which are a natural extension of
the preset list with no new plumbing required for BlueTopo and a
deferred/greyed-out treatment for GEBCO.

**Well-scoped?** Yes. The deliverable is coherent and completable in a single PR.
The GEBCO/WMS dependency on #118 is explicitly acknowledged; a greyed-out entry
in the preset table avoids splitting the preset list across two issues.

**Right repo?** Yes — `src/camp_map/background/background_manager.{h,cpp}` in
the `camp` project repo. Pure UI/persistence work; no workspace infra touches.

**Dependencies**: #118 (WMS layer type, for GEBCO preset display); #99 (NEXRAD
radar — preserved via its preset, behavior unchanged); independent of #116/#69/#80.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Moves from invisible hard-coded layers to operator-controlled, QSettings-persisted layers. First-run seed (lean: OSM) is written to QSettings immediately, making it operator-controllable from the first restart. |
| Enforcement over documentation | OK | No new rules; existing QSettings pattern extended. |
| Capture decisions, not just implementations | Watch | Two design decisions need recording: (a) the first-run / upgrade-migration seed strategy; (b) the `BackgroundTileLayers` QSettings key schema. Consider a camp ADR addendum to ADR-0003 (GGGS has `GggsTileLayers/dirs` documented in ADR-0005; this deserves equivalent). |
| A change includes its consequences | Action needed | Existing users upgrading from a version with hard-coded layers will have no `BackgroundTileLayers` key in QSettings and will see an empty map unless the first-run seed logic also fires on upgrade (i.e., seed whenever the key is absent, not only on a literal fresh install). The issue's "one-time seed" covers this if the sentinel key is written on upgrade — plan should make this explicit. Tests for the persistence round-trip, fresh-start, and preset-add flow are needed. |
| Only what's needed | Watch | GEBCO/WMS entry in the preset table is speculative — acceptable if it is a data entry only (greyed-out/hidden), not new infrastructure that must be maintained before #118 lands. |
| Improve incrementally | OK | Components are tightly coupled; co-delivering them is the right call. |
| Test what breaks | Action needed | Tests needed: (1) fresh QSettings → first-run seed produces one OSM layer; (2) upgrade with no `BackgroundTileLayers` key → same seed fires; (3) restart with persisted layers → layers restored correctly; (4) preset added via UI → round-trips through QSettings. |
| Workspace vs. project separation | OK | Project repo work only. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| camp ADR-0003 §4 | Yes — directly governs this | §4 specifies backgrounds persist as app-level QSettings. Hard-coded layers predate ADR-0003 implementation; this issue completes the pattern. Already satisfied conceptually by the design. |
| camp ADR-0006 D2 | Informational | ADR-0006 already cites #117 ("follows #117: a default, never a hardcode") — confirming direction is correct and anticipated. |
| workspace ADR-0001 | Watch | The `BackgroundTileLayers` QSettings key schema and first-run seed strategy are design decisions. If they are non-obvious, record them as a camp ADR addendum to ADR-0003 rather than leaving them implicit. |
| workspace ADR-0002 | OK | Already in the correct worktree. |

### Consequences

- `background_manager.cpp` loses the four hard-coded layer blocks and gains a
  QSettings restore loop (mirroring the GGGS/rasters pattern already present in
  the same function).
- A new preset table (header or inline) carries name/type/URL/opacity/visibility/
  refresh-interval for each preset. No new infrastructure for XYZ/WMTS presets;
  GEBCO requires #118.
- Existing users upgrading from a hard-coded-layers version need the first-run
  seed to fire on the first launch after upgrade (key-absent check, not just
  version or "first install" check).
- If a camp ADR is added, update `docs/decisions/` and the ADR index.

### Actions
- [ ] Resolve and document the upgrade-migration / first-run seed strategy: seed fires whenever `BackgroundTileLayers` key is absent (covers both fresh install and upgrade). Capture in the plan or as a camp ADR addendum to ADR-0003.
- [ ] Define the `BackgroundTileLayers` QSettings key schema (name/type/url/opacity/visibility/refresh-interval) explicitly in the plan before implementation.
- [ ] Add tests: fresh-start seed, upgrade-path seed, persistence round-trip, preset-add-via-UI.
- [ ] Decide and document GEBCO/WMS preset treatment: include as a data entry now (greyed-out/hidden until #118) or defer to #118 — record the decision in the plan.

### Checkpoint resolutions (operator, 2026-07-24)

- **Seed strategy**: seed **OSM only**, firing whenever the
  `BackgroundTileLayers` key is absent (covers fresh install and upgrade).
  Upgrading operators re-add OpenSeaMap/NOAA/NEXRAD from presets as wanted.
- **GEBCO/WMS presets**: **include now as inert data entries** (type=WMS,
  greyed-out/hidden until the WMS layer type from #118 exists). Preset table
  stays complete in one place; #118 only enables the type.

## Plan Authored
**Status**: complete
**When**: 2026-07-24 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-117/plan.md` at `4f2d109`
**Branch**: feature/issue-117 at `4f2d109`
**Phases**: single

### Open questions
- [ ] No open questions — all operator checkpoints resolved (2026-07-24). Plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-07-24 18:11 +00:00
**By**: Claude Code Agent (Claude Opus) (in-context — author self-review)

**Plan**: `.agent/work-plans/issue-117/plan.md` at `4f2d109`
**PR**: PR-less (--issue / worktree mode)
**Verdict**: approve-with-suggestions

### Findings
- [ ] (must-fix) De-persistence on layer *removal* is unspecified — `MapTiles` is generic (also used for non-persisted overlays like radar) and removal happens on the layer's own Layers-tree context menu (ADR-0003 §4), not `BackgroundManager`; without a concrete hook, removed tile layers resurrect on restart (GGGS de-persists in the layer's own removal hook, `test_gggs_persistence.cpp:174`). Add a de-persist-on-remove test with the fix — `plan.md:49`
- [ ] (suggestion) QSettings `beginWriteArray` schema makes keyed removal clunky (index-addressed, whole-array rewrite); consider a per-layer keyed group like GGGS `settingsKey()`, or describe the array rewrite-on-remove — `plan.md:33`
- [ ] (suggestion) Capture the `test_map_model.cpp` consequence: confirm it tolerates the 1-layer OSM seed (it asserts only the top-3 user prefix, `test_map_model.cpp:262`), and note it builds `Map` without a test org/app name so the new seed writes into the developer's real QSettings — set a test org/app name or clear — `plan.md:65`
- [ ] (suggestion) WMTS restore must replicate the async ordering/ownership: `Capabilities` parented to `this`, then `setLayoutFromWMTS()`, then `caps->setUrl()` (per `background_manager.cpp:38-41`); persist any non-default layer_id/tile_matrix_set — `plan.md:40`
- [ ] (suggestion) Commit to a short camp ADR-0003 addendum for the `BackgroundTileLayers` schema + seed strategy rather than leaving it conditional (GGGS's equivalent key is documented in ADR-0005) — `plan.md:90`

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-24 18:57 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-117 at `95fe4ff`
**Mode**: pre-push
**Depth**: Deep (reason: 13 files, 200+ lines, ADR-0003 addendum = Deep promotion trigger)
**Must-fix**: 2 | **Suggestions**: 4
**Round**: 1 | **Ship**: continue — must-fix findings present; both mechanical, re-review after fixes

Specialists: Static Analysis (cppcheck; no actionable findings) · Governance · Plan Drift · Claude Adversarial ×2 (Lens A + Lens B). Copilot off (default). Local Adversarial skipped (Ollama server not reachable).

### Findings
- [x] (must-fix) Test-isolation consequence handled for only 1 of ~10 Map-constructing test mains — `Map` ctor now writes the QSettings seed; add `setOrganizationName`/`setApplicationName` to the 8 un-isolated suites (`test_gggs_band_select`, `test_gggs_layer_name`, `test_gggs_render`, `test_gggs_rescan`, `test_map_tiles_eviction`, `test_map_tiles_refresh`, `test_raster_layer_gdal_cleanup`, `test_sonar_live_eviction`) — `test/*.cpp:main()`
- [x] (must-fix) `Capabilities` object leak (cross-pass confirmed, Lens A+B): `new wmts::Capabilities(name, this)` parented to the long-lived BackgroundManager, never deleted; `MapTiles` holds only a non-owning raw ptr — repeatable via WMTS add/remove/re-add. Parent caps to the MapTiles layer or delete after `ready()` — `src/camp_map/background/background_manager.cpp:383`
- [x] (suggestion) `AddFromPresetRoundTrips` never pumps the event loop, so the deferred `MapItem/<settingsKey>` readSettings (the claimed presentation round-trip) is untested — asserts only the immediate setOpacity — `test/test_background_persistence.cpp:889`
- [x] (suggestion) Seed path writes construction params but not the `MapItem/<settingsKey>` presentation group the add path writes; latent if the seed set ever widens past default-presentation OSM — `src/camp_map/background/background_manager.cpp:439`
- [x] (suggestion) De-persist leaves the `MapItem/<settingsKey>` presentation group orphaned (pre-existing GggsTileLayer convention; orphans accumulate, re-add briefly inherits stale opacity/visible) — `src/camp_map/map_tiles/map_tiles.cpp:341`
- [x] (suggestion) Seed is written via a separate QSettings instance then read back through the outer one; correct in-process but pass the outer `settings` in to make ordering explicit — `src/camp_map/background/background_manager.cpp:44`

## Implementation
**Status**: complete
**When**: 2026-07-24 19:15 +00:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-117 at `a45ad71`
**Addressed**: Local Review (Pre-Push) — 2026-07-24 18:57, at `95fe4ff` (verdict changes-requested; 2 must-fix, 4 suggestions)
**Commits**: `da6f2c6`, `0c0225a`, `9c9f8a7`, `6d26c4c`, `a45ad71`

All six open findings from the source review were actioned (none deferred). The
cited line numbers in the review (…:383/:439) were stale — the reviewed file is
258 lines; each finding was verified against the current source before fixing.

### Actions
- [x] (must-fix) `Capabilities` object leak — parented `wmts::Capabilities` to the MapTiles layer instead of the long-lived `BackgroundManager`, so it is destroyed with the layer (fixes the per-add/remove/re-add leak). `src/camp_map/background/background_manager.cpp` (`da6f2c6`)
- [x] (must-fix) Test-isolation for the 8 un-isolated Map-constructing mains — added `setOrganizationName("camp_test")` + a per-suite `setApplicationName` so the new `Map`-ctor QSettings seed writes to a test scope, not the developer's real camp settings. `test/{test_gggs_band_select,test_gggs_layer_name,test_gggs_render,test_gggs_rescan,test_map_tiles_eviction,test_map_tiles_refresh,test_raster_layer_gdal_cleanup,test_sonar_live_eviction}.cpp:main()` (`0c0225a`)
- [x] (suggestion) De-persist orphaned presentation group — `MapTiles::onRemovedFromMap` now removes the `MapItem/<settingsKey>` group alongside the ids/construction entry (settingsKey() still valid pre-detach). `src/camp_map/map_tiles/map_tiles.cpp` (`9c9f8a7`)
- [x] (suggestion) Seed writes presentation group + (suggestion) seed uses the outer QSettings instance — threaded the caller's `QSettings&` (and the target `LayerList`) into `seedDefaultTileLayers`/`persistTileLayer`, and added a shared `persistTileLayerPresentation` helper used by both the add path and the seed, so the seed writes the `MapItem/<settingsKey>` group under the exact key the restored layer reads. `src/camp_map/background/background_manager.{h,cpp}` (`6d26c4c`)
- [x] (suggestion) `AddFromPresetRoundTrips` never pumps the event loop — added `QCoreApplication::processEvents()` after add and after restart, then re-asserted opacity/visible, so the deferred `readSettings` presentation round-trip is genuinely exercised (both live and restored layer). `test/test_background_persistence.cpp` (`a45ad71`)

### Verification
Pre-commit hooks (trailing-whitespace, EOF, merge-conflict, line-ending,
large-file, and the branch guard) passed on every commit. A package build/test
could **not** be run in this worktree: `colcon build --packages-select camp`
fails at CMake `find_package(marine_ais_msgs)` — that dependency is absent from
every layer install here (a pre-existing worktree/environment gap, unrelated to
these changes; CMake errors before any of the edited files are compiled). The
changes were self-reviewed against the current source instead; the re-review
should build in an environment with the lower layers installed.

### Next step
Lifecycle: **Implementation** → **review-code** (re-review the fixes). Hand off to
a fresh-context sub-agent:

    .agent/scripts/dispatch_subagent.sh --mode in-process --issue 117 --skill review-code

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-24 19:29 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-117 at `49ca2bb`
**Mode**: pre-push
**Depth**: Deep (reason: ADR-0003 addendum + cross-cutting persistence/lifecycle change, ~1.2k lines)
**Must-fix**: 0 | **Suggestions**: 4
**Round**: 2 | **Ship**: recommended — 0 must-fix; round-1's 2 must-fix + 4 suggestions all addressed and re-verified correct

Specialists: Static Analysis (cppcheck; no actionable findings — Qt `slots` MOC false positives + sub-threshold STL nits only) · Governance (all principles Pass; ADR-0003/0006/ws-0001/ws-0002 compliant; consequences map complete) · Plan Drift (faithful; one documented beneficial deviation — sentinel-gated seed vs. plan's ids-absent check) · Claude Adversarial ×2 (Lens A logic + Lens B systemic). Copilot off (default). Local Adversarial skipped (Ollama server not reachable).

Round-1 must-fixes re-verified as correctly resolved: Capabilities leak (now parented to the MapTiles layer, `background_manager.cpp:388`) and test QSettings isolation (8 mains set `camp_test` org + per-suite app name).

### Findings
- [x] (suggestion) Dialog OK-enable ignores whether the current preset row is inert (disabled); clicking OK on e.g. GEBCO silently no-ops (defended downstream — nullptr) — `src/camp_map/background/add_tile_layer_dialog.cpp:189`
- [x] (suggestion) Restore loop dedups at creation but never rewrites `BackgroundTileLayers/ids`, so duplicate or non-constructible entries persist (harmless at runtime; can't normally occur) — `src/camp_map/background/background_manager.cpp:63`
- [x] (suggestion) Remove-then-quit race: `aboutToQuit→writeSettings()` on the not-yet-deleted removed layer re-persists the `MapItem/<settingsKey>` presentation group `onRemovedFromMap()` deleted (orphan only; layer does not resurrect; near-impossible via UI; pre-existing convention shared with GggsTileLayer) — `src/camp_map/map_tiles/map_tiles.cpp:341`
- [x] (suggestion) No explicit `settings.sync()` after the seed; deferred `readSettings` correctness rests implicitly on local QSettings destruct-ordering (correct today, fragile to refactor) — `src/camp_map/background/background_manager.cpp:44`

### Next step
Lifecycle: **Local Review** (approved) → push / open PR → **triage-reviews**. All findings are optional low-severity suggestions; none block the push. Hand off to a fresh-context sub-agent after push:

    .agent/scripts/dispatch_subagent.sh --mode in-process --issue 117 --skill triage-reviews

## Implementation
**Status**: complete
**When**: 2026-07-24 19:44 +00:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-117 at `3413ca9`
**Addressed**: Local Review (Pre-Push) — 2026-07-24 19:29, at `49ca2bb` (verdict approved; 0 must-fix, 4 suggestions)
**Commits**: `6764d87`, `546e67e`, `3413ca9`

All four open suggestions from the source review were actioned (none deferred).
Cited line numbers were stale (e.g. `add_tile_layer_dialog.cpp:189` in a 104-line
file); each finding was verified against the current source before fixing.

### Actions
- [x] (suggestion) Dialog OK-enable ignored inert preset rows — `updateFields()` now gates OK on the selected preset being enabled (or Custom), so OK can't be clicked on an inert row (e.g. a WMS entry parked until #118) to silently no-op. `src/camp_map/background/add_tile_layer_dialog.cpp` (`6764d87`)
- [x] (suggestion) Restore loop never rewrote `BackgroundTileLayers/ids` — the loop now builds a normalized id list (dedup + drop entries whose type can't be constructed) and writes it back only when it differs, so stale/duplicate ids don't linger in QSettings. `src/camp_map/background/background_manager.cpp` (`3413ca9`)
- [x] (suggestion) Remove-then-quit race re-persisting the presentation group — added a base-class `removed_from_map_` guard set in `Layer::removeFromMap()` and checked in `MapItem::applicationQuitting()`, so a shutdown that races the pending `deleteLater` skips `writeSettings()` for a removed layer (fixes the shared MapTiles/GggsTileLayer convention at its root, not just MapTiles). `src/camp_map/map/{map_item.h,map_item.cpp,layer.cpp}` (`546e67e`)
- [x] (suggestion) No explicit `settings.sync()` after the seed — added `settings.sync()` immediately after `seedDefaultTileLayers()`, making the deferred `readSettings()` ordering explicit rather than resting on the local QSettings instance's destruct-time flush. `src/camp_map/background/background_manager.cpp` (`3413ca9`)

### Verification
Pre-commit hooks (trailing-whitespace, EOF, merge-conflict, line-ending,
large-file, and the branch guard) passed on every commit; the trailing-whitespace
hook incidentally cleaned pre-existing whitespace on unrelated lines in the map
base files (folded into `546e67e`). A package build/test could **not** be run
here: `colcon build --packages-select camp` still fails at CMake
`find_package(marine_ais_msgs)` — that dependency is absent from every layer in
this worktree (the same pre-existing environment gap the prior Implementation
entry recorded; CMake errors before any edited file is compiled). The changes
were self-reviewed against the current source; the re-review should build in an
environment with the lower layers installed.

### Next step
Lifecycle: **Implementation** → **review-code** (re-review the fixes). Hand off to
a fresh-context sub-agent:

    .agent/scripts/dispatch_subagent.sh --mode in-process --issue 117 --skill review-code

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-24 20:33 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-117 at `830c0d1`
**Mode**: pre-push
**Depth**: Deep (reason: ADR-0003 addendum + cross-cutting persistence/lifecycle change touching shared MapItem/Layer base classes, ~1.3k lines)
**Must-fix**: 0 | **Suggestions**: 3
**Round**: 3 | **Ship**: recommended — 0 must-fix; the 4 round-2 suggestions (dialog OK-gate, id normalization, shutdown-race guard, seed sync) all implemented and independently re-verified correct

Specialists: Static Analysis (cppcheck 2.13; no actionable findings — only Qt `slots` MOC false positives) · Governance (all principles Pass; ADR-0003/0006/ws-0001/ws-0002 compliant; consequences map complete) · Plan Drift (faithful; one documented beneficial deviation — sentinel-gated seed vs. plan's ids-absent check) · Claude Adversarial ×2 (Lens A logic + Lens B systemic — both clean of must-fix). Copilot off (default). Local Adversarial skipped (Ollama server not reachable).

Round-2 suggestion fixes re-verified correct: (1) dialog OK gated off inert preset rows (`add_tile_layer_dialog.cpp` updateFields); (2) restore loop normalizes/rewrites `BackgroundTileLayers/ids` on change; (3) shutdown-race guard `removed_from_map_` added to the shared `MapItem`/`Layer` base — correctly generalizes to every layer type, and a removed-then-re-added layer is a fresh object so its settings still persist; (4) `settings.sync()` after the seed. Lens A/B both confirmed the seed presentation key equals the restored layer's `settingsKey()` and the `wmts::Capabilities` re-parenting fixes the prior leak without a use-after-free.

Build note: `colcon build --packages-select camp` still fails at CMake `find_package(marine_ais_msgs)` (pre-existing worktree env gap, errors before any changed file compiles); static + adversarial review substituted. CI/full-layer build should confirm the new `test_background_persistence` target links and passes.

### Findings
- [ ] (suggestion) `addTileLayerFromPreset()` (public; tests/programmatic) does not reject an empty preset name — UI-defended but would persist an empty ids entry; add an early `if(preset.name.isEmpty()) return nullptr;` — `src/camp_map/background/background_manager.cpp`
- [ ] (suggestion) Custom layer name colliding with an existing/persisted layer is silently refused (nullptr discarded by `addTileLayer()`); dialog closes with no feedback — consider a warning — `src/camp_map/background/background_manager.cpp`
- [ ] (suggestion) `normalized_ids` rewrite is not `sync()`'d unlike the seed; purely defensive (rewrite is idempotent on next launch, self-heals) — `src/camp_map/background/background_manager.cpp`

### Next step
Lifecycle: **Local Review** (approved) → push / open PR → **triage-reviews**. All three findings are optional low-severity suggestions; none block the push. Hand off to a fresh-context sub-agent after push:

    .agent/scripts/dispatch_subagent.sh --mode in-process --issue 117 --skill triage-reviews
