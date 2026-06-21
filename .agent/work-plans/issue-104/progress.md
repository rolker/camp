---
issue: 104
---

# Issue #104 — Stores layer redesign: store-tree browser → flat selectable display layers (band + colormap + order)

## Issue Review
**Status**: complete
**When**: 2026-06-21 01:06 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Issue**: #104
**Comment**: https://github.com/rolker/camp/issues/104#issuecomment-4760974444
**Scope verdict**: needs-splitting

### Actions
- [ ] Split into ~3 PRs: (3a) browser/manager + flat selectable layers, (3b) band-select + per-band colormap, (3c) GggsTileLayer compositing test — or at minimum land band-selection as its own commit-set behind the structural change. (Principle: Improve incrementally; Only what's needed)
- [ ] Write an ADR (or extend ADR-0003 §4 split-persistence) recording the browse-vs-compose split AND the persistence-format change (store roots → selected flat layers + band + colormap + order). This reverses camp#90's "store folders straight into the layers tree" design — ADR-0001 + ADR-0003 trigger. (Principle: Capture decisions; ADR-0001/0003 Action needed)
- [ ] Reframe item (4) "draw order untested": the Map model reorder path is ALREADY tested (test_map_model.cpp: ReorderKeepsModelValid, DropReordersToTargetRow, #84). What's untested is GggsTileLayer COMPOSITING — stack-order + opacity through the offscreen-FBO QPainter::drawImage paint path (gggs_tile_layer.cpp:499). Target the new test there (e.g. sidescan over bathy). (Principle: Test what breaks — Action needed)
- [ ] Band selection (item 3): GggsTile/loadPixels hardwire GetRasterBand(1) (gggs_tile.cpp:34,77); add a band index param plumbed through loadPixels + auto-range; reuse the existing camp#90 colormap LUT (setColormap). Add coverage rendering a non-1 band of a ≥2-band GeoTIFF. (Principle: Test what breaks)
- [ ] Reuse, don't rebuild: keep each flat layer a GggsTileLayer so camp#90 colormap + camp#102 lazy/async/default-off/visibility-persist carry over; add only the band-index parameter.
- [ ] State the persistence migration explicitly in the PR: old QSettings GggsStores/roots (+ per-layer visible) honored vs reset; update background_manager.cpp createDefaultLayers restore path (lines 77-86) and the onRemovedFromMap drop-on-remove analogue for selected layers; update .agents/README.md persistence notes. (Principle: A change includes its consequences)

## Plan Authored
**Status**: complete
**When**: 2026-06-21 01:38 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-104/plan.md` at `8262e2f`
**Branch**: feature/issue-104 at `8262e2f`
**Phases**: single (piece 3a; band-select #108 + compositing test #109 are separate follow-ups)

### Open questions
- [ ] Browser entry-point UX: seed via existing "pick store-root" QFileDialog then browse (plan's choice), vs. a roots-managing browser ("Add store root…" inside the dialog) — confirm minimal seed-then-browse is acceptable.
- [ ] Browser presentation: modal QDialog (plan's choice) vs. a persistent docked panel — defer the panel unless wanted in 3a.

## Plan Review
**Status**: complete
**When**: 2026-06-21 01:51 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-104/plan.md` at `8262e2f`
**PR**: PR-less
**Verdict**: approve-with-suggestions

### Findings
- [ ] (suggestion) Restore loop must dedup `GggsTileLayers/dirs` against itself AND skip a dir already live as a flat layer, or a re-selected dir spawns a duplicate layer on restart — `plan.md:114`
- [ ] (suggestion) `GggsTileLayer::onRemovedFromMap()` drop-on-remove keys on `directory()`; if the same dir is selected twice (two flat layers), removing one drops the dir and the other no longer re-persists. Plan should state dedup-on-select prevents duplicates (matches the existing `openTileStore` `!roots.contains` guard) — `plan.md:106,117`
- [ ] (suggestion) Resolve OQ1 (seed-then-browse) before implementing — seed-then-browse is the right minimal call and is consistent with the existing `QFileDialog::getExistingDirectory` entry; recommend confirming and proceeding, not building roots-management (edges into #68/#69) — `plan.md:190`
- [ ] (suggestion) Resolve OQ2 (modal dialog) — modal `QDialog` is the correct lowest-risk first cut; docked panel rightly deferred. Recommend confirming — `plan.md:196`
- [ ] (suggestion) `item_types.h` removes `GggsStoreLayerType`; the enum is positional (UserType+offset) and other entries follow it, but values are not persisted by index, so removal is safe — confirm no `qgraphicsitem_cast<GggsStoreLayer*>` survives (grep shows none) — `plan.md:148`

## Implementation
**Status**: complete
**When**: 2026-06-21 06:25 +0000
**By**: Claude Opus

**Plan**: `.agent/work-plans/issue-104/plan.md` (synced to as-built)
**Branch**: `feature/issue-104` at `94bf6a9`
**Build**: `./ui_ws/build.sh camp` — clean (warnings are pre-existing `-Wunused-parameter`/`-Wsign-compare` in unrelated files).
**Tests**: `./ui_ws/test.sh camp` — **85 tests, 0 failures, 2 skipped** (the 2 skips are the pre-existing GL render tests that self-skip without offscreen GL). 10 of those are the new 3a tests.

### What was built (piece 3a, ADR-0005)
Five atomic commits on `feature/issue-104`:
1. `9a3b351` — generic catalog seam + `GggsStoreSource` (added alongside the old layer so it builds standalone).
2. `819215a` — retire `GggsStoreLayer`; persistence reset; wire the "Stores" tab.
3. `92a2462` — the three new gtests.
4. `94bf6a9` — docs sync (plan, ADR-0005, README).
(Plus the pre-existing plan/ADR-draft commits from the planning phase.)

**Generic catalog module** (`src/camp2/catalog/`, ROS-free, in `camp_map`):
- `CatalogItem` — group/selectable-leaf tree node carrying an opaque source-id + key.
- `CatalogModel` — generic `QAbstractItemModel` over the item tree.
- `CatalogSource` — the reuse seam (`discover()` + `instantiate(LayerList*)`).
- `CatalogBrowser` — embeddable `QWidget` (tree + "Open store…" seed + "Add to map").

**`raster::GggsStoreSource : CatalogSource`** — first + only consumer. `discover()` folder-scans a store root into `CatalogItem` nodes (recursive `buildNode` + file-scope `dirHasTifs`); `instantiate()` spawns a flat top-level `GggsTileLayer`, dedups-on-select, and persists the dir.

**Retired `GggsStoreLayer`**: removed `gggs_store_layer.{h,cpp}`, the `GggsStoreLayerType` enum entry, its CMake source, and the `background_manager.cpp` references. Confirmed no surviving `GggsStoreLayer*` cast. Scan logic salvaged into the source.

**Persistence RESET (no migration)**: stopped persisting `GggsStores/roots`; selected flat layers persist under the new `GggsTileLayers/dirs`. `createDefaultLayers()` restores a flat `GggsTileLayer` per still-existing dir, deduped against itself AND already-live layers; `GggsTileLayer::onRemovedFromMap()` drops its dir; the old `GggsStores/roots` key is cleared once on startup. Band 1 + camp#90 colormap + camp#102 lazy/async/default-off/visibility carry over unchanged.

### Final design decisions honored
1. **Seed-then-browse** — "Open store…" reuses `QFileDialog::getExistingDirectory`, then browses that root.
2. **Embedded tabbed widget** (NOT a modal dialog) — `CatalogBrowser` is a "Stores" tab in the deployed `MainWindow`'s tree-tab widget (`src/camp/mainwindow.cpp`), next to Mission + Layers, targeting `project->map()->topLevelLayers()`. The generic `CatalogModel`/`CatalogSource` seam is unchanged.

### Plan-Review suggestions folded
1. Restore loop dedups vs itself (`QSet`) AND already-live flat layers — done (`background_manager.cpp`); covered by `test_gggs_persistence` `RestoreRecreatesFlatLayerDeduped`.
2. Dedup-on-select in `GggsStoreSource::instantiate()` (returns the existing layer for a re-selected dir; dir persisted uniquely) so `onRemovedFromMap` can't orphan a duplicate — done; covered by `DedupOnSelect`.
3. OQ1 seed-then-browse — confirmed/implemented.
4. OQ2 — implemented as the embedded tab (supersedes modal).
5. Stale `GggsStoreLayer` comments cleaned (`layer.h`, `gggs_tile_layer.{h,cpp}`).

### Deviations from the original plan (all reflected in plan.md/ADR-0005)
- **Presentation**: embedded "Stores" tab, not a modal `QDialog` (per operator decision 2).
- **Entry point**: the browser's own "Open store…" button, not a `BackgroundManager` context action — "Open tile store" was retired (camp2 sandbox loses the entry; the deployed app gains the tab).
- **Persistence ownership**: per-selection persist lives in `GggsStoreSource::instantiate()` (keeps the browser generic), not in `BackgroundManager`.
- **Discovery helpers**: only `dirHasTifs` was extracted (file-scope in `gggs_store_source.cpp`); `subtreeHasTifs` was unnecessary — the recursive `buildNode`'s `nullptr` return prunes tile-free subtrees. No `gggs_scan.h`.

### Known limitations (deferred, noted in plan + ADR)
- The retired store node's `QFileSystemWatcher` (camp#102 live tile/epoch pickup) is gone; a flat `GggsTileLayer` needs a restart (or manual `rescan()`) to pick up newly-landed tiles. Per-layer watcher is a follow-up.
- The seed affordance is directory-based (`getExistingDirectory`), fine for all 3a (GGGS) sources; a future non-directory source would need its own seed path.

### Environment note
The dependency layer installs (`underlay_ws`/`core_ws/...`) were empty in this worktree. Built the three `core_ws` packages camp needs (`marine_ais_msgs`, `marine_interfaces`, `marine_autonomy` — all other deps resolve from system ROS Jazzy) into the shared `layers/main/core_ws/install` to enable the camp build. No source changes there.

### Next step
3a is complete and green. Follow-ups are the carved-out stacked issues: **#108** (band-select + per-band colormap) and **#109** (compositing z-order/opacity render test). Not pushed (host performs pushes).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-21 06:40 +0000
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-104 at `6333700`
**Mode**: pre-push
**Depth**: Deep (reason: new ADR + cross-cutting persistence/lifecycle + deployed-MainWindow UI integration + layer-type retirement)
**Must-fix**: 0 | **Suggestions**: 2
**Round**: 1 | **Ship**: recommended — no must-fix; clean build, 85 tests/0 fail (10 new); retirement, persistence reset, and tab integration all verified against the scrutiny list.

### Findings
- [ ] (suggestion) No direct `QAbstractItemModelTester` over `CatalogModel` — its index/parent/rowCount protocol is only exercised indirectly (spawn test wraps the Map model; discover test checks the CatalogItem tree). Model reads as correct; a direct tester is cheap insurance — `test/test_gggs_flat_layer_spawn.cpp` / `src/camp2/catalog/catalog_model.cpp`
- [ ] (suggestion) Watcher-loss escape hatch unreachable: `rescan()` lost its only caller (the retired `QFileSystemWatcher`) and has no UI affordance, yet ADR/plan/comments cite "(or a manual `rescan()`)". Add a "Rescan" context-menu action (restores a manual refresh + gives `rescan()` a caller) or reword — `src/camp2/raster/gggs_tile_layer.cpp:150`

### Regression assessment — QFileSystemWatcher loss
**Acceptable deferral.** #104 scopes live refresh out; the loss is documented in lockstep in ADR-0005 Consequences, plan Known Limitations, and `.agents/README.md`, with a per-layer-watcher follow-up acknowledged. The real behavior change (a loaded layer no longer auto-picking up even static new tiles) is low-impact at current store scale (restart recovers). Only residual is suggestion #2 (the "manual rescan" claim has no UI path).

### Notes
- Retirement clean: no dangling `GggsStoreLayer*` casts/includes; `GggsStoreLayerType` removed and `ItemType` is compared by symbolic value (not persisted by ordinal), so removal is safe.
- Persistence reset correct: restore dedups vs self (QSet) + live layers; `instantiate()` dedups-on-select; `directory_` stored verbatim so `removeAll` matches the persisted key; one-time `GggsStores/roots` clear is read-free and idempotent (no old+new collision/data loss).
- Tab integration sound: `CatalogBrowser` parented to `treeTabs`; `currentChanged` lambda's `else` branch handles the 3rd tab without breaking Mission/Layers.
- Generic seam right-sized (operator-mandated reusability, not speculative).
- Static analysis: cppcheck flagged 2 sub-threshold const-pointer nits + Qt `slots` false positives; cpplint unavailable. Nothing actionable.

### Next step
Approved pre-push. Lifecycle: address the 2 suggestions if desired (both optional) → push / open PR → triage-reviews. Not pushed (host performs pushes).

## Implementation
**Status**: complete
**When**: 2026-06-21 06:55 +0000
**By**: Claude Opus

**Branch**: `feature/issue-104` at `94816b7`
**Build**: `./ui_ws/build.sh camp` — clean (only pre-existing `-Wunused-parameter`/
`-Wsign-compare`/`-Wlogical-not-parentheses` warnings in unrelated files; none in
the changed code).
**Tests**: `./ui_ws/test.sh camp` — **86 tests, 0 failures, 2 skipped** (was 85;
+1 is the new model-tester case. The 2 skips are the pre-existing GL render tests
that self-skip without offscreen GL).

Both Local Review (Pre-Push) suggestions addressed before push (operator elected
BOTH). Two atomic commits + this progress entry.

### Suggestion #2 — Rescan action (implemented as an action, not a doc-reword) — `0a0505e`
- Added a **"Rescan" context-menu action** to the flat `GggsTileLayer`
  (`gggs_tile_layer.cpp` `contextMenu()`, alongside the base Opacity/Remove menu
  and the Colormap submenu). It calls the existing `rescan()`, which re-enumerates
  the tile-set dir and appends any newly-landed `<level>_<row>_<col>.tif` not
  already held (and is safe to call repeatedly / when nothing changed — it builds
  a `known` set and no-ops when there's nothing new).
- This restores a manual refresh path **and** gives `rescan()` a live caller (it
  had been dead code since the `QFileSystemWatcher` retirement). It is the chosen
  lightweight mitigation for the deferred watcher loss: "restart CAMP to pick up
  newly-landed tiles" → "right-click → Rescan."
- **Docs synced to the new reality** — the previously-aspirational "(or a manual
  `rescan()`)" claims now reference the Rescan action and read "mitigated by a
  manual Rescan context-menu action; live auto-pickup (a per-layer watcher)
  remains a follow-up": ADR-0005 Consequences, `plan.md` Known Limitations,
  `.agents/README.md` (ADR-0005 paragraph), the `gggs_tile_layer.cpp` comments
  near `rescan()` (incl. the half-written-tile retry note), and the `rescan()`
  header doc. The per-layer-watcher follow-up stays acknowledged as the live
  solution; Rescan is the stopgap. (`src/camp2/map/layer.h` had no
  watcher/rescan comment to update — its `RasterLayer, GggsTileLayer` note is
  already correct.)

### Suggestion #1 — direct model test — `94816b7`
- Added `GggsCatalogModel.SatisfiesItemModelProtocol` to
  `test/test_gggs_flat_layer_spawn.cpp`: a `CatalogModel` populated by
  `raster::GggsStoreSource::discover()` from a temp store fixture, with a
  `QAbstractItemModelTester` attached **before** population (so it validates the
  `begin/endInsertRows` insertion path of `addTopLevel` too) plus an explicit
  index round-trip (root → child → `parent()`). This exercises the
  index/parent/rowCount/columnCount protocol **directly** — the prior tests only
  hit it indirectly (spawn tests wrap the Map model; the discovery test checks the
  `CatalogItem` tree). Reuses the file's existing `WarningTrap` so any protocol
  violation fails the test.
- Kept non-GL/GDAL (discovery only stats `*.tif` filenames; empty placeholder
  tiles suffice). `QAbstractItemModelTester` is from `QtTest`; the
  `test_gggs_flat_layer_spawn` target already links `Qt5::Test` and includes the
  tester, so **no `CMakeLists.txt` change was needed**.

### Notes
- No behavior regression to undo (review verdict was `approved`, 0 must-fix);
  these are quality polish. `rescan()` semantics are unchanged — it gained a UI
  caller and accurate docs.
- Not pushed (host performs pushes).
