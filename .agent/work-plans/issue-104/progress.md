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

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-21 13:20 +0000
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-104 at `3cb8738`
**Mode**: pre-push
**Depth**: Deep (reason: concurrency/lifecycle in the new `rescan()` caller; whole branch carries Round-1 Deep signal)
**Must-fix**: 1 | **Suggestions**: 0
**Round**: 2 | **Ship**: continue — one must-fix introduced by `0a0505e` (rescan abandons an in-flight load), but it is a single precise mechanical fix, so the next round should converge fast.

### Findings
- [ ] (must-fix) `rescan()` cancels an in-flight initial load and only resumes it when new tiles were found (`if(added && load_started_)`): a manual Rescan during the async load that finds nothing new aborts the worker (whole-tile granularity) and never restarts it, leaving tiles permanently `pixelsLoaded()==false` → silently half-blank layer, no recovery, status reads loaded. Reachable only because `0a0505e` gave `rescan()` its first live caller. Fix: compute the new-tile set before aborting; only abort+join+push+re-kick when there is something to add (else return false without disturbing the live load) — `src/camp2/raster/gggs_tile_layer.cpp:159`

### Re-review scope (Round 2)
Focused on the two new commits, with whole-branch re-confirmation:
- `0a0505e` (Rescan context-menu action): **no tile-list / atomic-flag corruption** — `rescan()` aborts+joins the worker under mutex before any `push_back`, so the vector is never mutated under the worker; `pixelsLoaded()` acquire/release untouched. Idempotent (no double-add via the `known` set); half-written tiles degrade to `valid()==false` and are skipped/retried. QAction ownership clean (parented to menu; `connect(..., this, ...)` auto-disconnects on destroy; matches the Opacity/Remove + Colormap pattern). Docs (ADR-0005, plan.md, .agents/README.md, inline comments) consistently describe the Rescan action + deferred per-layer watcher — no lingering "dead rescan()"/"(or a manual rescan())" contradiction in live docs (only in historical progress.md entries, correctly untouched). The **only** gap is the resume-logic must-fix above.
- `94816b7` (direct `QAbstractItemModelTester` over `CatalogModel`): tester attached before `addTopLevel` (validates the `begin/endInsertRows` insertion path); meaningful root→child→`parent()` round-trip with `rowCount`/`columnCount` asserts; genuinely non-GL/GDAL (`discover()` only `entryList`s `*.tif` filenames; empty placeholders suffice); deterministic, no flake. Cleared.
- Round-1 clears (retirement completeness, persistence reset, tab integration) untouched by these commits and still hold. Both Round-1 suggestions resolved (#1 by `94816b7`, #2 by `0a0505e` — the latter introducing the new must-fix).

### Environment / verification note
Could not re-run `./ui_ws/build.sh camp` in this worktree — the dependency-layer installs (`core_ws/install`, `underlay_ws/install`) are empty (same condition flagged in the Implementation entry; the deps built there have since been cleaned). The finding is a logic defect independent of compilation; relying on the impl phase's documented clean build + 86 tests/0 fail. Static analysis (cppcheck) on the changed C++ surfaced only the pre-existing Qt `slots`/namespace parser noise.

### Next step
Verdict is **changes-requested** (1 must-fix). Host (`/run-issue`) should dispatch **address-findings** to work the open must-fix from this entry, then re-dispatch `review-code` for Round 3 — the diff is not pushed until a pre-push review comes back approved. Not pushed (host performs pushes).

## Implementation
**Status**: complete
**When**: 2026-06-21 13:35 +0000
**By**: Claude Opus

**Branch**: `feature/issue-104`
**Build**: `./ui_ws/build.sh camp` — clean (only pre-existing `-Wunused-parameter`/
`-Wsign-compare`/`-Wlogical-not-parentheses` warnings in unrelated files; none in
the changed code).
**Tests**: `./ui_ws/test.sh camp` — **89 tests, 0 failures, 2 skipped**. The 2
skips are the pre-existing GL render self-skips; +2 over the suite at HEAD are the
new `GggsRescanTest` cases (both verified passing by running the target directly).

Addressed the single Round-2 `## Local Review (Pre-Push)` must-fix: `rescan()`
abandoned an in-flight initial async load. Two atomic commits + this entry.

### The fix — `rescan()` computes the new-tile set BEFORE disturbing the worker — `src/camp2/raster/gggs_tile_layer.cpp`
The old order was: abort+join any in-flight load UP FRONT, then only re-kick it
`if(added && load_started_)`. A manual Rescan that landed during the camp#102
initial load and found **no** new tiles therefore aborted the worker (whole-tile
granularity) and never restarted it — stranding those tiles at
`pixelsLoaded()==false` forever (silently half-blank, status reads loaded).

Reordered exactly as the reviewer specified:
1. Build the `known` set and construct the candidate `GggsTile`s into a local
   `new_tiles` vector FIRST. This only **reads** `tiles_` (paths are immutable
   post-construction) and reads tile metadata off disk — it never mutates
   `tiles_`, so it races nothing the worker does and needs no abort.
2. If `new_tiles` is empty → `return false` **without** touching the worker — the
   in-flight load runs on untouched. (This is the bug being fixed.)
3. Only when there is ≥1 tile to add: abort+join the worker under
   `abort_flag_mutex_` (so the subsequent `push_back` can't reallocate `tiles_`
   under the worker that captures `this` and iterates it), then do the
   extent merge + `push_back` (the previously-reviewed-clean has-new path,
   unchanged), then re-kick via `loadTiles()` if `load_started_`.

Idempotency (the `known` set), the under-mutex vector mutation, and the
`pixelsLoaded()` acquire/release ordering are all intact. Corner cases:
- New tiles while load in flight → still aborts+joins+pushes+re-kicks (unchanged).
- **No** new tiles while load in flight → leaves the worker running (fixed).
- Rescan after the load finished → adds new tiles + kicks a load for them; no-op
  when nothing new (`return false`).
- Concurrency: worker/`load_started_` state is only consulted after the no-op
  early-return, and the abort+join still happens under the existing mutex before
  any `tiles_` mutation — no new race.

The leading comment block + the abort comment were rewritten to describe the new
"compute first, abort only if adding" ordering and to record the camp#104 Round-2
rationale.

### Regression test — `test/test_gggs_rescan.cpp` (+ `CMakeLists.txt`)
Added a deterministic, **non-GL / non-async** `GggsRescanTest` (2 cases) pinning
down rescan's add/no-op contract via the public API (`rescan()` return value +
`sceneBounds()`):
- `NoNewTilesIsNoOp` — nothing new → `rescan()==false`, extent untouched, idempotent.
- `PicksUpNewlyLandedTile` — a tile landing after load → `rescan()==true`, extent
  grows eastward (west edge unchanged), then nothing-new → `false`.
It writes tiny real GeoTIFFs with GDAL (no GL); the scan/merge runs synchronously
on the calling thread, so it is non-flaky. New `ament_add_gtest` target modeled on
`test_gggs_visibility` (camp_map + Qt + GDAL).

**What the test does NOT cover, and why (per the task's allowance):** the *exact*
in-flight-abort race — "rescan-finds-nothing must leave a still-RUNNING pixel-load
worker alone" — is **not deterministically unit-testable here**. Distinguishing
the old vs new behavior requires the load worker to be genuinely in-flight at the
moment `rescan()` is called AND a way to observe per-tile `pixelsLoaded()`. The
production class deliberately exposes neither a worker-pause seam nor a
`pixelsLoaded()` accessor, and there is no public "start load without joining"
(only `paint()`/`waitForLoad()`, which join). A timing-based variant would only
detect the bug on runs where tiles happen to load slowly — it passes on the buggy
code whenever they load fast (false assurance), so it was **not** added rather than
ship a flaky-detecting test. Adding the pause/observability plumbing would exceed
the "minimal and local" scope of this fix and introduce its own concurrency
surface. The race itself is covered by the code-level reasoning above + review; the
committed test guards the contract the fix preserves.

### Environment note
Same as prior phases: the dependency-layer installs (`core_ws/install`,
`underlay_ws/install`) were empty in this fresh worktree, so `build.sh camp` first
failed on missing `marine_ais_msgs`. Rebuilt the three core_ws packages camp needs
(`marine_ais_msgs`, `marine_interfaces`, `marine_autonomy`) into the shared
`layers/main/core_ws/install`, then camp built clean. No source changes in those
layers.

### Next step
Round-2 must-fix resolved; build clean, suite green (89/0/2). Ready for a Round-3
pre-push `review-code`. Not pushed (host performs pushes).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-21 13:42 +0000
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-104 at `982aab5`
**Mode**: pre-push
**Depth**: Deep (reason: concurrency/lifecycle of the `rescan()`↔load-worker interaction; whole branch carries the Round-1/2 Deep signal)
**Must-fix**: 0 | **Suggestions**: 0
**Round**: 3 | **Ship**: recommended — the single Round-2 must-fix is correctly and completely resolved by `3a304f1`; no new findings, suite green.

### Findings
- [ ] No issues found. LGTM.

### Round-3 verification — `3a304f1` (`rescan()` reorder) closes the Round-2 defect
Re-review focused on the must-fix commit + its regression test, with whole-branch re-confirmation. Every claim traced and verified:
- **No-new early return leaves the in-flight load untouched** (the exact Round-2 defect). Between entry and `return false` (`gggs_tile_layer.cpp:165–188`), the code only builds `known`, scans the dir, and constructs candidates into a local `new_tiles` — **nothing reads/writes `future_watcher_`, `abort_flag_`, or `load_started_`** before the return. Provably no abort/join/disturb.
- **Candidate construction (step a) is read-only on `tiles_` and race-free.** `known` reads `tile->path()` (returns immutable `path_`); the worker (`loadTilesWorker`) writes only `data_`/range/`pixels_loaded_`/`texture_`, never `path_` — different-member, no data race. No `tiles_.push_back` before the abort+join, so no reallocation/iterator invalidation under the worker. The sole `push_back` is the post-join loop (`:206–222`).
- **Has-new path unchanged-clean**: abort+join under `abort_flag_mutex_` (`:198–204`) still precedes every `push_back`; the previously-reviewed mutation/extent-merge/re-kick path is byte-for-byte preserved, just gated on `!new_tiles.empty()`. `abort_flag_` left true is harmless — the re-kick `loadTiles()` re-arms it (`:253–255`).
- **`pixelsLoaded()` acquire/release + `known`-set idempotency intact**; no new race on `load_started_` (GUI-thread-only, read-only here) or `abort_flag_`. Edge cases (first-extent gating; empty `GggsTile` dtor → candidate/invalid tiles destroy safely with no GL context) check out.
- **Comments rewritten accurately** to the "compute first, abort only if adding" ordering with the camp#104 rationale; no stale "abort first" text.

### Test assessment — `test/test_gggs_rescan.cpp` (+ CMake)
- 2 cases (`NoNewTilesIsNoOp`, `PicksUpNewlyLandedTile`) are deterministic, non-GL, non-async (no `paint()`/`waitForLoad()` → no worker kicked; synchronous scan/merge on the test thread), asserting via the public API (`rescan()` return + `sceneBounds()`). Real tiny GeoTIFFs via GDAL; non-flaky. CMake target matches `test_gggs_visibility` exactly and is wired once.
- **The exact in-flight-abort race is acceptably NOT unit-tested.** Deterministic coverage would need BOTH a worker-pause seam AND a layer-level `pixelsLoaded()`/`loadInFlight()` accessor — neither exists, and `waitForLoad()` only kicks+joins. A timing test false-passes on fast loads. No cheap deterministic seam exists that isn't production-only test surface (virtual worker hook / controllable thread pool / test accessor), which exceeds this local reorder's scope. The race is closed *structurally* by the early return touching no worker state (review-verifiable); the committed contract test guards the add/no-op surface. Omission is the right call.

### Environment / verification note
Could not re-run `./ui_ws/build.sh camp` — the dependency-layer installs (`core_ws/install`) are empty in this fresh worktree (same condition as prior rounds). The finding-set is structural/logic, independent of compilation; relying on the Implementation entry's documented clean build + **89 tests/0 fail** (the 2 new rescan cases verified by running the target directly). Static analysis (cppcheck) on `gggs_tile_layer.cpp` surfaced only pre-existing Qt `slots`/parser noise; cpplint unavailable.

### Scope note
`3a304f1` touches only `rescan()` + the new test + CMake; it disturbs no Round-1/2 clear (retirement, persistence reset, tab integration, model tester, QAction ownership, docs), so none were re-litigated.

### Next step
Verdict is **approved** (0 must-fix). Lifecycle: push / open PR → triage-reviews. Round-2 → Round-3 converged. Not pushed (host performs pushes).

## Implementation
**Status**: complete
**When**: 2026-06-21 14:54 +0000
**By**: Claude Opus
**Commits**: `7981d61` (browser seed persistence + restore + removal), `12dc994` (seed-persistence tests), `97a9214` (docs)

### What & why
Operator GUI testing surfaced that the **browsed store root** in the Stores tab
did not persist: flat layers stick (ADR-0005 §4 `GggsTileLayers/dirs`), but a
relaunch left the Stores browse tree empty ("the store I added didn't stick"), so
the operator had to "Open store…" again each session. Added a `CatalogBrowser`-
level convenience that remembers which store ROOT(s) were opened and re-`discover()`s
them at startup.

**Orthogonal to the §4 flat-layer persistence, not a reversal of the reset**
(framed in ADR-0005 §5): it repopulates the **browse tree only** — it spawns no
layers and does **not** resurrect the retired nested `GggsStoreLayer`. The two
persistence records (browser seeds = browse state; `GggsTileLayers/dirs` =
display state) are separate and never interact. `GggsTileLayers`/dirs untouched.

### Design (as-built)
- **Parallel `seeds_` vector.** `CatalogBrowser` holds `std::vector<Seed>` where
  `struct Seed { QString sourceId; QString root; }`, kept strictly parallel to the
  model's top-level rows (each `addTopLevel` appends exactly one top-level node per
  seed, in order).
- **`seedRoot()` refactor.** Extracted the discover+add out of `openStore()` into a
  non-dialog `bool seedRoot(CatalogSource*, const QString& root)`: `discover(root)`;
  if it yields a node, `addTopLevel`, append to `seeds_`, **persist**, select/expand.
  Dedup-on-select: an already-present (sourceId, root) re-selects the existing row
  instead of re-adding (mirrors the flat-layer dedup). `openStore()` now = file
  dialog → `seedRoot(front_source, dir)`.
- **Generic persistence key.** `seeds_` persist under `QSettings CatalogBrowser/seeds`
  as a `QStringList` of `sourceId + "\t" + root` (tab delimiter — store paths may
  contain commas but not tabs). Whole list rewritten on every change; empty list →
  key removed (avoids the `@Invalid()` round-trip).
- **`restoreSeeds()` (public).** Wired into `MainWindow` right **after**
  `addSource(...)` so the seeds' sources exist. Skips a seed whose source is unknown
  or whose `QDir(root)` doesn't exist (skip-missing, like the flat-layer restore),
  dedups against already-present seeds, and rewrites the persisted list once only if
  it pruned a stale/duplicate/malformed entry (a clean restore leaves the key
  byte-for-byte untouched — restoring isn't a user change).
- **Removal affordance.** `CatalogModel::removeTopLevel(int)` (proper
  begin/endRemoveRows) + `CatalogItem::removeChild(int)` back a tree context menu
  ("Remove from browser" on a top-level node → `CatalogBrowser::removeSeed(int)`:
  drop the model row, erase the matching seed, re-persist), so a mis-picked folder
  isn't stuck forever. Seeds↔rows index alignment preserved across removal.

### Tests
`test/test_catalog_browser_seeds.cpp` — new `CatalogBrowserSeeds` gtest, widget/model
level and GL/GDAL-free (discovery only stats `*.tif` filenames; QSettings scoped to a
test org/app in `main()`), 4 cases all green:
- **PersistRoundTrip** — `seedRoot()` writes `CatalogBrowser/seeds`; a fresh browser +
  `restoreSeeds()` rebuilds the top-level node (`topLevelCount()==1`).
- **RestoreSkipsMissingRoot** — a bogus persisted root → `restoreSeeds()` adds nothing,
  no crash.
- **SeedDedup** — seeding the same root twice → one top-level node, one persisted seed.
- **RemoveDePersists** — seed → `removeSeed(0)` → key no longer contains it; fresh
  restore yields no node.
Registered in `CMakeLists.txt` mirroring `test_gggs_persistence`.

### Build / test
Clean. A fresh worktree had empty `core_ws/install`, so I first built the camp deps
(`colcon build --packages-up-to marine_ais_msgs marine_interfaces marine_autonomy`
in `core_ws` — succeeded, warnings only), then `./ui_ws/build.sh camp` (clean, only
pre-existing `-Wunused-parameter` warnings). `./ui_ws/test.sh camp` → **94 tests,
0 errors, 0 failures, 2 skipped** (was 89; +4 new `CatalogBrowserSeeds` cases; the
new target verified running directly: 4/4 OK). Docs (ADR-0005 §5 + Consequences,
`.agents/README.md` two-orthogonal-records note, `plan.md` as-built) in sync.

### Deviations
None material. seedRoot/restoreSeeds/removeSeed/topLevelCount are public (testability,
per the brief). Not pushed (host performs pushes).

### Next step
Branch `feature/issue-104` at `97a9214`, three atomic commits + this entry. Ready for
a pre-push `review-code` of the browse-seed addition. Not pushed.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-21 15:02 +0000
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-104 at `067f671`
**Mode**: pre-push
**Depth**: Deep (reason: new cross-cutting persistence record + QAbstractItemModel removal protocol + MainWindow startup lifecycle wiring; whole branch carries the Round-1/2/3 Deep signal)
**Must-fix**: 0 | **Suggestions**: 2
**Round**: 4 | **Ship**: recommended — no must-fix; the browse-seed addition is correct and the `seeds_`↔model-row alignment invariant holds across add / bulk-restore / mid-list removal. Two optional test-coverage suggestions only.

### Findings
- [ ] (suggestion) Mid-list removal alignment (the headline `seeds_[i]`↔row-`i` invariant) is not directly tested — `RemoveDePersists` only removes index 0; add a 3-seed test removing the *middle* and asserting the survivors restore to the correct roots — `test/test_catalog_browser_seeds.cpp`
- [ ] (suggestion) Malformed-entry prune branch (`tab < 0` → skip) untested — `RestoreSkipsMissingRoot` uses a well-formed entry; a no-tab/empty entry would exercise the decode-robustness path cheaply — `src/camp2/catalog/catalog_browser.cpp:126`

### Round-4 scope — browse-seed persistence (`7981d61`, `12dc994`, `97a9214`)
Re-review focused on the three browse-seed commits, with whole-branch re-confirmation. Every scrutiny point verified:
- **`seeds_`↔top-level-row alignment holds across ALL paths.** `seedRoot` (add): `addTopLevel` appends at `childCount` and `seeds_.push_back` at the same index; dedup re-selects without touching either. `restoreSeeds` (bulk): each survivor does `addTopLevel` + `push_back` together; every skip path (`tab<0`, unknown source, vanished root, duplicate, empty `discover`) adds to **neither**. `removeSeed` (mid-list): `removeTopLevel(row)` and `seeds_.erase(begin()+row)` use the **same** index, both lists shift down identically (`[A,B,C]` remove 1 → `[A,C]` ↔ rows `[A,C]`); the context menu feeds `index.row()` (top-level row = seeds index) straight in. No drift reachable.
- **Encoding round-trip safe** — decode uses `indexOf`+`left`/`mid` (split on the *first* tab), more robust than `split("\t")` (tolerant even of a tab inside the root); malformed/empty → `tab<0` → skipped, no crash/mis-key; empty list → `remove(key)`, restore reads a missing key as empty list and early-returns (`@Invalid()` avoided both ways).
- **`restoreSeeds()` idempotent + orthogonal** — wired after `addSource`; a second call finds every entry `present` so nothing double-adds; clean restore leaves `pruned=false` → no rewrite → key byte-for-byte untouched; valid roots never dropped (the rewrite persists `seeds_`, which holds every survivor). Touches `GggsTileLayers/dirs` **nowhere** (only filesystem reads + `CatalogBrowser/seeds`).
- **Removal model protocol intact** — `removeTopLevel`/`removeChild` proper `begin/endRemoveRows` bracketing; `removeChild` frees the owning `unique_ptr` (no leak/dangling); siblings' `row()` recomputed dynamically; context menu offers "Remove from browser" **only** on a top-level node (`!index.parent().isValid()`), stack `QMenu`/parented action — lifetime clean; Round-1 `QAbstractItemModelTester` still valid.
- **Orthogonality audit clean** — `discover()`/`seedRoot()`/`restoreSeeds()`/`removeSeed()` touch only the seeds key + filesystem; `instantiate()` is the sole writer of `GggsTileLayers/dirs`; seeding spawns no layer; `GggsStoreLayer` is genuinely deleted (every mention is a comment about the retired class — no resurrection).
- **Tests** scoped to org `camp_test`/app (no real-config pollution, `clear()` per case), offscreen + empty-placeholder `.tif` (non-GL/GDAL, deterministic); 4 cases cover persist/restore, skip-missing, dedup, remove-de-persist.
- **ADR-0005 §5** "orthogonal to §4, not a reversal — repopulates the browse tree only, spawns no layers, doesn't resurrect `GggsStoreLayer`" is accurate to the code.

### By-design note — transient-empty de-persist
A seeded root whose directory still exists but currently holds zero tiles → `discover()` returns `nullptr` → the seed is dropped and rewritten out of persistence (`catalog_browser.cpp:157-162`). Accepted: this is the invariant-preserving choice (a persisted-but-unshown seed would force `seeds_` to diverge from model rows), consistent with the documented skip-missing parallel. Worth awareness only if transient store emptiness is plausible in the field.

### Environment / verification note
Could not re-run `./ui_ws/build.sh camp` — the dependency-layer installs (`core_ws/install`) are empty in this fresh worktree (same condition as Rounds 2-3). Findings are structural/logic, independent of compilation; relying on the Implementation entry's documented clean build + **94 tests / 0 fail** (4 new `CatalogBrowserSeeds` cases). cppcheck on the new C++ surfaced only sub-threshold `constVariablePointer` style nits + Qt `slots` parser noise; cpplint unavailable.

### Scope note
The three Round-4 commits disturb no Round-1/2/3 clear (retirement, persistence reset, tab integration, `rescan()` reorder, model tester); none were re-litigated.

### Next step
Verdict is **approved** (0 must-fix). Lifecycle: push / open PR → triage-reviews. The 2 suggestions are optional test-coverage hardening. Not pushed (host performs pushes).

## Implementation
**Status**: complete
**When**: 2026-06-21 15:20 +0000
**By**: Claude Opus
**Commits**: this entry's test commit on `feature/issue-104`

### What & why
Addressed the **2 optional test-coverage suggestions** from the Round-4
`## Local Review (Pre-Push)` entry. **Test-only** — no production code changed.

1. **Mid-list removal alignment** (suggestion 1) — the headline `seeds_[i]`↔
   top-level-row-`i` invariant under a *middle* removal was only exercised at
   index 0 (`RemoveDePersists`). New **`MidListRemovalKeepsAlignment`**: seeds
   three distinct store roots A, B, C (`topLevelCount()==3`), `removeSeed(1)`
   (the middle / index-1 row), then asserts `topLevelCount()==2` and that the
   survivors are A and C **in order** two ways — (a) directly against the model
   rows' `DisplayRole` (each top-level node's name is its root basename, read via
   `browser.findChild<QTreeView*>()->model()`), which catches a drift where
   `seeds_` erases index 1 but the model erases a different row; and (b) the
   persisted `CatalogBrowser/seeds` list is exactly `gggs\tA`, `gggs\tC` in order.

2. **Malformed-entry decode-skip** (suggestion 2) — the `tab < 0` branch
   (`catalog_browser.cpp:126`) was untested (`RestoreSkipsMissingRoot` uses a
   well-formed entry). New **`RestoreSkipsMalformedEntry`**: writes
   `CatalogBrowser/seeds` with a no-tab `"garbage"` entry **and** an empty entry
   (both `tab < 0`) alongside one valid `gggs\t<root>`, then `restoreSeeds()` on a
   fresh browser. Asserts no crash and `topLevelCount()==1` — only the well-formed
   seed rebuilds a node; the malformed entries add none.

### Design notes
Both cases match the existing four `CatalogBrowserSeeds` fixtures: scoped QSettings
org/app (set in `main()`), `QSettings().clear()` per case, empty-placeholder `.tif`
stores (discovery only stats `*.tif` filenames), offscreen QPA — deterministic and
GL/GDAL-free. **No production code edited and no accessor widened**: model-row
identity is asserted through existing public surface (`QObject::findChild` for the
browser's `QTreeView`, then its `model()`), per the brief's "assert through the
existing public API" preference. Added `#include`s for `<QAbstractItemModel>`,
`<QFileInfo>`, `<QTreeView>`; updated the file-header pinned-behavior comment.

### Build / test
Fresh worktree had empty `core_ws/install`, so I first built the camp deps
(`colcon build --packages-up-to marine_ais_msgs marine_interfaces marine_autonomy`
in `core_ws` — succeeded, warnings only), then `./ui_ws/build.sh camp` (clean, only
pre-existing `-Wunused-parameter` warnings). `./ui_ws/test.sh camp` →
**96 tests, 0 errors, 0 failures, 2 skipped** (was 94; +2 new `CatalogBrowserSeeds`
cases). Ran the target directly: 6/6 OK, both new cases pass.

### Next step
One atomic test-only commit on `feature/issue-104`. The two Round-4 suggestions are
now covered. Not pushed (host performs pushes).
