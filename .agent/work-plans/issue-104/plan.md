# Plan: Stores layer redesign (piece 3a) — generic catalog browser + flat selectable layers + persistence reset

## Issue

https://github.com/rolker/camp/issues/104 (scoped to **piece 3a** per the
operator scope-decision comment of 2026-06-21; band-select is
[#108](https://github.com/rolker/camp/issues/108), compositing test is
[#109](https://github.com/rolker/camp/issues/109) — both out of 3a scope).

## Context

camp#90 added `GggsStoreLayer` (`src/camp2/raster/gggs_store_layer.{h,cpp}`):
given a store-root directory it recursively builds a **nested subtree inside the
Layers tree** — grouping `GggsStoreLayer` nodes with `GggsTileLayer` leaves,
default-off. camp#102 made the leaves lazy/async/default-off with a
`QFileSystemWatcher`. Roots persist under `QSettings GggsStores/roots`;
`BackgroundManager::createDefaultLayers()` re-creates them on open
(`background_manager.cpp:77-86`), and `GggsStoreLayer::onRemovedFromMap()` drops a
root from that list on removal.

3a **reverses** the "store folders straight into the layers tree" design. Browse
and compose are split:

- A reusable **catalog/source browser** (separate panel/dialog) presents the
  hierarchy; the operator **selects** a tile-set.
- A selected tile-set spawns a **flat top-level `GggsTileLayer`** in
  `topLevelLayers()` — independently controllable (visibility/opacity/order all
  already free), keeping camp#90 colormap + camp#102 lazy/async.
- Persistence is **reset**: stop persisting `GggsStores/roots`; persist the
  *selected flat layers* instead. Old roots app-state is **dropped, not migrated**
  (ADR-0003 no-back-compat).

Band-select stays band 1 (`gggs_tile.cpp` `GetRasterBand(1)`) — #108. Compositing
z-order/opacity test — #109. Draw-order reorder is already implemented+tested
(`map.cpp` `stackBefore`; `test_map_model.cpp` `DropReordersToTargetRow`, #84).

## Approach

### The generic browse-source seam

The key design call (open question in the issue review) is **where the catalog
lives**. Decision: build a small, source-agnostic **catalog model + browser
widget** in a new `src/camp2/catalog/` module, with GGGS as the first and only
concrete source.

1. **`catalog::CatalogModel` (`QAbstractItemModel`)** — a generic hierarchical
   catalog of browsable items. A node is either a *group* (has children, not
   selectable) or a *selectable leaf* carrying an opaque payload (a source id +
   a string key — for GGGS, the tile-set directory path). The model knows nothing
   about GGGS, raster, or layers; it just presents a tree and emits
   `itemActivated(const CatalogItem&)` when the operator picks a leaf.

2. **`catalog::CatalogSource` (abstract)** — the seam. A source `discover()`s its
   hierarchy into `CatalogModel` nodes and, given a selected leaf payload,
   `instantiate(map::LayerList* layers)`s the corresponding flat layer (returns
   the created `map::Layer*`, or nullptr). This is the plug-point other
   layer-manager items (cf. #68/#69/#44 topic discovery) can implement later —
   3a ships exactly one implementation and does **not** add others.

3. **`raster::GggsStoreSource : catalog::CatalogSource`** — the first consumer.
   `discover()` reuses the folder-scan logic from `GggsStoreLayer::scan()`
   (modality → maturity → tile-set, "a dir with `*.tif` is a leaf; a dir with
   tifs deeper is a group") to build `CatalogItem` nodes instead of `Layer`
   objects. `instantiate()` does `new raster::GggsTileLayer(layers, tilesetDir)`
   (with dedup-on-select + persistence — see Persistence rework). **As-built:**
   the scan is a single recursive `buildNode()` at `gggs_store_source.cpp`
   file-scope using `dirHasTifs`; the old `subtreeHasTifs` prune is dropped — the
   recursion returns `nullptr` for a tile-free subtree, which subsumes the prune,
   so only `dirHasTifs` needed extracting.

4. **`catalog::CatalogBrowser` (embeddable `QWidget`)** — a generic tree view
   over a `CatalogModel` with a seed ("Open store…") affordance and an "Add to
   map" button (also double-click-leaf). It owns a set of `CatalogSource`s and a
   target `LayerList`; on activation it routes the selected leaf back to its
   source's `instantiate(target)`. **As-built decision (supersedes the earlier
   modal-dialog plan): the browser is shown as a tab in the Layers panel**, next
   to the layer tree — browse in the "Stores" tab, compose in the "Layers" tab.
   The `CatalogModel` stays a pure tree model (no `itemActivated` signal — the
   browser owns the add affordance and emits `layerAdded` after a spawn), keeping
   the generic seam unchanged.

### Repurpose / retire `GggsStoreLayer`

`GggsStoreLayer` (the nested in-tree node) is **removed** as a Layers-tree
participant. Its scan logic is salvaged into `GggsStoreSource::discover()`. We do
**not** keep it as a layer type (leaving it would resurrect nested trees from any
stale persisted root, and there's no consumer once persistence is reset). The
`GggsStoreLayerType` enum entry in `item_types.h` is removed. `onRemovedFromMap()`
(root-drop) goes with it.

### Wire the browser into the deployed UI

**As-built (supersedes the context-menu plan):** the `CatalogBrowser` is added
as a **"Stores" tab** in the deployed `MainWindow`'s left tree-tab widget
(`src/camp/mainwindow.cpp`, the `QTabWidget` that already holds Mission + Layers),
seeded with a `GggsStoreSource` and targeting `project->map()->topLevelLayers()`.
The operator clicks **"Open store…"** in the tab (the existing
`QFileDialog::getExistingDirectory`, seed-then-browse), browses the tile-set
catalog, and "Add to map" spawns a flat `GggsTileLayer` into the Layers tree
(persisted below).

`BackgroundManager`'s **"Open tile store" context action is retired** (the tab is
the entry point now); "Open raster" is unchanged. The browser is a `camp_map`
widget (ROS-free), consistent with the shared-lib boundary.

### Persistence rework (reset)

Replace `GggsStores/roots` with a **selected-flat-layers** record:

- **New key** `GggsTileLayers/dirs` (a `QStringList` of selected tile-set
  directories). Visibility/opacity already persist per-layer via
  `GggsTileLayer::read/writeSettings()` keyed by `itemID()` (camp#102) and
  `MapItem::setOpacity` persistence (layer.cpp) — so 3a does **not** re-persist
  those; it only needs to know *which* flat layers to recreate, in order.
  (Order: `QStringList` is ordered; recreate in stored order. Operator reorder in
  the Layers tree is live via `stackBefore` and is orthogonal to which-layers
  persistence — re-persisting tree order is a #109/follow-up nicety, not 3a.)
- `createDefaultLayers()`: drop the `GggsStores/roots` loop
  (`background_manager.cpp:77-81`); add a loop over `GggsTileLayers/dirs` that
  `new raster::GggsTileLayer(layers, dir)` for each still-existing dir. **As-built:
  the loop dedups against itself (a `QSet` of restored dirs) AND skips a dir
  already live as a flat layer** (Plan Review #1), so a repeated entry / a second
  `createDefaultLayers` can't double-spawn.
- On selection: **as-built, the persist lives in `GggsStoreSource::instantiate()`**
  (not `BackgroundManager`) — it appends the tile-set dir to `GggsTileLayers/dirs`
  (dir-unique) right where it spawns the layer, and **dedups-on-select** by
  returning the existing flat layer if one is already displayed on that dir (Plan
  Review #2). This keeps the browser generic (it knows nothing about the key) and
  the source owns its layer type's persistence.
- **Drop-on-remove:** `GggsTileLayer` gains an `onRemovedFromMap()` override that
  removes its `directory()` from `GggsTileLayers/dirs` — the flat-layer analogue
  of the old `GggsStoreLayer::onRemovedFromMap()`. (This is the only behavioral
  add to `GggsTileLayer`; render/colormap/lazy/visibility are untouched.)
- **Old `GggsStores/roots` is dropped, not migrated** (ADR-0003 §4 no-back-compat).
  We do not read it; a one-time `settings.remove("GggsStores/roots")` in
  `createDefaultLayers()` clears the stale key so it can't resurface. State this
  in the PR body.

### ADR

Author **`docs/decisions/0005-stores-browser-and-flat-display-layers.md`**
recording: (a) the browse/compose split reversing camp#90's folders-in-the-tree
design; (b) the generic `CatalogSource`/`CatalogModel`/`CatalogBrowser` seam with
GGGS as first consumer; (c) the persistence-format change (store roots → selected
flat layers) and the no-migration reset, extending ADR-0003 §4. Draft committed
with this plan.

## Files to Change

| File | Change |
|------|--------|
| `docs/decisions/0005-stores-browser-and-flat-display-layers.md` | **New** ADR (drafted with this plan) |
| `src/camp2/catalog/catalog_item.h` | **New** — `CatalogItem` (group/leaf + opaque payload: source id + key) |
| `src/camp2/catalog/catalog_model.{h,cpp}` | **New** — generic `QAbstractItemModel` over a `CatalogItem` tree; `itemActivated` signal |
| `src/camp2/catalog/catalog_source.h` | **New** — abstract `CatalogSource`: `discover()` + `instantiate(LayerList*)` seam |
| `src/camp2/catalog/catalog_browser.{h,cpp}` | **New** — generic embeddable widget (tree + "Open store…" seed + "Add to map"); routes to source `instantiate` |
| `src/camp2/raster/gggs_store_source.{h,cpp}` | **New** — `GggsStoreSource : CatalogSource`; folder-scan discovery + spawns flat `GggsTileLayer` (dedup-on-select + persist) |
| ~~`src/camp2/raster/gggs_scan.h`~~ | **Dropped** — only `dirHasTifs` needed; lives at `gggs_store_source.cpp` file-scope. `subtreeHasTifs` unnecessary (recursive `buildNode` nullptr-return subsumes the prune) |
| `src/camp2/raster/gggs_store_layer.{h,cpp}` | **Remove** — nested in-tree store node retired (scan logic salvaged into the source) |
| `src/camp2/raster/gggs_tile_layer.{h,cpp}` | Add `onRemovedFromMap()` override → drop `directory()` from `GggsTileLayers/dirs`; clean stale `GggsStoreLayer` comments |
| `src/camp2/map/item_types.h` | Remove `GggsStoreLayerType` enum entry |
| `src/camp2/map/layer.h` | Clean stale `GggsStoreLayer` doc comment (→ `GggsTileLayer`) |
| `src/camp2/background/background_manager.{h,cpp}` | Retire "Open tile store" action + `openTileStore`; `createDefaultLayers` restores from `GggsTileLayers/dirs` (drop `GggsStores/roots`, one-time `remove`, dedup vs self + live) |
| `src/camp/mainwindow.cpp` | **As-built** — add the "Stores" `CatalogBrowser` tab (seeded with `GggsStoreSource`, targeting the Map's top-level layers) next to Mission + Layers |
| `CMakeLists.txt` | Add `catalog/*.cpp` + `gggs_store_source.cpp` to `camp_map` sources; drop `gggs_store_layer.cpp`; register new gtests |
| `test/test_catalog_source.cpp` | **New** — discovery: a temp store tree → expected `CatalogItem` groups/leaves; tile-free dirs excluded; root-is-tile-set leaf; empty/missing roots yield nothing |
| `test/test_gggs_flat_layer_spawn.cpp` | **New** — `GggsStoreSource::instantiate()` adds exactly one flat `GggsTileLayer` to `topLevelLayers` (model stays valid via `QAbstractItemModelTester`); dedup-on-select |
| `test/test_gggs_persistence.cpp` | **New** — select → `GggsTileLayers/dirs` written (dir-unique); `Map`-ctor restore recreates the flat layer deduped + skips missing; remove → dir dropped; old `GggsStores/roots` ignored + cleared |
| `.agents/README.md` | Update persistence notes (roots → selected flat layers; reset; browse/compose split); note new `catalog/` module + ADR-0005 |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Capture decisions, not just implementations | New ADR-0005 records the camp#90 reversal + the persistence-format change before implementing. |
| A change includes its consequences | Persistence reset is handled in lockstep: `createDefaultLayers` restore path, `onRemovedFromMap` drop-on-remove analogue, one-time stale-key removal, `.agents/README.md` notes — all in this PR. |
| Only what's needed | One concrete `CatalogSource` (GGGS) only; band-select (#108) and compositing test (#109) carved out; browser is a modal dialog (docked panel deferred); no `layer.json` contract. The generic seam is justified by the operator's explicit "build it reusable" decision, not speculative. |
| Improve incrementally | 3a lands the structural change + persistence reset alone; band + composite test stack on top as #108/#109. |
| Test what breaks | Three targeted gtests: catalog discovery model, flat-layer spawn (model validity), persistence round-trip incl. drop-on-remove and old-key-ignored. No new GL test (render path unchanged; compositing is #109). |
| Human control and transparency | Operator explicitly browses + selects each layer; nothing auto-added. Removal is operator-driven and de-persists. |
| Workspace vs. project separation | CAMP-side display only; store schema/producer stays in unh_marine_autonomy (ADR-0006). |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0001 (TopicBridge/executor) | No | No ROS subscription path touched; `catalog`/browser are ROS-free `camp_map` code. |
| ADR-0002 (Web-Mercator scene / two-model split) | Yes | Flat selectable layers are ordinary Layers-tree entries — exactly the two-model split. Browser is a separate UI, not a third model. No scene/coord change. |
| ADR-0003 (backgrounds-as-layers + split persistence §4) | Yes — central | Persisted unit shifts from store *roots* to *selected flat layers*; old key dropped, not migrated (matches §4's no-back-compat + app-state-persistence stance). New ADR-0005 extends §4 explicitly. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Retire nested `GggsStoreLayer` | `item_types.h` enum, `CMakeLists.txt` sources, any references | Yes |
| Persist selected flat layers (not roots) | `createDefaultLayers` restore, browser-selection persist, `GggsTileLayer::onRemovedFromMap`, stale-key removal | Yes |
| Old `GggsStores/roots` app-state | One-time `remove()`; documented reset in PR + ADR + README | Yes |
| New `catalog/` module + `gggs_store_source` | `CMakeLists.txt` `camp_map` lib sources + test registration | Yes |
| Browse/compose UI change | `.agents/README.md` persistence + layer-model notes | Yes |
| Band-select still band-1 | #108 (separate) | No — out of 3a scope by decision |
| Compositing z-order/opacity test | #109 (separate) | No — out of 3a scope by decision |

## Open Questions — RESOLVED (operator-confirmed 2026-06-21)

- **Browser entry point UX** → **Seed-then-browse** confirmed: the "Open store…"
  button in the browser reuses `QFileDialog::getExistingDirectory` to seed the
  `GggsStoreSource`, then browses that root's catalog. No roots-management UI
  (that edges into #68/#69 territory and is out of 3a scope).
- **Modal dialog vs. docked panel** → **Embedded tabbed widget** (neither): the
  `CatalogBrowser` is an embeddable `QWidget` shown as a **"Stores" tab** in the
  Layers panel next to the layer tree — browse in the Stores tab, compose in the
  Layers tab. This supersedes the plan's earlier modal `QDialog`; the generic
  `CatalogModel`/`CatalogSource` seam is unchanged.

### Known limitations (deferred)

- The retired `GggsStoreLayer` carried a `QFileSystemWatcher` (camp#102) that
  auto-picked-up tiles/epochs landing after open. A flat `GggsTileLayer` has no
  such watcher. This is **mitigated by a manual "Rescan" context-menu action**
  (right-click the flat layer → Rescan re-enumerates the tile-set directory and
  adds newly-landed tiles), so the operator can refresh without restarting CAMP.
  Live auto-pickup (re-adding a per-layer watcher) remains a follow-up, not 3a.
- The seed affordance is directory-based (`getExistingDirectory`), which suits
  all 3a sources (GGGS). A future non-directory `CatalogSource` would need its
  own seed path — the seam allows it (`CatalogSource::seedLabel/discover`), but
  the browser's file-dialog seed is GGGS-shaped for now.

## Estimated Scope

Single PR for 3a. Moderately sized (new `catalog/` module = ~4 small files +
GGGS source + manager rewire + persistence reset + 3 gtests + ADR + README).
No GL/render changes. Band-select (#108) and compositing test (#109) are separate
stacked follow-ups.

## As-built amendment — browse-seed (Stores-tab root) persistence

Operator GUI testing (2026-06-21) found that while flat layers persist (the §4
reset), the **browsed store root in the Stores tab did not** — relaunch left the
tree empty. Added a `CatalogBrowser`-level convenience that persists the browsed
root(s) and re-`discover()`s them at startup. **Orthogonal to the flat-layer
persistence, not a reversal of the §4 reset** (ADR-0005 §5): it repopulates the
browse tree only, spawns no layers, and does not resurrect the nested
`GggsStoreLayer`. As-built:

- `CatalogBrowser` holds a `std::vector<Seed>` (`{sourceId, root}`) kept parallel
  to the model's top-level rows. `openStore()` is refactored to delegate to a new
  non-dialog `bool seedRoot(CatalogSource*, const QString&)` (discover + add +
  record + persist, dedup-on-select) so the dialog path and restore share one
  seam and `seedRoot` is unit-testable.
- Seeds persist under a generic `QSettings CatalogBrowser/seeds` key (a
  `QStringList` of tab-delimited `sourceId\troot`; tab, not comma, since paths may
  contain commas). Whole list rewritten on every change; empty → key removed.
- `restoreSeeds()` (public) is called from `MainWindow` right after `addSource()`;
  it skips unknown-source / vanished-root seeds (skip-missing like the flat-layer
  restore), dedups, and rewrites the list once if it pruned anything.
- `CatalogModel::removeTopLevel(int)` + `CatalogItem::removeChild(int)` back a
  "Remove from browser" tree context action (`removeSeed(int)` on the browser):
  drop the top-level row, erase the matching seed, re-persist — index alignment
  preserved.
- Tests: new `test/test_catalog_browser_seeds.cpp` (`CatalogBrowserSeeds`,
  4 cases — persist round-trip, restore-skips-missing-root, seed dedup, remove
  de-persists), registered in `CMakeLists.txt` mirroring `test_gggs_persistence`.

| File | Change |
|------|--------|
| `src/camp2/catalog/catalog_item.h` | Add `removeChild(int)` |
| `src/camp2/catalog/catalog_model.{h,cpp}` | Add `removeTopLevel(int)` (begin/endRemoveRows) |
| `src/camp2/catalog/catalog_browser.{h,cpp}` | `Seed` + `seeds_` parallel vector; `seedRoot`/`restoreSeeds`/`removeSeed`/`topLevelCount` + `persistSeeds`/`selectTopLevel`; "Remove from browser" context menu; `openStore` delegates to `seedRoot` |
| `src/camp/mainwindow.cpp` | `catalogBrowser->restoreSeeds()` right after `addSource(...)` |
| `test/test_catalog_browser_seeds.cpp` | **New** — seed persistence round-trip / skip-missing / dedup / remove de-persist |
| `CMakeLists.txt` | Register `test_catalog_browser_seeds` |
| `docs/decisions/0005-...md`, `.agents/README.md` | Browser-seed persistence documented as orthogonal to §4 |
