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
   `discover()` reuses the folder-scan logic currently in
   `GggsStoreLayer::scan()` (modality → maturity → tile-set, "a dir with `*.tif`
   is a leaf; a dir with tifs deeper is a group") to build `CatalogModel` nodes
   instead of `Layer` objects. `instantiate()` does
   `new raster::GggsTileLayer(layers, tilesetDir)`. The `*.tif` discovery helpers
   (`dirHasTifs` / `subtreeHasTifs`) move out of the anonymous namespace into a
   small reusable spot (e.g. `gggs_store_source.cpp` file-scope or a tiny
   `gggs_scan.h`) so both old code paths and the source share one definition.

4. **`catalog::CatalogBrowser` (`QDialog` or dockable `QWidget`)** — a generic
   tree view over a `CatalogModel` with an "Add to map" affordance
   (double-click-leaf or select+button). It is wired to a `CatalogSource` set;
   on `itemActivated` it calls the owning source's `instantiate(topLevelLayers)`.
   3a presents it as a **modal dialog** launched from the manager context menu
   (simplest, lowest-risk; a docked panel is a later refinement, not 3a).

### Repurpose / retire `GggsStoreLayer`

`GggsStoreLayer` (the nested in-tree node) is **removed** as a Layers-tree
participant. Its scan logic is salvaged into `GggsStoreSource::discover()`. We do
**not** keep it as a layer type (leaving it would resurrect nested trees from any
stale persisted root, and there's no consumer once persistence is reset). The
`GggsStoreLayerType` enum entry in `item_types.h` is removed. `onRemovedFromMap()`
(root-drop) goes with it.

### Wire the browser into the deployed UI

`BackgroundManager` is the host (it already owns the "Open tile store" action and
`topLevelLayers()`). In `background_manager.cpp`:

- `contextMenu()`: "Open tile store" → **"Browse tile stores…"** which opens the
  `CatalogBrowser` over a `GggsStoreSource`. The operator still picks a store-root
  directory first (a `QFileDialog::getExistingDirectory`, as today) to seed the
  source's scan root — *or* the dialog's own "Add store root…" button; 3a keeps
  the existing pick-a-root entry to minimize UI surface. Selecting a tile-set in
  the browser spawns a flat `GggsTileLayer` and persists it (below).
- The browser is a `camp_map`-level widget (ROS-free), consistent with the
  `camp_map` shared-lib boundary (.agents/README.md: keep `camp_map` ROS-free).

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
  `new raster::GggsTileLayer(layers, dir)` for each still-existing dir.
- On selection in the browser: append the tile-set dir to `GggsTileLayers/dirs`
  (dedup), mirroring today's `openTileStore` persist.
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
| `src/camp2/catalog/catalog_browser.{h,cpp}` | **New** — generic tree dialog/widget; "Add to map" → source `instantiate` |
| `src/camp2/raster/gggs_store_source.{h,cpp}` | **New** — `GggsStoreSource : CatalogSource`; folder-scan discovery + spawns flat `GggsTileLayer` |
| `src/camp2/raster/gggs_scan.h` (or file-scope in source) | **New** — shared `dirHasTifs`/`subtreeHasTifs` extracted from `gggs_store_layer.cpp`'s anon namespace |
| `src/camp2/raster/gggs_store_layer.{h,cpp}` | **Remove** — nested in-tree store node retired (scan logic salvaged into the source) |
| `src/camp2/raster/gggs_tile_layer.{h,cpp}` | Add `onRemovedFromMap()` override → drop `directory()` from `GggsTileLayers/dirs` |
| `src/camp2/map/item_types.h` | Remove `GggsStoreLayerType` enum entry |
| `src/camp2/background/background_manager.{h,cpp}` | `contextMenu` → "Browse tile stores…"; `createDefaultLayers` restores from `GggsTileLayers/dirs` (drop `GggsStores/roots`, one-time `remove`); browser launch + per-selection persist replaces `openTileStore` |
| `CMakeLists.txt` | Add `catalog/*.cpp` + `gggs_store_source.cpp` to `camp_map` sources; drop `gggs_store_layer.cpp`; register new gtests |
| `test/test_catalog_source.cpp` | **New** — discovery model: a temp store tree → `CatalogModel` has expected groups/leaves; non-tif dirs excluded |
| `test/test_gggs_flat_layer_spawn.cpp` | **New** — `GggsStoreSource::instantiate()` adds exactly one flat `GggsTileLayer` to `topLevelLayers` (model stays valid) |
| `test/test_gggs_persistence.cpp` | **New** — select → `GggsTileLayers/dirs` written; `createDefaultLayers`-style restore recreates the flat layer; remove → dir dropped; old `GggsStores/roots` ignored |
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

## Open Questions

- **Browser entry point UX**: 3a plan keeps the existing "pick a store-root
  directory" `QFileDialog` to seed the source, then opens the catalog browser
  over that root. Alternative: the browser owns root management ("Add store
  root…" inside the dialog, persisted set of roots). The plan chooses the
  minimal seed-then-browse path; confirm that's acceptable vs. a roots-managing
  browser (the latter is more work and edges toward #68/#69 territory).
- **Modal dialog vs. docked panel**: plan ships a modal `QDialog`. A persistent
  docked panel reads better long-term but is more UI plumbing; deferred unless
  the operator wants it in 3a.

## Estimated Scope

Single PR for 3a. Moderately sized (new `catalog/` module = ~4 small files +
GGGS source + manager rewire + persistence reset + 3 gtests + ADR + README).
No GL/render changes. Band-select (#108) and compositing test (#109) are separate
stacked follow-ups.
