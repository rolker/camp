# ADR-0005: Catalog browser + flat selectable display layers for GGGS stores

## Status

Accepted

Extends [ADR-0003](0003-backgrounds-as-layers-and-depth-tree.md) §4
(split persistence). Reverses part of camp#90's store-layer design.

## Context

camp#90 (`src/camp2/raster/gggs_store_layer.{h,cpp}`) added GGGS tile **stores**
to CAMP by recursively building the store's folder hierarchy *as a nested subtree
inside the Layers tree*: a store root becomes a grouping `GggsStoreLayer`, its
modality/maturity subdirectories become more grouping nodes, and any directory
holding `*.tif` becomes a renderable `GggsTileLayer` leaf (default-off, lazy/async
loaded — camp#102). The set of opened store roots persists under
`QSettings GggsStores/roots`; `BackgroundManager::createDefaultLayers()` re-creates
the subtree per root on startup, and `GggsStoreLayer::onRemovedFromMap()` drops a
root from that list when the operator removes it.

That got discovery "for free" but conflated **browsing** a store catalog with
**compositing** display layers:

- The Layers tree fills with store-structure folders the operator did not choose
  to display — the tree is meant to be the *display* list, not a file browser.
- Compositing controls (visibility, opacity, draw order) are meaningful only on
  the leaves, yet the leaves are buried in a per-store subtree rather than being
  peers in the top-level layer list where order across stores matters (e.g.
  sidescan over bathy).
- The persisted unit is the store *root*, so app state restores entire nested
  trees, not the specific layers the operator was actually displaying.

The redesign (issue #104) splits browse from compose. Issue #104 also wanted
this seam built **generically** — a reusable catalog/source mechanism other
layer-manager items (cf. topic discovery, camp #44/#68/#69) can plug into later —
with the GGGS stores as the **first and only** consumer for now. Band selection
and a compositing z-order/opacity test were carved out to separate follow-ups
([#108](https://github.com/rolker/camp/issues/108),
[#109](https://github.com/rolker/camp/issues/109)).

## Decision

### 1. Browse and compose are split

A store is **browsed** through a catalog, not mounted into the Layers tree. The
operator **selects** a tile-set from the catalog; the selection spawns a **flat,
top-level display layer** in the Layers tree. The nested in-tree `GggsStoreLayer`
is retired.

### 2. A generic catalog/source seam, GGGS as the first consumer

The browser is source-agnostic:

- **`catalog::CatalogModel`** — a generic hierarchical `QAbstractItemModel` of
  *group* nodes (not selectable) and *selectable leaf* nodes carrying an opaque
  payload (source id + a string key). It knows nothing about GGGS, raster, or
  layers.
- **`catalog::CatalogSource`** (abstract) — the plug-point. A source `discover()`s
  its hierarchy into the model and, given a selected leaf payload,
  `instantiate(map::LayerList*)`s the corresponding flat layer.
- **`catalog::CatalogBrowser`** — a generic, **embeddable `QWidget`** (tree view
  + "Open store…" seed + "Add to map") shown as a **tab in the Layers panel**
  next to the layer tree: browse in the "Stores" tab, compose in the "Layers"
  tab. It owns the `CatalogSource`s and routes a selected leaf back to its source.
- **`raster::GggsStoreSource`** — the first and only concrete source: folder-scan
  discovery (modality → maturity → tile-set, salvaged from `GggsStoreLayer::scan`)
  and `instantiate()` = a flat `raster::GggsTileLayer(layers, tilesetDir)`, with
  dedup-on-select and `GggsTileLayers/dirs` persistence owned by the source.

No other `CatalogSource` is implemented in this pass — the seam exists so future
layer-manager items can adopt it without re-deriving a browser.

### 3. Selected layers are flat top-level `GggsTileLayer`s

A selected tile-set becomes an independent top-level `GggsTileLayer` (not nested
under a store node). Keeping it a `GggsTileLayer` means camp#90's colormap LUT and
camp#102's lazy/async/default-off/visibility-persistence carry over unchanged.
Visibility, opacity (`MapItem::setOpacity` + slider), and draw-order reorder
(`Map` `stackBefore`, tested in `test_map_model.cpp`) are already provided by the
layer model — no new compositing machinery. The layer renders band 1 for now;
band selection is #108.

### 4. Persistence: selected flat layers, with a one-time reset of the old roots

Persistence moves off store roots and onto the **selected flat layers**, extending
ADR-0003 §4 (background/display layers persist as app/Map state):

- A new key `GggsTileLayers/dirs` records the selected tile-set directories (in
  order). Per-layer visibility/opacity already persist via the `GggsTileLayer`
  /`MapItem` settings keyed by `itemID()` — this key only records *which* layers
  to recreate.
- `createDefaultLayers()` restores a flat `GggsTileLayer` per still-existing dir.
- Selecting a tile-set appends its dir (dir-unique, in `GggsStoreSource::
  instantiate()`, which also dedups-on-select by reusing an already-displayed
  layer on that dir); removing a flat layer drops it
  (`GggsTileLayer::onRemovedFromMap()` — the flat-layer analogue of the retired
  `GggsStoreLayer::onRemovedFromMap()`). The restore loop dedups against itself
  and against already-live layers so a repeated dir can't double-spawn.
- The old `GggsStores/roots` key is **dropped, not migrated**: it is not read, and
  is cleared once on startup so a stale value can't resurrect a nested store tree.
  This follows ADR-0003 §4's "no compatibility shim for old on-disk state" — and
  QSettings layer-restore is app state, not a project file, so a one-time reset is
  acceptable.

### 5. Browse-seed persistence (Stores-tab roots) — amendment, orthogonal to §4

Operator GUI testing surfaced a gap: the *flat layers* persist fine (§4), but the
**browsed store root** in the Stores tab does not — relaunching CAMP left the
Stores tree empty, so the operator had to "Open store…" again every session ("the
store I added didn't stick"). This amendment **re-introduces persistence of the
browsed root(s)** as a `CatalogBrowser`-level convenience:

- `CatalogBrowser` keeps a `std::vector<Seed>` (`{sourceId, root}`) **parallel to
  the model's top-level rows** (each `addTopLevel` appends exactly one top-level
  node per seed, in order). `seedRoot()` — the non-dialog core extracted from
  `openStore()` — discovers a root, adds the node, records the seed, and persists.
- Seeds persist under a **generic** `QSettings CatalogBrowser/seeds` key (a
  `QStringList` of `sourceId<TAB>root` entries; a tab delimiter since store paths
  may contain commas but not tabs). The key is browser-level and source-agnostic,
  not GGGS-specific.
- `restoreSeeds()` runs once from `MainWindow` right after `addSource()` (so the
  seeds' sources exist): it re-`discover()`s each persisted root, **skipping a
  seed whose source is unknown or whose root no longer exists on disk**
  (skip-missing, mirroring the flat-layer restore), dedups, and rewrites the
  persisted list once if any stale/duplicate entry was dropped.
- A "**Remove from browser**" context action on a top-level node drops the model
  row, erases the matching seed, and re-persists (so a mis-picked folder is not
  stuck forever), keeping the seeds↔rows index alignment correct.

This is deliberately **orthogonal to §4, not a reversal of it**. It persists only
**which store root(s) to re-browse**, so the Stores *tree* repopulates; it does
**not** spawn display layers and does **not** resurrect the retired nested
`GggsStoreLayer` (a browsed root is still just a catalog tree — adding a tile-set
to the map remains an explicit operator action that spawns a flat `GggsTileLayer`,
persisted independently via `GggsTileLayers/dirs`). The two persistence records —
browser seeds (browse state) and flat-layer dirs (display state) — are separate
and do not interact.

## Consequences

- `GggsStoreLayer` and its `GggsStoreLayerType` enum entry are removed; its scan
  logic moves into `GggsStoreSource::discover()` as a recursive `buildNode` using
  a file-scope `dirHasTifs` (the old `subtreeHasTifs` prune is unnecessary — the
  recursion's `nullptr` return for a tile-free subtree subsumes it).
- New `src/camp2/catalog/` module (model + source seam + browser) is net-new code
  in the `camp_map` shared lib (ROS-free, per `.agents/README.md`).
- `BackgroundManager`'s "Open tile store" context action is **retired** (stores
  are browsed via the catalog tab); its `createDefaultLayers` restore path is
  reworked in lockstep. The browser is wired as a "Stores" tab in the deployed
  `MainWindow` (`src/camp/mainwindow.cpp`).
- The retired `GggsStoreLayer`'s `QFileSystemWatcher` (camp#102 live tile/epoch
  pickup) is dropped with it; a flat `GggsTileLayer` does not auto-refresh on
  newly-landed tiles. This is **mitigated by a manual "Rescan" context-menu
  action** on the flat layer (right-click → Rescan re-enumerates the tile-set
  directory and adds any newly-landed tiles), which gives the operator a refresh
  path without restarting CAMP. Live auto-pickup (a per-layer watcher) remains a
  deferred follow-up.
- Operators upgrading lose their persisted nested store trees once (the old
  `GggsStores/roots` is reset); they re-select tile-sets through the browser,
  which then persist as flat layers. This is a deliberate one-time reset.
- Browsed store roots now persist under `CatalogBrowser/seeds` (§5) so the Stores
  tab repopulates its tree on launch — orthogonal to the `GggsTileLayers/dirs`
  flat-layer persistence; it rebuilds the browse tree only and spawns no layers.
- `.agents/README.md` persistence/layer-model notes are updated.
- Band selection (#108) and a compositing z-order/opacity render test (#109) build
  on this structural change as separate follow-ups.

## Alternatives considered

- **Keep store folders in the Layers tree (camp#90 as-is).** Rejected: conflates
  browsing with compositing, clutters the display list, and persists whole trees
  rather than the displayed layers — the problems this ADR fixes.
- **GGGS-specific browser (no generic seam).** Rejected per the operator's
  explicit decision to make the source seam reusable for other layer-manager items
  (camp #44/#68/#69) — the generic split is cheap to express and avoids a second
  rewrite when topic discovery adopts the same browse/select pattern.
- **Migrate old `GggsStores/roots` into flat layers on upgrade.** Rejected:
  ADR-0003 §4 already established no back-compat for old persisted state; a
  migration shim is unwarranted for app state, and a clean reset is simpler and
  unambiguous.
- **Modal dialog for the browser.** Rejected in favour of an **embedded tabbed
  widget** (a "Stores" tab beside the layer tree): browse and compose sit
  side-by-side in the same panel, so the operator picks a tile-set and sees it
  appear in the adjacent Layers tab without a dialog round-trip. A fully docked
  (detachable) panel remains a possible later refinement.

## References

- Issue [#104](https://github.com/rolker/camp/issues/104) — stores redesign
  (this ADR covers piece 3a)
- [#108](https://github.com/rolker/camp/issues/108) — band selection + colormap
  (follow-up)
- [#109](https://github.com/rolker/camp/issues/109) — compositing z-order/opacity
  test (follow-up)
- [ADR-0002](0002-web-mercator-scene-and-layer-model.md) — Web-Mercator scene,
  two-model split
- [ADR-0003](0003-backgrounds-as-layers-and-depth-tree.md) — backgrounds as
  layers + split persistence (§4 extended here)
- camp#90 — GGGS store layer (the nested-tree design reversed here)
- camp#102 — lazy/async/default-off tile loading (carried over)
- `src/camp2/raster/gggs_store_layer.{h,cpp}`, `gggs_tile_layer.{h,cpp}`,
  `src/camp2/background/background_manager.cpp` — the code reworked

---
**Authored-By**: `Claude Code Agent`
**Model**: `Claude Opus 4.8 (1M context)`
