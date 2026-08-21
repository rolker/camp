# ADR-0013: LOD level selection and demand-driven tile loading

## Status

Accepted (camp#103, the LOD half; the visible-region half is ADR-0011).
Amended by camp#194: the selection is a **ceiling** (multi-level
compositing), not an equality filter — see "Multi-level compositing",
"Extent semantics", and "Auto-range across levels" below.

## Context

A GGGS store layer (`GggsTileLayer`) held every fine tile's pixels resident:
`loadTilesWorker()` read all tiles at first paint (3.6 GB observed open cost on
the sidescan store), and at fit-zoom the renderer drew every fine L13 tile —
far more data than the screen can show. The producer half landed in
unh_marine_autonomy#188 (uma ADR-0011): stores now carry a derived
`overviews/` sidecar — a flat directory of coarser GGGS-level tiles named
`<level>_<row>_<col>.tif`, folded 4→1 per level down to a single L0 apex.

## Decision

### Level selection (the generic multi-level wiring)

`selectLodLevel(ground_metres_per_pixel, available_levels)`
(`raster/lod_level_selector.h`, header-only/inline):

- The **ideal** level is `gggs::Level::fromCellSize(mpp)` — the coarsest GGGS
  level whose cells are at most one screen pixel.
- The pick is the **finest available level coarser-or-equal to the ideal**
  (largest level number ≤ ideal). On a contiguous pyramid this is the ideal
  itself; on a sparse ladder the choice degrades toward coarser (fewer tiles),
  never loading finer data than the screen can resolve. If every available
  level is finer than the ideal, the coarsest available is the closest match.
- Empty ladder → `-1`, the caller's "no selection" sentinel.

The input is **true ground metres per pixel**. Web-Mercator scene metres are
inflated by ~sec(latitude) (≈1.37× at 43°N — about half a level), so `paint()`
converts with `web_mercator::metersPerUnit` at the viewport centre.

The function is deliberately layer-agnostic (operator decision 2026-07-31):
`GggsTileLayer` feeds it the levels found in its directory + `overviews/`
sidecar; a future natively multi-level layer (chart ENC scale ladder, uma
ADR-0010 — native levels, no derived overviews) feeds it its own ladder.

### Demand-driven loading

`paint()` derives the viewport clip first (ADR-0011), selects the level, then
kicks the loader with a **snapshotted filter** (level + viewport as value
copies — the live members are reassigned every frame, so the worker must never
read them). The worker loads only not-yet-loaded tiles at levels **up to**
the selected level (the ceiling — camp#194) intersecting the snapshot
viewport; levels finer than the selection stay excluded, which is what keeps
the load viewport-bounded rather than store-bounded.

Re-kick conditions (each a reviewed must-fix):

- **First paint** — the one-time lazy kick, already filtered.
- **Level change** — select the new level and re-kick (the kick's own
  abort+join replaces any in-flight load).
- **Pan/zoom exposure** — when idle and the filter has moved since the last
  kick and `hasUnloadedVisibleTiles(viewport)` is true. A pure pan therefore
  re-kicks (previously panned-in regions stayed blank forever); the
  moved-since-last-kick guard keeps a permanently failing tile from re-kicking
  every frame, and kicking only when idle avoids per-frame abort+join stalls
  during pan storms.

### Multi-level compositing (amended by camp#194; supersedes "progressive refinement")

The original design treated `selected_level_` as an **equality** filter:
exactly one level loaded, rendered, and stayed resident at steady state, with
other levels drawn only transiently as a zoom backdrop. That model implicitly
assumed a nested pyramid (every coarser level covers the same footprint as
the finest) — true for a derived `overviews/` sidecar, false for a
**region-disjoint native ladder** (ENC chart store, uma ADR-0010 D7: one
native level per compilation scale, each covering only its own sub-region).
Under the equality filter such a store rendered exactly one region band per
zoom (camp#194).

The amended model: `selected_level_` is a **max threshold (ceiling)**.

- **Residency**: every available level ≤ the selection loads
  (viewport-bounded) and stays resident **permanently** — it is part of the
  composited picture, not a transient backdrop. Levels > the selection never
  load.
- **Render**: `itemsIntersecting()` draws the whole resident set in one
  ascending pass — coarse→fine painter's order, **no render-time level
  filter**. Fine overdraws coarse where both exist; coarse fills where fine
  is absent, so a disjoint ladder shows all its regions at every zoom
  **at-or-finer than each region's native level**. Coarse-zoom limitation:
  a region whose *only* native level is finer than the current selection
  never loads (levels > selection are excluded by the residency rule), so
  it renders blank at coarser zooms even though `sceneBounds()` includes
  its footprint. This is the intended viewport-bounded tradeoff — loading
  finer-than-needed data to fill coarse zooms would reintroduce the
  store-bounded open this ADR exists to avoid; if it matters in practice
  (e.g. a chart ladder whose finest-only regions vanish at overview zooms),
  the residency/coverage follow-up family is camp#195.
- **Level-switch transitions** (the camp#103 field-verified no-blank-frame
  guarantee, both directions). Scope of "no blank frame": it is a guarantee
  about the *transition*, not about coverage — a region that has coverage at
  both the outgoing and the incoming selection never blanks while the switch
  is in flight. It does **not** override the coarse-zoom limitation above: a
  region whose only native level is finer than the new selection has no
  coverage to draw once its tiles release, so it blanks by the residency
  rule, not by a transition gap.
  - *Zoom-in*: the resident coarser levels back the arriving selected level —
    unchanged, except they now simply remain part of the picture afterward.
  - *Zoom-out*: the still-resident finer tiles (now > the selection) keep
    drawing — on top, since ascending order puts them last — until the
    coarser selection's visible set finishes loading. Release earlier and
    the view would blank for the whole load, the exact flicker the
    2026-07-31 field verify eliminated for zoom-in.
- **NoData backfill (QA fidelity)**: with no render-time level filter, the
  shader's NoData discard means coarse data now shows through *within* a fine
  tile's footprint — both through interior NoData holes and through the
  padding a fine tile carries out to its GGGS grid-cell edge. Pre-camp#194
  those pixels were transparent (only one level drew), so a hole read as
  "no data here"; now it reads as the coarser level's value. This is a
  deliberate consequence of compositing — filling coverage gaps from coarser
  levels is the whole point of the fix — but it is a **fidelity** change of
  the same family that keeps `smooth_interpolation_` default-OFF (camp#132):
  what the operator sees at a given pixel may come from a coarser
  compilation/fold than the tile that nominally covers it. Two mitigations
  keep it honest today: the depth-at-cursor readout (camp#180,
  `getElevation()`) always samples **finest-covering-tile-first** independent
  of what is drawn, so inspection is unaffected; and the auto-range fold
  spans every composited level, so the coarse fill is colour-mapped on the
  same scale as the fine data rather than against a foreign range. If a QA
  workflow ever needs "show me only this level's own data", that is a
  render-time opt-in (a composite-depth cap of 1), not a change to this
  decision — it rides camp#195 alongside the overdraw mitigation.
- **Release**: `tilesReady()` releases only tiles at levels **finer than the
  selection** (CPU `resetPixels()` **paired with** GL `releaseGL()` — a
  CPU-only clear leaves a stale texture shadowing any re-load), and only once
  the selection's visible set has fully loaded
  (`hasUnloadedVisibleTiles()`, which tests the same ≤-selection ceiling)
  with no worker running. That load-before-release gate is what carries the
  zoom-out no-blank guarantee.

A rapid multi-level zoom/pan sweep can transiently stack several
finer-than-selection levels (each load aborted before the release condition
fires), bounded by the levels traversed and freed wholesale at the first idle
completed load — self-healing, and still far below the pre-#103 eager
whole-store residency. If that transient ever matters in practice, a
release-on-abort pass is the follow-up shape.

**Residency bound (ADR-0010 cross-reference, camp#195)**: `GggsTileLayer` has
no residency/eviction budget — camp ADR-0010 governs `SonarLiveCacheLayer`, a
different class. Under compositing, multiple levels stay resident
simultaneously at steady state, so the pre-existing unbounded-within-level
growth multiplies across every level ≤ the selection. For the `chart` store
this is genuinely bounded (a fixed native ladder, 54 tiles); for a derived
`overviews/` pyramid the overhead is a bounded geometric series (~33% over
the fine level alone). The classes to watch are **`reference`** (mixed-level
imports: S-102 + fine imported grids vs coarse legacy priors) and
**`draft`/`processed`** (uma ADR-0010 D9 generates overview pyramids over
potentially large fine-level survey coverage) — `chart` is the one member of
this family that is *not* the risk. Given the camp#153 `SonarLiveCacheLayer`
OOM precedent for this accumulation shape, the eviction-bound follow-up is
tracked as **camp#195**. Compositing also multiplies per-frame **overdraw**,
not just memory: on a deep nested pyramid every resident level contributes a
near-full-viewport quad at fine zoom (up to ~14 stacked quads for a full L13
pyramid — a fill-rate cost that software-GL backends pay too). Mitigation
(skip fully-covered coarse tiles, or a composite-depth cap) rides camp#195
if pan latency regresses.

**Headless / no-selection defaults**: `selected_level_ == -1` disables the
level ceiling everywhere (worker, range fold, release) and a null viewport
disables the spatial filter — so a layer that never paints (headless tests
driving `waitForLoad()` + `renderImage()`) behaves exactly as before this
ADR: load and render everything. (Since camp#194 the render path itself has
no level filter to disable — it always draws the resident set.)

### Extent semantics (amended by camp#194)

`sceneBounds()` unions **every native (non-overview-sidecar) tile's extent,
at any level**. A region-disjoint native ladder needs every level's footprint
in the union — with the old finest-level-only union, the regions covered only
by coarser native levels sat outside `boundingRect()` and could never paint
(QGraphicsView culls there), regardless of the compositing fix. Overview
tiles (tagged via `GggsTile::isOverview()` from the `overviews/` scan) stay
excluded: they are padded to their coarse GGGS grid cell (the L0 apex spans a
whole 8° grid); uniting them would balloon fit-to-extent far beyond the data
footprint. For the legacy single-native-level store + `overviews/` sidecar
the two unions are identical.

**Exception — the overview-only store.** The exclusion above is stated for a
store that HAS native tiles. If a store's main directory holds no usable
native tile at all (only an `overviews/` sidecar — a pyramid whose fine level
was pruned or has not been imported), excluding overviews would leave
`scene_bounds_` null, and a null `boundingRect()` silently blanks the layer
everywhere. So `rebuildLevelIndex()` falls back in that case to unioning the
FINEST overview level present (`gggs_tile_layer.cpp`, the second union pass),
which is also the old finest-level-only behavior for this store shape. The
contract in full: **overview tiles are excluded whenever any native tile
exists; an overview-only store takes the finest-overview-level union
instead** — a deliberately over-large (grid-cell-padded) extent, chosen
because an over-large fit-to-extent is recoverable by the operator and a null
one is not.

**The store-layout contract this depends on.** `isOverview()` records
*directory provenance* (the tile was found under `overviews/`), but the
exclusion rule above is about *padding* (the tile is padded out to its full
GGGS grid cell). Those are different properties, and the guard is only
correct because the producer side guarantees they coincide:

- Tiles under `overviews/` are the derived pyramid (uma ADR-0011): each is a
  4→1 MEAN fold filling its whole coarse grid cell, so its extent is the grid
  cell, not the data footprint.
- Tiles in the store's main directory are native — written where the data is.
  They may be padded to their own (fine) grid cell, but that cell is at most
  one fine tile larger than the data, which is the footprint granularity the
  extent has always had.

So a producer that ever wrote grid-cell-padded coarse tiles into the **main**
directory, or unpadded data tiles into `overviews/`, would break
`sceneBounds()` — over-large fit-to-extent in the first case, a clipped
extent in the second. That is a uma-side contract, not something camp can
detect from a tile alone (a padded tile and a coincidentally-full tile are
byte-identical in geometry). If the layouts ever diverge, the fix is an
explicit padded/native flag in the store metadata, not a heuristic here.

### Auto-range across levels (amended by camp#194)

A level switch does not reset the layer auto-range; the fold only ever widens
it. The fold's level gate matches the composited steady-state render set
exactly (every level ≤ the selection) — a strictly more precise statement of
the same never-reset behavior, not a policy change. This is sound because the
imagery overview fold is MEAN (uma ADR-0011), so overview values are
contained in the fine range; a native ladder's levels each contribute their
own regions' true extents. A fold policy that can exceed the source range
(none exists today) would need a reset-on-switch here.

### The camp#172 hook (implemented — camp#171/#172 PR)

The demand-driven pattern — "the viewport exposes tiles whose pixels are not
resident ⇒ kick a filtered load" — is exactly the reload seam
`SonarLiveCacheLayer` needs for evicted tiles (camp#172), and the shared-fold
adoption path (camp#171) rides the same structure. That PR **implements** the
hook: `SonarLiveCacheLayer::hasUnloadedVisibleTiles()` tests the evicted-fine
index set against the viewport, and `paint()` kicks a snapshot-filtered
`QtConcurrent` reload worker (loading each evicted fine tile's cached GeoTIFF),
reusing this loader's moved-since-last-kick guard and the `waitForLoad()`-style
headless seam (`waitForReload()`). The one addition over this layer's loader is a
**reload-hysteresis** guard (ADR-0010 D6): the reload fires only while residency
is below `0.75×` the eviction budget, so the reloaded tile cannot immediately
re-trigger eviction (no reload↔evict ping-pong). camp#171 confirms the overview
fold converges with the uma shared fold engine (uniform `TiledRasterTile::edge`,
MEAN) — same geometry this ADR's overview render already assumes.

## Consequences

- Store open cost becomes viewport-bounded instead of store-sized; the eager
  3.6 GB read is gone. Acceptance: seconds-fast open on the 1012-tile store.
- Fit-zoom renders the overview pyramid (crisp at ~1 cell/pixel) instead of
  every fine tile.
- `rescan()` scans fine tiles only; newly built `overviews/` content is picked
  up at next open (the sidecar is batch-produced by `build_sidescan_overviews`,
  not live-updated). Follow-up if that changes.
- Loading state surfaces through the existing status mechanism
  (`"(loading...)"` → `""`/`"(no data)"`), so streaming stays visible to the
  operator.
