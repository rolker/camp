# ADR-0013: LOD level selection and demand-driven tile loading

## Status

Accepted (camp#103, the LOD half; the visible-region half is ADR-0011)

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
read them). The worker loads only not-yet-loaded tiles at the selected level
intersecting the snapshot viewport.

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

### Progressive refinement across a level switch (field-verified fix)

A level change does **not** release the outgoing level's tiles. They stay
resident and `itemsIntersecting()` draws them as the backdrop — stale levels
first, coarse→fine, with the selected level last (on top) — so the view stays
populated while the new level streams in, each arriving tile covering its
backdrop. The initial eager-release design blanked the layer for the whole
load on every zoom across a level boundary (very visible flicker in the
2026-07-31 field verify against the real store).

`tilesReady()` releases the stale levels (CPU `resetPixels()` **paired with**
GL `releaseGL()` — a CPU-only clear leaves a stale texture shadowing any
re-load) once the selected level's visible set has fully loaded and no worker
is running, so steady-state renders — and holds resident — only the selected
level. Transient cost: the outgoing level's visible tiles stay resident for
the duration of the incoming load. A rapid multi-level zoom/pan sweep can
transiently stack several stale levels (each load aborted before the release
condition fires), bounded by the levels traversed and freed wholesale at the
first idle completed load — self-healing, and still far below the pre-#103
eager whole-store residency. If that transient ever matters in practice, a
release-on-abort pass is the follow-up shape.

**Headless / no-selection defaults**: `selected_level_ == -1` disables the
level filter everywhere (worker, `itemsIntersecting()`, the range fold) and a
null viewport disables the spatial filter — so a layer that never paints
(headless tests driving `waitForLoad()` + `renderImage()`) behaves exactly as
before this ADR: load and render everything.

### Extent semantics

`sceneBounds()` unions **finest-level tile extents only**. Overview tiles are
padded to their coarse GGGS grid cell (the L0 apex spans a whole 8° grid);
uniting them would balloon fit-to-extent far beyond the data footprint.

### Auto-range across levels

A level switch does not reset the layer auto-range; the fold only ever widens
it. This is sound because the imagery overview fold is MEAN (uma ADR-0011), so
overview values are contained in the fine range. A fold policy that can exceed
the source range (none exists today) would need a reset-on-switch here.

### The camp#172 hook (enabled, not implemented)

The demand-driven pattern — "the viewport exposes tiles whose pixels are not
resident ⇒ kick a filtered load" — is exactly the reload seam
`SonarLiveCacheLayer` needs for evicted tiles (camp#172), and the shared-fold
adoption path (camp#171) rides the same structure. This PR deliberately keeps
the hook clean (`hasUnloadedVisibleTiles()` + the snapshot-filtered worker) and
implements neither.

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
