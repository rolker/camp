# ADR-0010: Bounded live-tile eviction + consumer-side overview pyramid

## Status

Accepted

Amended by camp#171/#172 (world-store LOD step 4): D3 reframed as *convergence* with
the uma shared fold engine (no geometry change); D2 on-demand reload implemented; D6
reload-hysteresis added. See the "Consequences" memory-math and migration notes.

Amended [field 2026-08-27]: **D7** added — catalog prune-on-absence propagates into the
pyramid. This closes the "overview lifecycle-on-retraction / catalog-prune propagation"
item that the Consequences list had carried as a deferred follow-up since 2026-08-20.
It was deferred as bounded and display-grade; the BizzyBoat deployment showed it is
neither once a boat-side store is reset, because warm-load reinstates the stale pyramid
on every restart, so no code path could ever reflect the reset.

Cross-reference (camp#195, no change to this decision):
[ADR-0014](0014-gggs-viewport-scoped-residency.md) gives `GggsTileLayer` its own
residency budget. It is a **sibling**, not an extension — same forces, different
data contract — and it records where the two policies deliberately diverge: a
GGGS tile is file-backed, so it is dropped and re-read on pan-back instead of
being folded into a parent, and its budget is expressed in bytes converted to a
tile count from the observed tile size. Both decisions implement
`uma-ADR-0013` D4.

Implements camp issue #160 (supersedes #153, bounded eviction alone). Part of the
#154 camp resource self-monitoring umbrella. Extends
[ADR-0006](0006-live-tile-cache-persistence.md) (live tile cache persistence): it
fills the "eviction-by-area follow-up" left as a TODO in ADR-0006's consequences,
adds warm-load bounding, and adds a coarse overview product plus an LOD render
fallback. Depends on the GGGS `parent()`/`children()` index-math helpers
(unh_marine_autonomy#249).

## Context

`SonarLiveCacheLayer::tiles_` was **unbounded** — the only cap was reconciler
prune-on-absence. During a sustained live survey the in-memory map (each tile a
Float32 CPU band buffer plus an R32F GL texture) grows monotonically until RAM/VRAM
is exhausted; camp crashed on the salmon operator station shortly after enabling
live data (#153). This is a design gap, not a leak — tiles are freed correctly,
just never capped.

The naïve fix (evict fine tiles over a budget) fixes the crash but makes a
zoomed-out view go **blank** over evicted areas until the tiles are re-sent. The
operator's requirement is to *still see full coverage when some tiles are unloaded
at their highest resolution*. So eviction and a multi-resolution overview pyramid
are one design: an evicted fine tile is folded into its coarse parent **before** it
is dropped, so coverage degrades to a lower resolution instead of disappearing.

ADR-0006 makes this cheap and safe: live tiles are a **display-grade preview, not
data of record** — lossy, quantized, always rebuildable from the boat's durable
stores. Building downsampled overviews is display-LOD generation with no
provenance/fidelity burden.

Forces:

1. **Bound the footprint** so long surveys can't OOM the operator station.
2. **Never lose data** — an evicted tile must be persisted before its buffers are
   released (ADR-0006's disk cache is the durable backing).
3. **Keep the big picture** — a zoomed-out view must show coverage everywhere the
   survey reached, even where fine tiles were evicted.
4. **Don't waste bandwidth** (#71) — eviction must not trigger a re-request/re-evict
   churn on a constrained link.
5. **GUI-thread-only** mutation (ADR-0006 D4): the reconciler and tile maps are not
   thread-safe.

## Decision

### D1 — View-based LOD eviction against a resident-footprint budget

A configurable budget (`QSettings LiveTileCache/max_vram_bytes`, default 512 MiB;
`0` disables — the pre-#160 behaviour) caps the **total resident** footprint (CPU
band data + any uploaded GL texture) across **both** fine tiles **and** overview
tiles — the overviews grow with survey area (see D3), so they must count toward the
budget or they'd be an unbounded second cache. `evictIfOverBudget()` runs on the
GUI thread after every
applied patch and after warm-load, in **two phases**, always **farthest from the
current viewport centre first** (classic slippy-map LOD — keep what's near the view,
discard the rest; viewport centre from `scene()->views()`, LRU by a monotonic
`last_access_seq` fallback when headless):

1. **Fine tiles** (the detail) are evicted first, each folded into its coarse parent
   (D3) so its coverage survives at lower resolution.
2. If the pyramid *itself* is still over budget, the **numerous near-fine overview
   tiles** are evicted next — but **never the coarse apex** (`level <=
   kApexProtectLevel`, currently 6), so a whole-survey zoomed-out view always has
   coverage. An evicted overview's data already lives in its coarser ancestors (built
   by the same fold chain), so dropping it loses no coverage, only mid-zoom fidelity.

The apex is inherently a small, bounded handful for any realistic survey extent
(level-6 tiles span ~0.125°), so total resident memory is bounded: fine tiles +
near-view overviews to the budget, plus the O(small) apex. No vessel-position / tf
dependency — the operator chose view-based LOD over distance-from-vessel as the
simpler, more natural model.

### D2 — Persist-then-drop; keep possession in the reconciler (no churn)

Tiles are persisted continuously — a fine tile on every applied patch
(`handleTile`), an overview on every fold (`foldIntoParent`), both via the existing
coalesced `scheduleWriteThrough`. So by eviction time the tile (or an in-flight write
holding a copy of it) is already on disk, and eviction just folds a fine tile into
its parent, frees the GL texture under the renderer's context, and erases the entry —
**no write at eviction time** (which avoids a write-back storm when warm-load trims a
large cache of byte-identical tiles). The reconciler's `markHave` for that index is
**kept, not dropped**: we
still *possess* the tile (it is on disk), so the next catalog reconcile will not
re-request it. `tiles_` tracks residency (memory); the reconciler tracks possession
(disk) — the two intentionally decouple under eviction. Dropping it would cause a
re-request → re-receive → re-evict churn that wastes the very bandwidth #71 guards.
A genuine later catalog prune still frees the disk copy via `handleCatalog`.

**On-demand reload (camp#172, implemented here — see D6).** An evicted fine tile is
re-loaded from its disk cache when the operator pans back into its area within the
same session: `paint()` detects a visible evicted index and kicks a snapshot-filtered
reload worker (the `hasUnloadedVisibleTiles()` seam ADR-0013 left ready). Until the
reload completes, the coarse overview covers the area, so coverage never blanks. The
2026-07-23 field collapse — fine tiles shed and staying coarse for the rest of the
session — is closed by this path.

### D3 — Overview pyramid: uma-convergent geometry, fold-on-evict, full chain to level 0

`foldIntoParent()` folds an evicted fine tile into the GGGS `parent()` of its index
and recurses up to level 0, so a zoomed-out view always has coverage. Each overview
tile is built at the **fine tile's own `width×height`** — i.e. the fixed uniform
`marine_tiled_raster_store::TiledRasterTile::edge` (960²) — so a parent W×H tile
covers its ~4 children at half linear resolution. This is the **standard
half-resolution-per-level pyramid, and it converges with the uma shared fold engine**
(`overview_builder.hpp::buildParentTile`, which builds every parent at the fixed
`TiledRasterTile<T>::edge` and folds cells with the MEAN policy for imagery): camp's
live overviews therefore carry **identical fidelity to the merged store's pyramid at
the same zoom** (camp#171 / uma ADR-0011, world-store LOD step 4). The uniform
same-size parent/child is a **hard invariant of `foldChild()`**, not merely a choice:
it area-maps each child cell into a ¼ sub-window of the same-size parent and averages
the finite, non-NoData samples (MEAN) — area-mapping (rather than a fixed 2×2
quadrant) keeps the polar `latitudeScaleFactor` column scaling correct and matches
`buildParentTile`'s geographic-centre cell mapping. Overviews are folded **on eviction
only** (cheapest; the overview may lag the latest fine data until first eviction,
acceptable per ADR-0006 D1). They persist to an `overviews/` sub-directory of the
cache dir (own `<level>_<row>_<col>.tif` files — the uma sidecar layout) so warm-load
of fine tiles can't pick them up.

A camp-specific `⌊W/2⌋` quarter-resolution decimation was considered (to force more
per-level memory savings) and **rejected**: it would diverge from the merged store's
fidelity *and* violate `foldChild()`'s same-size invariant. Uniform-edge convergence
is the decision.

### D4 — Overviews are a local derived product, outside the reconciler

Overview tiles are **never** entered into `reconciler_` / the anti-entropy `tiles_`
set. They are not in the boat's catalog, so if they were reconciled they would be
pruned on absence. They live in a separate resident `overview_tiles_` map,
warm-loaded from the `overviews/` sub-dir (each file's level recovered from its
stem, since overviews span multiple coarse levels), and are freed under the
renderer context in the destructor exactly like fine tiles.

### D5 — LOD fallback by draw order

`items()` emits overview tiles first (coarse→fine, the natural `std::map` order by
GGGS level) and the fine tiles last, so the renderer draws fine tiles on top. Where
a fine tile is present it fully covers its parent; where it was evicted, the coarse
parent shows through instead of a blank gap. `recomputeBounds()` and
`foldAutoRange()` union both maps so the extent and colormap range stay correct when
only overviews remain for a region.

### D6 — Reload hysteresis (no ping-pong with eviction)

The on-demand reload (D2) inserts fine tiles back into `tiles_`, which grows
residency and would re-trigger D1 eviction. To avoid a reload↔evict ping-pong, reload
fires **only when residency is below a hysteresis fraction of the budget**:
`accountedBytes() < vram_budget_bytes_ × kReloadHysteresisFactor`
(`kReloadHysteresisFactor = 0.75`). So a reload can only run when there is real
headroom, and `evictIfOverBudget()` run after the insert (`onReloadFinished`) cannot
immediately shed the tile it just reloaded. `paint()` additionally guards the kick on
*viewport-moved-since-last-kick* (mirroring the `GggsTileLayer` demand-driven loader),
so a permanently-unloadable evicted index re-kicks at most once per distinct viewport,
never every frame. This is the GUI-thread-only (D4 / ADR-0006 D4) analogue of the
demand-driven worker in `GggsTileLayer` (ADR-0013).

The hysteresis blocks a *same-frame* reload↔evict ping-pong; it does **not** by itself
eliminate *cross-pan* churn, where panning back and forth across a boundary reloads a
region on one frame and evicts it a few frames later as a new region comes into view.
That churn is bounded, not pathological: (a) each reload batch is capped to a
quarter-budget of fine tiles (`kickReload`, using the last evicted tile's footprint), so
a wide zoom-out cannot reload the whole survey in one transient over-budget spike; and
(b) the reloads are display-grade off the disk cache — no network, no reconciler traffic
(D2 keeps possession, so a reload never re-requests). Both make the worst case a bounded
amount of local disk I/O, which is the accepted trade for pan-back recovering full
resolution. A stronger anti-churn scheme (e.g. a dwell timer before eviction) was judged
unnecessary at the lake/harbour envelope; revisit if a survey exercises it.

### D7 — Prune-on-absence propagates into the pyramid [field 2026-08-27]

D4 keeps overviews out of the reconciler; it does **not** exempt them from
prune-on-absence, though the code read as though it did (the `foldIntoParent()` comment
asserted the exemption as intent). The pyramid is a pure function of the fine tiles, so
the catalog is authoritative over it too: **an overview tile should exist exactly when
it is an ancestor of a live fine index.**

`SonarLiveCacheLayer::reconcilePyramid()` runs at the end of every `handleCatalog()`
reconcile, after the fine prune, and applies two distinct repairs:

- **Stale by absence.** An overview with no live descendant is entirely stale: remove it
  from `overview_tiles_`, free its GL texture under the renderer's context (the
  camp#134 discipline the fine prune already follows), and delete its `overviews/` file.
  The **directory itself is swept**, not just the resident map — eviction phase 1/D2
  frees an overview Entry while deliberately keeping its disk copy, so a stale overview
  can be absent from memory and still be warm-loaded back on the next restart. That is
  exactly how the pre-reset coverage kept returning.
- **Dirty by partial withdrawal.** An overview that kept some descendants but lost
  others holds folded contributions that cannot be subtracted: `foldChild()` only ever
  folds data IN and has no inverse. Such a tile is **rebuilt** from its surviving
  children rather than kept. The rebuild reproduces the original construction exactly —
  D3's chain folds the *whole* parent into the grandparent, so every level above the
  fine tiles is by construction "the fold of my children" — and it draws its inputs from
  a child repaired earlier in the same pass, else the resident copy, else the disk copy
  (D2 makes disk the durable backing for both pools). A tile with **no** surviving child
  is removed instead: a zoomed-out gap is honest, stale coverage is not.

Two properties are load-bearing and are pinned by
`test/test_sonar_live_overview_prune.cpp`:

- **Proportionality.** Every fine tile's ancestor chain reaches level 0, so invalidating
  the chain of each pruned tile would destroy the whole pyramid on any ordinary
  retraction — and tiles are withdrawn and re-added in normal operation. Nothing is
  removed while a live descendant remains, and only ancestors of something that really
  was withdrawn are rebuilt, so the work is proportionate to what actually went away.
- **Prune-gate parity.** A `generation_time` of 0 disables prune-on-absence in the
  reconciler (ADR-0008 D4b: no held version is strictly older than 0), so it disables
  the pyramid sweep too. The live set is also unioned with the ancestors of the fine
  tiles still held locally, so a held tile that was absent from the catalog but too
  *new* to prune keeps its ancestors alive.

The budget accounting (D1) is unaffected: removals shrink `overviewResidentBytes()`, a
rebuild that brings a non-resident overview back into memory is followed by
`evictIfOverBudget()`, and overviews still never enter `reconciler_` (D4 stands).

## Consequences

- Long surveys no longer grow resident memory/VRAM without bound; the crash scenario
  (#153) is closed. Total residency = fine tiles + near-view overviews to the budget,
  plus the O(small) protected apex. A one-time `qWarning` logs when the shed path is
  first entered. (Residual: the protected apex grows *minimally* — at the coarsest
  resolution — with survey *extent*, unavoidable if whole-survey zoom-out must always
  show coverage; negligible for the lake/harbour envelope.)
- Warm-load is bounded: it loads fine tiles incrementally and trims every 64 inserts,
  so enabling a source with a large disk cache can't spike memory on enable (the
  salmon accelerant) — not just a post-load trim.
- Evicted coverage degrades gracefully to a coarser resolution rather than vanishing,
  and — with D2/D6 reload — recovers to full resolution when the operator pans back.
- **Overview memory converges to the 1.33× series.** Because 4 same-size fine tiles
  collapse into 1 same-size uniform parent (D3), the full pyramid over a densely
  covered region sums to `1 + ¼ + 1/16 + … = 4/3` of one level — eviction frees
  **real** memory (4 fine buffers → 1 parent buffer, not a same-count re-tiling). For a
  *linear* lawnmower survey the near-fine collapse is closer to 2→1 (a ~2× series)
  because few fine tiles complete a 2×2 block; the protected apex plus the evictable
  near-fine overviews (D1 phase 2) still keep total residency bounded to the budget.
- **No on-disk migration.** The overview geometry and sidecar layout are **unchanged**
  from the geometry shipped in #160 (uniform edge, `overviews/<level>_<row>_<col>.tif`
  — already the uma layout); the camp#171 change is documentation-only convergence, so
  existing `overviews/*.tif` remain valid and warm-load unchanged. Overviews are
  display-grade and self-heal on the next eviction/regen regardless.
- The `LiveTileCache/max_vram_bytes` budget is self-accounted today; when the #155
  `ResourceMonitor` lands, its VRAM feed replaces `accountedBytes()` self-accounting
  behind the same knob (a named integration seam, not in this change).
- **[field 2026-08-27] Prune now reaches the pyramid** (D7). A retracted region loses its
  coarse coverage and its `overviews/` files instead of keeping them forever, and a
  boat-side store reset is reflected on the next catalog rather than never. The costs
  are accepted and bounded: each catalog reconcile lists the `overviews/` sub-dir
  (published on change, not at rate), and a repair re-folds only the ancestors of what
  was actually withdrawn, preferring resident children so the common case does no disk
  I/O at all.
- **Deferred follow-ups:**
  - Pyramid content that went stale *before* D7 shipped, in a session whose descendant
    overviews were already correctly removed, is not detectable: the repair keys off
    what this reconcile withdrew, and overviews carry no provenance. Not reachable going
    forward, and not reachable for the state D7 shipped into (the stale `overviews/`
    files were all still present, so the first reconcile withdrew them and rebuilt their
    ancestors). Persisting per-overview provenance was rejected — it cannot survive
    warm-load, which is precisely the path that resurrects a stale pyramid.
  - Nightly-regen anti-clobber (a regenerated world store re-publishing a catalog that
    momentarily disagrees with the live one) is untouched by D7 and remains open.
  - A write-through already in flight for a tile the sweep deletes can still land its
    file after the `remove()`. `cancelPendingWrite()` stops the queued/coalesced case;
    an in-flight worker cannot be cancelled without reintroducing the two-workers-one-
    tmp-path race the coalescing exists to prevent. The next catalog's directory sweep
    deletes it again, so the window self-heals rather than persisting.
  - `foldChild` silently drops a child cell whose geographic centre falls outside the
    parent (only reachable across a ±72°/±80° GGGS latitude-band boundary) → a possible
    overview seam on a high-latitude survey; add a boundary test / handling if such
    surveys arise.

## Alternatives considered

- **Distance-from-vessel eviction** (keep tiles near the survey head): rejected by
  the operator as needless complexity and a tf dependency; view-based LOD is the
  natural model for a pan/zoom display.
- **Fixed 64×64 overview tiles** (decouple overview size from fine size): rejected in
  favour of matching fine dimensions (standard pyramid, uniform tile size). This choice
  is exactly what makes the overview geometry **converge with the uma shared fold
  engine** (D3, camp#171) — the uniform edge is the same `TiledRasterTile::edge` the
  merged store folds at — so it is reinforced here, not reversed.
- **`⌊W/2⌋` quarter-resolution overview tiles** (halve each parent's pixel dimensions
  to force more per-level savings): rejected — it would diverge from the merged store's
  fidelity at the same zoom and violate `foldChild()`'s same-size parent/child
  invariant (see D3).
- **Fold-on-arrival** (keep overviews continuously fresh): rejected as needless
  compute; fold-on-evict suffices for a display-grade preview.
- **Producer-side overviews** (boat emits a coarse level, serving slow-link
  first-paint before any fine tiles are pulled, #71): a separate, deferred bandwidth
  feature; this ADR is consumer-side only.
