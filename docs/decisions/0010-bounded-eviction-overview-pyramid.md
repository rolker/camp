# ADR-0010: Bounded live-tile eviction + consumer-side overview pyramid

## Status

Accepted

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
`0` disables — the pre-#160 behaviour) caps the **resident fine-tile** footprint
(CPU band data + any uploaded GL texture; overviews are excluded, being bounded and
intentionally resident). `evictIfOverBudget()` runs on the GUI thread after every
applied patch and after warm-load. When over budget it evicts fine tiles **farthest
from the current viewport centre first** (classic slippy-map LOD — keep what's near
the view, discard the rest), obtaining the viewport centre from
`scene()->views()`. When no view is attached (headless / tests) it falls back to
**LRU** by a monotonic `last_access_seq` (bumped on patch and on render). No
vessel-position / tf dependency — the operator explicitly chose view-based LOD over
distance-from-vessel as the simpler, more natural model.

### D2 — Persist-then-drop; keep possession in the reconciler (no churn)

On eviction a tile is (a) folded into its overview parent, (b) written through to
the disk cache (reusing the existing coalesced `scheduleWriteThrough` path), then
(c) its GL texture is freed under the renderer's context and it is erased from
`tiles_`. The reconciler's `markHave` for that index is **kept, not dropped**: we
still *possess* the tile (it is on disk), so the next catalog reconcile will not
re-request it. `tiles_` tracks residency (memory); the reconciler tracks possession
(disk) — the two intentionally decouple under eviction. Dropping it would cause a
re-request → re-receive → re-evict churn that wastes the very bandwidth #71 guards.
A genuine later catalog prune still frees the disk copy via `handleCatalog`.

**Consequence / known limitation:** an evicted fine tile is not re-loaded from disk
when the operator later pans back into its area within the same session (there is no
view-change → disk-reload hook yet); the coarse overview covers it until the layer
is re-enabled or the boat re-sends a newer version. On-demand disk reload on view
change is a deferred follow-up.

### D3 — Overview pyramid: match-resolution, fold-on-evict, full chain to level 0

`foldIntoParent()` decimates an evicted fine tile into the GGGS `parent()` of its
index and recurses up to level 0, so a zoomed-out view always has coverage. Each
overview tile **matches the fine tile's width/height** (standard pyramid: every tile
is the same pixel size; a parent W×H tile covers its ~4 children at half linear
resolution). `SonarLiveTile::foldChild()` area-maps each child cell to the parent
cell containing its geographic centre and averages the finite, non-NoData samples —
area-mapping (rather than a fixed 2×2 quadrant) keeps the polar `latitudeScaleFactor`
column scaling correct. Overviews are folded **on eviction only** (cheapest; the
overview may lag the latest fine data until first eviction, acceptable per ADR-0006
D1). They persist to an `overviews/` sub-directory of the cache dir (own
`<level>_<row>_<col>.tif` files) so warm-load of fine tiles can't pick them up.

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

## Consequences

- Long surveys no longer grow memory/VRAM without bound; the crash scenario (#153)
  is closed. A one-time `qWarning` logs when the shed-load path is first entered.
- Warm-load is bounded: enabling a source with a large disk cache trims to budget
  before subscribing, so it can't spike memory on enable (the salmon accelerant).
- Evicted coverage degrades gracefully to a coarser resolution rather than vanishing.
- The `LiveTileCache/max_vram_bytes` budget is self-accounted today; when the #155
  `ResourceMonitor` lands, its VRAM feed replaces `accountedBytes()` self-accounting
  behind the same knob (a named integration seam, not in this change).
- Deferred: on-demand disk reload of an evicted fine tile when the operator pans back
  into its area (D2 limitation).

## Alternatives considered

- **Distance-from-vessel eviction** (keep tiles near the survey head): rejected by
  the operator as needless complexity and a tf dependency; view-based LOD is the
  natural model for a pan/zoom display.
- **Fixed 64×64 overview tiles** (decouple overview size from fine size): rejected in
  favour of matching fine dimensions (standard pyramid, uniform tile size).
- **Fold-on-arrival** (keep overviews continuously fresh): rejected as needless
  compute; fold-on-evict suffices for a display-grade preview.
- **Producer-side overviews** (boat emits a coarse level, serving slow-link
  first-paint before any fine tiles are pulled, #71): a separate, deferred bandwidth
  feature; this ADR is consumer-side only.
