# ADR-0014: GGGS tile-layer viewport-scoped residency and budget

## Status

Accepted

Implements camp issue #195. The display-side implementation of
`uma-ADR-0013` **D4** ("Residency is budgeted, and pressure relaxes quality
rather than thrashing") for `GggsTileLayer`.

Amends [ADR-0013](0013-lod-level-selection-demand-driven-load.md): that ADR's
"levels ≤ the selection stay resident permanently" residency rule is superseded
here, and its "the only release path is the level switch" statement gains a
second, viewport-keyed path. It does **not** change ADR-0013's LOD selection or
its demand-driven load — this ADR governs only what is *kept*.

*Amendment form*: the ADR-0013 passages this change touches are annotated in
place (`*Amended by camp#195*`), which is the addendum shape the workspace's
own ADR-0012 asks for. One passage — its "Residency bound" paragraph — was
instead **rewritten**, because its central claim ("`GggsTileLayer` has no
residency/eviction budget") became false wholesale rather than partially; the
rewrite states what the paragraph originally recorded before correcting it, and
camp's own ADR-0010 has the precedent (camp#171/#172 added a whole decision to
an accepted ADR). Noted here so the divergence is recorded rather than assumed.

Sibling, not parent: [ADR-0010](0010-bounded-eviction-overview-pyramid.md)
answers the same forces for `SonarLiveCacheLayer`. The two layers face different
data contracts, and where the answers diverge this ADR records why.

**Citation convention** (from `uma-ADR-0013`): cross-repo ADR references are
repo-qualified — `camp-ADR-00NN` / `uma-ADR-00NN`. Within this repo, relative
links to sibling ADRs stay unqualified.

## Context

`GggsTileLayer` had exactly one code path that ever freed a tile: the release
loop in `tilesReady()`, which fires after a level switch and frees only tiles at
levels *finer* than the selection. **No release was ever driven by the
viewport.** At a fixed zoom, every tile the view scrolled across stayed resident
for the life of the session.

Per-tile cost is high and deliberately so: `GggsTile::texture()` retains the CPU
buffer past the GPU upload (camp#180 — the depth-at-cursor readout samples it),
so a painted 960×960 tile holds a 3.52 MiB `std::vector<float>` **plus** a
3.52 MiB R32F texture ≈ 7.03 MiB. A hundred panned-over tiles is ~700 MiB.

The loader was already viewport-filtered on the way *in* (ADR-0013), which is
the trap this ADR exists to name: **a viewport-filtered loader is not a
residency bound.** Filtering what enters bounds the rate of growth, not the
total; residency equalled the union of every viewport ever visited.

`SonarLiveCacheLayer` had this identical shape and OOM'd the salmon operator
station shortly after live data was enabled (camp#153), which is why ADR-0010
exists. `GggsTileLayer` is the layer that never got the treatment. The Isles of
Shoals work puts a 1 m Appledore MLLW grid into the `reference` layer, in front
of an operator panning across it repeatedly during survey operations — the exact
accumulating usage pattern.

## Decision

### D1 — A byte budget, converted to a tile count from the observed tile size

`QSettings GggsTileLayers/max_resident_bytes`, default **512 MiB**, `0` disables
eviction entirely (the pre-camp#195 behaviour). The key mirrors
`LiveTileCache/max_vram_bytes`, whose default is the same 512 MiB, so the two
co-resident working sets are expressed in the same units and are tunable
together on the host (camp#117 / ADR-0006 D2: a default, never an un-changeable
hardcode).

`uma-ADR-0013` D4 explicitly blesses a **count** budget for fixed-size tiles and
names GGGS's 960×960 as such a case. We keep the count *mechanics* — the cap,
the protection, and the eviction loop all work in tiles — but derive the count
from a byte target and the **observed** per-tile cost
(`max(width × height) × 4 × 2` over the tile-set), because `width_`/`height_`
come from GDAL: 960×960 is a store convention, not an invariant, and a hardcoded
count would silently mis-size on any store that departs from it. The factor of
two is the retained CPU buffer plus the GL texture.

*Why 512 MiB is acceptable next to the live cache's own 512 MiB*: the camp#153
OOM precedent is **salmon**'s hardware. The Shoals operator station is **pandy**,
not salmon, and pandy has the headroom for both working sets. The number is a
default for a machine we know, not a portable claim — which is exactly why it
lives behind a settings key: it is expected to be tuned on the station during a
rebuild day rather than in a rebuild of CAMP.

The principled budget number is camp#155/#156 (ResourceMonitor: self-sampled
footprint and GPU-byte accounting). This ADR deliberately does not block on
them; the measured value wires in through the same seam later.

### D2 — Current-frame protection is structural, and comes from the loader predicate

`uma-ADR-0013` D4 requires "cannot evict what this frame selected" to be a
property of the data structure rather than a checked condition. `TileResidency`
(`raster/tile_residency.h`) is that structure: two `std::list<std::size_t>`
partitions (protected / evictable) plus an iterator index and a frame epoch, so
`beginFrame()` and `protect()` are O(1) splices and `candidates()` — the only
accessor an eviction pass gets — exposes the evictable partition alone. There is
no accessor that yields a protected index as a candidate.

The protected set is derived from **the loader's predicate**, not from the draw
list: tiles intersecting the load viewport at a level ≤ the selection. This is
not a stylistic choice. `cached_image_` short-circuits `itemsIntersecting()` on a
static frame, so on exactly the frames an eviction is most likely to run, a
draw-list-sourced protection would protect **nothing** and the live visible set
would be evictable. Not-yet-loaded tiles are counted: they are part of the
working set the cap must accommodate. Terminally-failed tiles are not — they
never become resident, so counting them would inflate the floor with memory
nothing will ever occupy.

Two classes of in-view tile **finer** than the selection are protected on top of
the loader's set. Both are drawn, and neither can reload if dropped
(`loadTilesWorker()` skips `level > selection`), so for both the "eviction is
free because the reload path exists" premise fails:

- **an already-resident finer tile.** `itemsIntersecting()` draws the whole
  resident set with no level filter, so during a **zoom-out** these tiles are
  the entire visible picture until the coarser selection finishes loading —
  camp#103/#194's field-verified no-blank-frame guarantee, which the budget
  would otherwise quietly undo under pressure. Protecting them leaks nothing:
  `tilesReady()`'s level-switch release still drops them at the moment the
  guarantee ends.
- **a camp#194 hole coverer** (D5).

The loader predicate is still the right *base*, for the `cached_image_` reason
above; these two are additions to it, not a return to the draw list.

Two placement rules follow, and both are load-bearing:

- In `paint()`, protection is refreshed **after** the load viewport is set and
  **before** every remaining early return. `refreshProtection()` begins by
  splicing the whole protected partition back into the evictable one, so a
  return between "begin" and "protect" would leave the live visible set
  evictable.
- The eviction pass **re-derives** the protected set live before choosing
  candidates (the `MapTiles`/camp#98 rule). The pass is queued and debounced, so
  the protection captured at schedule time may be several pan steps stale, and a
  tile that re-entered the view in between must never be evicted.

### D3 — The cap is floored at the working set; over-budget is reported, not thrashed

`cap = max(budget_tiles, protected_count)`.

`uma-ADR-0013` D4: "the budget must exceed [the tiles selected this frame] or the
system thrashes by construction". A hard ceiling that can fall below the current
frame's own selection would evict what the layer is drawing, re-load it next
frame, and evict it again. So when one viewport's working set alone exceeds the
byte target, the layer **exceeds its budget deliberately** and says so in its
status line ("over the tile budget — raise `GggsTileLayers/max_resident_bytes`
or zoom in"). Eviction runs down to `max(protected_count, 0.75 × cap)`, the
hysteresis analogue of ADR-0010 D6, so the next frame cannot immediately
re-trigger it.

D4's other half — *relax the quality target* (raise `τ`, select a coarser level)
under this pressure — is **deferred to camp#197**. As originally specified it
oscillated deterministically: degrade → the protected set shrinks ~4× → a
cool-down relaxes it → the trigger fires again, with a period equal to the
cool-down. A stable relax needs the *predicted* protected count at the finer
level under a hysteresis factor, not a timer. Deferring is safe because the
unbounded-pan accumulation — the actual OOM, and the camp#153 precedent — is
fully fixed by the budget alone; the lever only addresses the narrower case
where a single viewport is itself over budget. camp#195's own requirement —
"Report the degraded state; never fail silently" — is met meanwhile by the cap
floor plus the reported state. (D4 itself puts it as "the display degrades
visibly and predictably rather than churning"; the crisper phrasing is the
issue's, not the ADR's.)

### D4 — Drop and re-read, not fold-to-parent

ADR-0010 folds an evicted fine tile into its coarse parent before dropping it,
because a live tile arrives **once** over ROS and its only durable copy is the
cache on disk. A GGGS tile's source of truth *is* the file on disk, and
ADR-0013's demand-driven loader already re-reads a tile when the viewport
returns to it (`hasUnloadedVisibleTiles()` plus the moved-since-last-kick
re-kick). `resetPixels()` + `releaseGL()` is therefore a complete release, and
the reload path costs nothing to build.

Building a CAMP-side pyramid instead would also manufacture a derived product
for the two store classes `uma-ADR-0010` D9 deliberately does not generate one
for: `chart` ("LOD arrives built-in — the scale ladder is a cartographer-curated
pyramid … No pyramid generation") and `reference` ("as imported"). Only
`draft`/`processed` get generated overviews there, and those are
shallowest-preserving — a fidelity property a display-side fold would not
reproduce.

Cost, stated plainly: an evicted area **blanks** until its reload lands, rather
than degrading to coarser coverage. Two things buy that back. On a real ladder
every level ≤ the selection is resident, so a panned-away area usually retains
coarse coverage; and the coarsest available level is exempt from eviction (D6).
On a single-level store neither applies and the blank stands — a deliberate
trade, since the alternative is manufacturing data the store does not have.

### D5 — camp#194 hole coverage: protected in view, last-resort out of view

camp#194 established that a tile **finer** than the selection which intersects a
`loadFailed()` tile at a level ≤ the selection is the *only usable coverage over
that hole*, and must survive `tilesReady()`'s level-switch release. The budget
must not silently undo that, so the same predicate (`coversHole()`, one helper
shared by both release paths, so the two rules cannot drift) governs the
eviction pass:

- **In the viewport** — protected, exactly like a selected tile.
- **Outside the viewport** — a *last-resort* candidate, ordered behind every
  ordinary candidate.

The asymmetry exists because such a tile is one of the two classes for which
"the reload path is free" is **false** (the other is D2's zoom-out backdrop):
`loadTilesWorker()` skips `level > selection`, so an evicted hole coverer cannot
come back on its own. It returns when the operator **zooms in** past it (the
finer level becomes the selection, so the loader will read it again), or when
the underlying failure is repaired and **Rescan** picks the repair up — note
that Rescan alone does *not* reload the evicted finer tile; it helps by closing
the hole a different way. The status line therefore names both actions
("coverage over failed tile(s) released — zoom in or Rescan") rather than losing
the coverage silently.

That report is **live, not latched**: it is shown only while some tile is still
failed (no failure, no hole, so the message is moot) and `rescan()` clears it
outright, since Rescan is the operator's explicit retry and the flag must
describe the current state rather than a past event. A message that survived the
recovery it advertises would be its own kind of silent failure.

*Alternative considered and rejected*: extending the loader's ceiling to admit
finer tiles over a failed footprint would make them reloadable and remove the
special case. But a failed **coarse** tile spans an enormous footprint, so at low
zoom this would pull every fine tile in the viewport into the load — the
store-bounded open ADR-0013 exists to avoid — and inflate the protected floor
with it.

### D6 — Hybrid eviction ordering, and a bounded coarsest-level exemption

Candidates are ordered by, in priority:

1. **last-resort tier** — out-of-view hole coverers (D5) and surplus
   coarsest-level tiles go last;
2. **staleness** — a tile not seen within the last ~120 paints goes before one
   seen recently;
3. **distance** — farthest from the viewport centre first;
4. **LRU generation** — oldest last-visible first.

Staleness sits above distance on purpose. ADR-0010's ordering is
distance-primary with LRU only as a headless fallback, and `uma-ADR-0013` D4
objects to exactly that: "LRU alone retains a useless working set after a
viewpoint jump; distance-only ordering discards history along a path being
traversed." Ranking a recently-traversed tile behind a never-seen one is the
cheapest way to keep the track. The viewport centre is the clip's own centre,
already computed every `paint()`; with CAMP's single MapView it is the painting
view's centre (recovering the painting view under multiple views is camp#150).
A null viewport — headless, or a layer that has never painted — is *not*
"LRU instead of distance": the protection predicate skips the viewport filter
entirely, so every non-failed tile at a level ≤ the selection is protected, the
cap floors at that whole set, and eviction is inert. That is deliberate — it is
what keeps ADR-0013's "Headless / no-selection defaults" (load and render
everything) true — and it is why the headless tests set an explicit test
viewport. Only tiles finer than the selection remain candidates in that state,
and with every distance 0 they order by generation, i.e. pure LRU.

Tiles at the **coarsest available level** are exempt, the analogue of ADR-0010's
protected apex: on a ladder that guarantees a zoom-out always has something to
draw. Two bounds keep the exemption from becoming the leak it is meant to
prevent:

- it applies only when the ladder has **more than one** level — on a
  single-level store every tile is at the coarsest level, and an unbounded
  exemption would make the budget a no-op;
- at most `min(kCoarsestExemptCap, cap / 4)` such tiles are exempt,
  nearest-first; the surplus — which is what grows with the area panned — joins
  the last-resort tier instead of accumulating. The bound is relative to the cap
  as well as absolute because a flat 64 can exceed the eviction target on a
  small budget (64 against a ~57-tile target at the 512 MiB / 960×960 default),
  and exempt tiles still count toward residency: the victim loop could then never
  reach its target and would shed every ordinary candidate on every pass —
  eviction/reload thrash. An exemption that can outgrow the target is a second,
  unevictable budget, not a floor.

`available_levels_` is rebuilt by `rescan()`, so the exemption is evaluated per
pass and never cached.

### D7 — Deferred, debounced eviction that re-arms when the loader is busy

`paint()` only *schedules* the pass (queued invocation, debounced by a pending
flag): releasing a tile inside `paint()` would mutate the set the render pass is
reading. The pass itself must not run while the loader worker is iterating
`tiles_`, so it checks `future_watcher_.isRunning()` — and when it finds the
worker busy it **re-arms** on a short timer with the debounce still held.

Not dropping the request is the whole point. A continuous pan re-kicks the
loader at every step, so a pass that simply returned when it found the worker
running would almost never fire during a sustained pan — precisely the scenario
the budget exists for. Abort-and-join (what `loadTiles()` does before mutating
`tiles_`) was rejected here: it would stall the GUI thread for a whole tile
read, mid-pan.

But a timer re-arm alone is a **blind resample**, not a rendezvous. During a
sustained pan the idle window between one worker finishing and `paint()`
re-kicking is about one event-loop turn inside a duty cycle dominated by tile
reads, so a 100 ms retry lands in it only by luck — the budget could be starved
for many seconds of continuous panning while residency grows. The deterministic
rendezvous is `tilesReady()`: it runs on the GUI thread immediately after the
worker's join and before any `paint()` can re-kick, so a pending pass is
**drained there**. The timer stays as the backstop for the case where nothing
completes.

### D8 — The auto-range is not recomputed on eviction

`foldDataRange()` widens only, and ADR-0013 (as amended by camp#194) already
established that a level switch must not reset it. Eviction is the same case,
and more clearly so: an evicted tile's pixels are **unchanged on disk**, so the
contribution it already made stays true. Re-deriving the range from a shrinking
resident set would only make the colormap flicker as tiles come and go. The one
case that *does* invalidate a contribution — `rescan()` replacing a tile's file
— keeps its reset-and-recompute path, unchanged.

## Consequences

- Residency is bounded by the viewport rather than by the union of every
  viewport ever visited. A pan of arbitrary length over a store of arbitrary
  size stays within the budget, floored at whatever the current frame needs.
- `getElevation()` (camp#180) returns NaN over an area the view has panned away
  from and the budget has released. In practice the readout is unaffected: the
  cursor is always inside the viewport, and viewport tiles are structurally
  protected. Asserted in `test/test_gggs_elevation.cpp`.
  Note for camp#197: raising `τ` under pressure would weaken ADR-0013's
  "`getElevation()` samples finest-covering-tile-first" fidelity mitigation,
  because the finest covering tile may no longer be resident. That interaction
  belongs to whoever implements the lever.
- An evicted area blanks briefly on pan-back until the re-read lands (D4). On a
  ladder it usually shows coarse coverage instead.
- The budget is **per layer**. N GGGS layers multiply the process footprint, and
  nothing here accounts for the co-resident `SonarLiveCacheLayer` budget.
  Process-wide accounting is camp#155/#156; this is a known gap, not an
  oversight.
- Eviction adds an O(n log n) candidate sort per pass, off the paint path and
  debounced. `paint()` itself pays more than the protection sweep alone: the
  sweep (with a `failedFootprints()` scan inside it), plus `residentTileCount()`
  — each O(tiles), each with a `tileSceneRect()` geo→Mercator transform per
  tile. `perTileResidentBytes()` is memoized against `tiles_.size()` so it is
  not a third pass. This is real per-frame GUI-thread work on a
  thousand-tile store, and it lands on frames that previously short-circuited on
  `cached_image_`. Accepted for now — the sweep is what keeps the LRU generations
  honest — but it is the first thing to profile if pan latency regresses
  (alongside camp#198's overdraw).
- If a GL context exists but refuses to become current, `releaseTiles()`
  deliberately releases **nothing**, to keep the `resetPixels()`/`releaseGL()`
  pairing invariant intact. `RasterGlRenderer` latches that failure without
  destroying the context, so the condition can be permanent — the budget then
  stops being enforced for the rest of the session. The layer reports that
  state ("tile budget NOT enforced (GL context unavailable)") rather than
  growing silently toward the camp#153 OOM. Freeing the CPU half alone would in
  fact be safe once the renderer's GL-failed flag is latched (no new texture can
  ever be uploaded, so nothing could shadow a re-load), but that reasoning
  depends on a renderer internal the layer cannot currently observe; exposing it
  is a follow-up, not a silent assumption.
- A malformed `GggsTileLayers/max_resident_bytes` (`"512M"`, a stray quote)
  converts to 0, which is the *disable* sentinel — a typo in the hand-edited
  operator-station settings would silently restore the pre-camp#195 unbounded
  behaviour. The layer detects the parse failure, warns, and falls back to the
  default; only an explicit `0` disables.
- **Coverage gap**: the headless tests exercise only the CPU half of the
  release. With no GL context `releaseGL()` is a no-op, so the texture half —
  the larger part of the 7.03 MiB — is not covered by an automated test. The
  pairing invariant it protects (`gggs_tile.h`: a CPU-only clear leaves a stale
  texture shadowing any re-load) is enforced in one shared helper precisely
  because it cannot be asserted here.
- The reusable lesson, recorded because it was mis-diagnosed twice: **a
  viewport-filtered loader is not a residency bound**, and multi-level
  compositing did not cause this accumulation — it predated camp#194 and needed
  no multi-level store.

## Alternatives considered

- **Fold-to-parent before dropping (ADR-0010's answer)** — rejected in D4: the
  tiles are files, the reload path already exists, and a CAMP-side pyramid would
  manufacture a derived product `uma-ADR-0010` D9 withholds.
- **A fixed tile count (e.g. 64 tiles ≈ 450 MiB)** — rejected in D1: it hides a
  960×960 assumption that GDAL does not guarantee, and it is not comparable to
  the co-resident live-cache budget.
- **A hard maximum cap with no floor** — rejected in D3: it is D4's "thrashes by
  construction".
- **The `τ` degrade lever in this change** — deferred to camp#197 in D3: as
  specified it oscillates with a period equal to its cool-down.
- **Per-kick loader volume cap** — rejected. The kick set is already
  viewport-bounded (it *is* the protected set), and truncating a kick would
  leave a stationary operator with a permanently incomplete picture: the re-kick
  guard requires the selection or viewport to change before it will fire again,
  and the load would settle with a cleared status over the gap.
- **Recovering the painting view in `deriveViewportClip()`** — out of scope. The
  eviction metric needs a view centre, and the clip's own centre already is the
  painting view's centre under CAMP's single MapView. Multi-view recovery is
  camp#150.
