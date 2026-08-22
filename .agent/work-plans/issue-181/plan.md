# Plan: Depth layers: land/water-distinguishing topo-bathy colormap pivoted at chart datum

## Issue

https://github.com/rolker/camp/issues/181

## Replan notice — this supersedes the plan committed at `56ac199`

The previous plan built a `chart_datum_service` that queried VDatum **at
runtime, per render**, to compute a per-region chart-datum pivot. The operator
directed a redesign at the `/run-issue` checkpoint because that contradicts the
project's colormap vision (`docs/vision.md` on `feature/issue-12` of
`rolker/marine_colormap`, PR mc#14, unmerged). Verbatim from it:

- *"There is no `chart_datum` runtime frame, and that is a decision, not an
  omission."* (uma ADR-0010 D5 removes it.)
- *"Datum conversion happens **at import**, through the ROS-free
  `marine_vertical_datum` library, not at runtime."*
- *"Breakpoints are constants in the chosen frame. Express the field relative to
  `map_tide` and the shoreline break is 0.0 ... permanently."*
- *"The transform is where the tide-awareness lives, applied once, upstream."*

So: **no runtime datum service, no VDatum, no PROJ, no grids, no per-region
query.** The anchor is the sea surface, read from the `map_tide` frame. The
whole `chart_datum_service` step, the `~`-expansion bug, the grid-provisioning
question and the GUI-thread PROJ stall are dropped as moot.

The mechanism the old plan chose — `BreakpointMap` anchoring inside the LUT
bake — survives unchanged and is still right. Only the **source of the anchor
value** changes, and that change deletes most of the risk.

## Ground truth (verified this session by reading source, not assumed)

- **`sea_surface_estimator` exists** — it is a node inside `mru_transform`, not
  its own package: `layers/main/platforms_ws/src/mru_transform/mru_transform/nodes/sea_surface_estimator.cpp`.
  It publishes `map → map_tide` with `sea_surface_frame` defaulting to
  `map_tide`. That resolves the handoff's "could not find it" flag.
- **The `map_tide` read pattern is settled**, in `bathymetry_layer.cpp:653-672`:
  `lookupTransform(map_frame, map_tide_frame, TimePointZero).transform.translation.z`
  is the sea-surface **ellipsoidal** height, because `map`'s z=0 is the WGS84
  ellipsoid — the same datum the depth stores use. Its guard rails are worth
  copying wholesale: refuse when `map_frame == map_tide_frame` (the #220
  degenerate self-lookup that read a whole survey as LETHAL), never treat the
  default 0.0 as a valid surface, and re-render only when the tide moves more
  than `tide_invalidate_threshold` (0.1 m, chosen to clear ~±0.02 m estimator
  jitter).
- **`/tf` and `/tf_static` are bridged to the operator station** over udp_bridge
  (`bizzyboat_project11/config/bizzyboat.yaml:312-313`), so `map_tide` is
  reachable from CAMP at the ROC *when the boat is up*. Whether it is actually
  present on the ROC machine on Tuesday is a verification item, not an
  assumption — see Open Questions.
- **CORRECTION to the handoff — the raster layers cannot call TF.** camp does
  own a `tf2_ros::Buffer` (`src/camp/ros/node_thread.h`, `camp_map/ros/layer.cpp`),
  but `RasterLayer`, `GggsTileLayer` and `RasterGlRenderer` all live in
  **`libcamp_map`, which CMakeLists.txt:239 declares "pure Qt/GDAL (ROS-free)"**
  with the boundary "verified one-directional: the src/camp_map core has zero
  ROS includes" (ADR-0002). `GggsTileLayer`/`RasterLayer` derive from
  `map::Layer`, **not** `camp::ros::Layer`, so they have no `node_`. A TF call
  inside them would breach the layering AND make the existing headless GL tests
  (`test_gggs_render.cpp`, `test_gggs_band_select.cpp`) host-dependent. The
  anchor value must therefore be **pushed in** from the ROS side, not pulled.
- `marine_colormap::BreakpointMap` (`lookup.hpp:267`) clamps every degenerate
  input and never throws; a break outside the domain is documented as the
  *ordinary* case. `PaletteDomain::shoreline_position` (`palette.hpp:56`) is
  "metres, positive up", matching the stores' ellipsoidal up-positive heights —
  so the anchor value is directly comparable with no sign flip. `oleron`/
  `hypsometric` carry it; the other four palettes do not.
- `ColormapLegendWidget::setLut()` **already exists**
  (`colormap_legend_widget.hpp:65`) and overrides the linear palette sampling at
  `colormap_legend_widget.cpp:185`. The colorbar correctness problem is
  therefore a three-line *fix*, not a documented gap.
- `web_mercator::mapToGeo()` already exists (`web_mercator.h:28`) — but under
  this design nothing needs it.

## Design decisions this plan makes explicitly

**D1 — Uniform shift, not GeoZui4D's asymmetry.** The vision quotes
`gutm.cpp` shifting only submerged terrain and leaving emergent terrain at its
datum-referenced elevation. **This PR does not implement that asymmetry, and the
reason is our frame.** GeoZui4D's land branch works because its land heights are
datum-referenced with 0 at the datum. Ours are **ellipsoidal**: land at the
waterline near Portsmouth is ≈ −28 m. Keeping `h` for land and `h − S` for water
would open a ~28 m discontinuity exactly at the shoreline — the opposite of what
this issue asks for. A uniform shift makes 0.0 the shoreline everywhere and
keeps the field continuous. Its cost is that a hill's *displayed* height moves
with the tide by the tidal range (±~1.5 m out of tens of metres). Accepted, and
recorded in the ADR; the asymmetry becomes correct only once the field carries
orthometric land elevations, which is future work, not this PR.

**D2 — The shift is expressed as an anchor, not as arithmetic on the data.** A
uniform shift is invisible to a linear ramp: `t = (v−lo)/(hi−lo)` is
shift-invariant, so subtracting `S` from every texel would change nothing. The
shift matters *only* because it moves the break. So the implementation anchors
the palette's `shoreline_position` at the data value `S` in the unshifted
ellipsoidal frame — mathematically identical to shifting the field and breaking
at 0.0, with no texture rewrite and no shader change.

**D3 — Measured-local tier only.** `map_tide` is a single scalar at the vessel,
not a surface. Over a display spanning Boston to the Isles of Shoals one value
is applied everywhere, so the anchor is wrong far from the boat by the spatial
tide gradient (order tens of cm over ~50 km, and larger up an estuary). For a
nearshore survey with the boat inside the view this is well inside the useful
band; for a wide-area chart it is a known approximation. The vision's modelled
global tier ("local costmap vs global costmap") is named as future work, not
built here.

**D4 — No tide source must never mean "anchor at 0.0".** Anchoring at 0.0 in
the ellipsoidal frame puts the land/water break ~28 m into deep water — worse
than today. With no tide the layer falls back to its manual anchor if the
operator set one, else renders **unanchored** (byte-identical to today) and says
so in its status.

## Approach — sequenced so operator-visible correctness lands first

### Phase A — anchoring mechanism + manual anchor (ROS-free, no boat needed)

This phase alone fixes the reported symptom and is verifiable on the dev host.

1. **`src/camp_map/raster/pivoted_lut.{h,cpp}` (new).** Pure, no GL, no I/O:
   ```cpp
   std::vector<marine_colormap::Rgba8> bake_anchored_lut(
     const marine_colormap::Palette& palette, float lo, float hi,
     std::optional<float> anchor_value, std::size_t n);
   ```
   When `anchor_value` **and** `palette.domain()->shoreline_position` are both
   present: build `BreakpointMap(lo, hi, {{*anchor_value, *shoreline_position}})`,
   and for each `i` take `v = lo + (hi−lo)·i/(n−1)`, `t = map.normalize(v)`,
   `palette.sample(t)`. Otherwise return
   `bake_lut(palette, TransferParams{}, n)` verbatim. Calling `Palette::sample()`
   directly rather than `bake_lut` makes ADR-0008 Consequence #1 (identity
   `TransferParams`) structurally unbreakable on this path, not merely honored.
2. **`RasterGlRenderer::setShorelineAnchor(std::optional<float>)`** + getter,
   mirroring `setColormap`'s dirty-flag pattern. `ensureLut()` now depends on
   `lo`/`hi` as well as the palette name, so it caches the last baked
   `(name, lo, hi, anchor)` and re-bakes when any changes. The shader is
   **untouched**.
3. **Manual per-layer anchor** — the interim form the issue itself proposes
   ("a per-layer constant pivot parameter ... user-set once per region"). Add an
   anchor field to `ColormapRangeState` / the "Colormap range…" dialog, persisted
   beside `range_min`/`range_max` under the layer's `settingsKey()`. Offered only
   when the active palette has a `shoreline_position`. **This is the piece that
   works on Tuesday with no boat, no TF and no network.**
4. **Fix the colorbar (ADR-0009 must-fix, properly).** With an anchor active,
   `colormap_range_dialog.cpp` calls
   `legend->setLut(bake_anchored_lut(palette, lo, hi, anchor, 256))` instead of
   `legend->setPalette(index)`. `setLut()` already overrides the linear sampling
   at `colormap_legend_widget.cpp:185`, so the colorbar's colour↔value mapping
   becomes *correct*, not merely annotated — and `marine_colormap_widgets` needs
   no change. Also show the resolved anchor value as a label so the operator can
   read what the break is pinned to.

### Phase B — tide-linked anchor (crosses the ROS boundary; separable)

5. **`src/camp_map/raster/sea_surface_reference.{h,cpp}` (new, ROS-free).** A
   small `QObject` holding `std::optional<double> height` + a `changed()` signal,
   owned by the map (reachable via `parentMap()`), with **zero ROS includes** so
   `libcamp_map`'s verified boundary and the headless GL tests stay intact. Tests
   set the value directly.
6. **`src/camp_map/ros/sea_surface_tracker.{h,cpp}` (new, in `camp_map_ros`).**
   A `QTimer`-driven poller doing
   `lookupTransform(map_frame, tide_frame, TimePointZero).translation.z` and
   writing `SeaSurfaceReference`. Copies `bathymetry_layer`'s guards: reject
   `map_frame == tide_frame` outright; never accept a default 0.0 as a surface;
   only publish a change when it moves more than 0.1 m. Frame names come from
   `QSettings` (`SeaSurface/map_frame`, `SeaSurface/tide_frame`) and, when unset,
   are **auto-discovered** from the buffer's frame list — a unique frame whose
   name ends in `map_tide` plus its sibling `map` — because the real frames are
   namespaced (`bizzy/map_tide`) and no operator will hand-edit QSettings before
   a survey. Ambiguous or absent → no tide, honestly reported.
7. **Per-layer anchor source: `None | Manual | Tide`.** On
   `SeaSurfaceReference::changed()`, a layer records the value in a member,
   drops `cached_image_`, and requests a repaint. Tide falls back to Manual (if
   set), then to None (D4).
8. **Status, routed correctly.** `GggsTileLayer` reports through
   `updateStatus()` — camp#195's single composer — as a new *part*
   (`"shoreline unanchored - no tide reference"`), never via a direct
   `setStatus()`. Nothing publishes model state from inside `paint()`:
   `renderImage()` only records the anchor it used; composition and
   `setStatus()` happen in the `changed()` slot, outside paint
   (`MapItem::setStatus` → `Map::updateDisplay` → `emit dataChanged`).

### Phase C — record the decision

9. **`docs/decisions/0015-tide-anchored-topo-bathy-lut.md`.** Records the
   `BreakpointMap`-in-the-bake seam, the `shoreline_position` gate, `map_tide`
   as the anchor source with a pointer to uma ADR-0010 D5 for *why not*
   `chart_datum`, D1's deliberate departure from GeoZui4D's asymmetry, D3's
   single-scalar limitation, and D4's fallback contract. **Explicitly amends
   camp ADR-0008 Decision #2**: the LUT is no longer range-independent.
10. **Fix the now-stale comments in this PR**: `raster_gl_renderer.cpp:181-184`
    (`ensureLut()`'s "the LUT carries only the palette ramp") and
    `raster_gl_renderer.cpp:82-83` (the fragment-shader comment asserting the
    same), plus a cross-reference note in ADR-0008.

## Files to Change

| File | Change |
|------|--------|
| `src/camp_map/raster/pivoted_lut.h/.cpp` (new) | `bake_anchored_lut()` — `BreakpointMap` bake + byte-identical unanchored fallback |
| `src/camp_map/raster/raster_gl_renderer.h/.cpp` | `setShorelineAnchor()`; `ensureLut()` keyed on `(name, lo, hi, anchor)`; stale comments fixed |
| `src/camp_map/raster/colormap_range_dialog.h/.cpp` | Manual anchor field + persistence; `legend->setLut(bake_anchored_lut(...))`; anchor readout label |
| `src/camp_map/raster/sea_surface_reference.h/.cpp` (new) | ROS-free anchor holder + `changed()` signal |
| `src/camp_map/ros/sea_surface_tracker.h/.cpp` (new) | TF poller, frame auto-discovery, `#220` guards, 0.1 m threshold |
| `src/camp_map/raster/gggs_tile_layer.h/.cpp` | Anchor source; `changed()` slot invalidates cache; status part inside `updateStatus()` |
| `src/camp_map/raster/raster_layer.h/.cpp` | Same wiring; status composed outside `paint()` |
| `CMakeLists.txt` | New sources into `camp_map` / `camp_map_ros`; new gtests |
| `test/test_anchored_lut.cpp` (new) | Anchor lands at `shoreline_position`; anchor outside range / at a boundary / zero-width range; non-topo-bathy palette byte-identical to today |
| `test/test_sea_surface_reference.cpp` (new) | Threshold suppression, invalid→valid transition, fallback ordering Tide→Manual→None |
| `test/test_raster_gl_renderer.cpp` | Anchored vs unanchored LUT bytes; range change re-bakes |
| `test/test_range_persist.cpp` | Manual anchor round-trips through QSettings |
| `docs/decisions/0015-tide-anchored-topo-bathy-lut.md` (new) | ADR per step 9 |
| `docs/decisions/0008-adopt-marine-colormap-lut-bake.md` | Note that D#2's range-independence is amended by ADR-0015 |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Capture decisions, not just implementations | ADR-0015, and it amends ADR-0008 rather than quietly contradicting it |
| A change includes its consequences | The two stale code comments and ADR-0008's amended sentence land in this PR; the colorbar is fixed, not annotated |
| Test what breaks | Anchor placement + four degenerate cases; the no-tide fallback ordering; the unanchored path proven byte-identical |
| Only what's needed | No VDatum, no PROJ, no grids, no shader change, no `marine_colormap_widgets` change |
| Never document from assumptions | The `libcamp_map`-is-ROS-free constraint and the `setLut()` opportunity both came from reading source and both changed the design |
| Report the degraded state; never fail silently | D4: no tide anchors nothing and says so, rather than anchoring at a wrong 0.0 |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| camp ADR-0002 (ROS-free `libcamp_map`) | Yes — the crux | Anchor is pushed in via a ROS-free holder; all TF lives in `camp_map_ros`; GL tests stay hermetic |
| camp ADR-0007 (shared render path) | Yes | One seam (`ensureLut()`); all three consumers inherit it, gated on `shoreline_position` so non-topo-bathy layers pay nothing |
| camp ADR-0008 (LUT bake) | Yes — **amended** | Consequence #1 structurally enforced (never calls `bake_lut` on the anchored path); Decision #2's range-independence explicitly superseded |
| camp ADR-0009 (range dialog / colorbar) | Yes | Fixed, not deferred: `setLut()` makes the colorbar exact under an anchor, plus an anchor readout |
| uma ADR-0010 D5 (no `chart_datum` frame) | Yes | Obeyed — this is the whole reason for the replan |
| camp ADR-0014 / camp#195 (status composition) | Yes | Status goes through `updateStatus()`; nothing published from `paint()` |

## Consequences

| If we change... | Also update... | Included? |
|---|---|---|
| `ensureLut()` becomes range-dependent | ADR-0008 D#2, `ensureLut()` comment, shader comment | Yes — all three in this PR |
| A LUT re-bake now fires on every Auto-range update | Cost note in ADR-0015 (256-entry CPU bake, negligible; state it rather than let review find it) | Yes |
| `libcamp_map` gains an anchor concept | `SonarLiveCacheLayer` inherits it via the shared renderer; it has no anchor source and its palettes have no `shoreline_position`, so it is unaffected | Yes — stated, no wiring needed |
| A ROS→map push channel appears | ADR-0002's one-directional boundary claim stays true only because the holder is ROS-free — say so in ADR-0015 | Yes |
| Anchored LUT + `Linear` LUT filtering | The break smears over ~1 texel and quantizes to `(hi−lo)/255`; fine at nearshore spans | Yes — documented; `Nearest` deferred unless the break must be crisp |

## Documentation & Instruction Impact

- **Stale docs (must land in this PR)**: camp ADR-0008 Decision #2 (LUT
  range-independence — amended); `raster_gl_renderer.cpp:181-184`
  (`ensureLut()` comment); `raster_gl_renderer.cpp:82-83` (fragment-shader
  comment). The previous plan's "None" here was wrong.
- **Agent-instruction candidates**: one — `.agents/README.md` for camp should
  state the `libcamp_map`-is-ROS-free boundary as a *design constraint agents
  hit when adding data sources to raster layers*, since two consecutive plans
  for this issue assumed raster layers could reach the ROS node. Proposal only;
  operator decides.

## Schedule honesty (survey is Tuesday 2026-08-25; it is Saturday evening)

- **Phase A is achievable and verifiable by Monday.** It is ROS-free, testable
  headlessly on the dev host, and gives the operator a working manual anchor —
  the reported symptom fixed, with the operator entering one number per region
  (the issue's own "interim form").
- **Phase B is not safely verifiable before Tuesday.** It needs the ROC CAMP
  rebuild (already owed) *and* a live boat publishing `map_tide` to confirm the
  frame is visible over the udp_bridge link. Writing it is cheap; *verifying* it
  is not, and an unverified auto-anchor that silently picks a wrong frame is
  worse than a manual one the operator typed. **Recommendation: land Phase A
  first and treat Phase B as landing behind it — merged only if it can be
  exercised against a live boat before Tuesday, otherwise immediately after the
  survey.** Phase C's ADR covers both and lands with Phase A.
- If only one thing ships, it should be Phase A. That is a deliberate sequencing
  call, not a narrowing of scope.

## Open Questions

- [ ] **Is `map_tide` visible to CAMP on the ROC machine?** `/tf` is bridged
      (`bizzyboat.yaml:312`), but this has never been confirmed from CAMP's own
      TF buffer. Needs a `tf2_echo <prefix>/map <prefix>/map_tide` on the ROC
      before Phase B can be trusted. If it is not visible, Phase B is inert and
      Phase A's manual anchor is the whole feature.
- [ ] **Confirm the Phase A / Phase B split is the sequencing the operator
      wants** given Tuesday, or whether Phase B should be attempted regardless.
- [ ] **Frame auto-discovery policy**: is "unique frame ending in `map_tide`" an
      acceptable heuristic, or should CAMP require an explicit configured frame
      pair? Auto-discovery is friendlier before a survey; explicit is safer with
      two vehicles (`bizzy` and `izzy`) on one graph.
- [ ] **D1 (uniform shift, no GeoZui4D asymmetry)** — confirm acceptable. It
      makes displayed land elevation tide-dependent; the alternative opens a
      ~28 m discontinuity at the shoreline in our ellipsoidal frame.

## Estimated Scope

Two PRs, stacked. **PR1 = Phase A + Phase C** (anchoring mechanism, manual
anchor, colorbar fix, ADR, stale-comment fixes) — self-contained, headlessly
testable, shippable for Tuesday. **PR2 = Phase B** (tide-linked anchor, TF
tracker, status wiring) — branches from PR1, gated on the `map_tide` visibility
check above.
