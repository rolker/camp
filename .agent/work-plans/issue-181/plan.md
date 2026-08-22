# Plan: Depth layers: land/water-distinguishing topo-bathy colormap pivoted at chart datum

## Issue

https://github.com/rolker/camp/issues/181

## Plan history

- `56ac199` — original plan: a colormap-private `chart_datum_service` doing a
  per-render VDatum/PROJ query on the GUI thread. **Rejected** by the operator.
- `b94a7d4` — first replan: anchor read solely from the `map_tide` frame.
  Mechanism correct, but it over-corrected — it treated datum knowledge in CAMP
  as forbidden, and it wrongly concluded the tide path could not be verified
  before Tuesday.
- **This version** — anchor is **source-agnostic** with a stated precedence;
  both a chart-datum source and a `map_tide` source are in scope; the tide path
  is verifiable **now, in simulation**.

## What was actually rejected, and what is actually wanted

These are not the same thing, and conflating them is what made the last two
plans wrong in opposite directions.

**Rejected**: a *colormap-private* datum service performing a per-render PROJ
query on the GUI thread. That is a runtime datum dependency buried inside the
render path — expensive, untestable, and duplicating a capability CAMP needs
elsewhere.

**Wanted** (operator, verbatim): *"camp needs to know about datums to provide
water depth re chart datum for the user, so using the datum to set the
colormap's 0 to chart datum's 0 should be possible."*

CAMP's own code already anticipates exactly this, in two places:

- `src/camp/autonomousvehicleproject.h:86-87` — *"store values are ellipsoidal
  heights, not chart-datum depths — distinct labels until the datum service
  (#288)."*
- `src/camp/projectview.cpp:213-215` — *"chart-datum depth. Query stores first
  ... and label the value distinctly so the operator can tell the two datums
  apart until the datum service (#288)."*

So CAMP already renders two differently-labelled depths (`Elev: … (ellipsoid)`
and `Depth: …`) and is **already waiting on a datum capability for the cursor
readout (camp#180), independent of this issue.** The colormap anchor is a second
consumer of that same capability, not a reason to invent a private one.

### Reconciling with the `marine_colormap` vision doc

The vision (`docs/vision.md`, `feature/issue-12` of `rolker/marine_colormap`,
PR mc#14, unmerged) is often quoted as *"there is no `chart_datum` runtime
frame, and that is a decision"*. Read in full, it does not forbid this:

- What uma ADR-0010 D5 excludes is a `chart_datum` **TF frame in the navigation
  loop**. The vision is explicit that this is a scoping, not a ban: *"D5 keeps
  models out of the navigation loop, not out of the system"*, and a modelled,
  wide-area vertical reference is *"entirely appropriate in camp and other
  operator tools"*. CAMP is an operator tool, not the navigation loop.
- *"`marine_colormap` should never know about tides, datums or drafts"* remains
  **untouched and honored**: the library receives a single anchor number. Which
  frame that number lives in is the consumer's business — which is precisely
  what this plan implements.
- *"Breakpoints are constants in the chosen frame"* is also honored. Anchoring
  at value `A` in the ellipsoidal frame is mathematically identical to shifting
  the field by `A` and breaking at 0.0 (see D2), with no texture rewrite.
- The vision's own S-98 Appendix D reading gives the default (see D3).

The one thing this plan will not do is put datum resolution inside the render
path. It goes in a CAMP-level capability, off the GUI thread, shared with the
readout.

## Ground truth (verified by reading source; file:line given so it is checkable)

- **`sea_surface_estimator` exists** — a node inside `mru_transform`
  (`platforms_ws/src/mru_transform/mru_transform/nodes/sea_surface_estimator.cpp`),
  not a missing package. `sea_surface_frame` defaults to `map_tide`.
- **The `map_tide` read pattern is settled** in `bathymetry_layer.cpp:653-672`:
  `lookupTransform(map_frame, map_tide_frame, TimePointZero).translation.z` is
  the sea-surface **ellipsoidal** height, because `map`'s z=0 is the WGS84
  ellipsoid — the same datum the stores use. Its guards are worth copying:
  refuse `map_frame == map_tide_frame` (the #220 degenerate self-lookup that
  read a whole survey as LETHAL), never accept the default 0.0 as a surface,
  re-render only past `tide_invalidate_threshold` (0.1 m, sized to clear ~±0.02 m
  estimator jitter).
- **The simulator provides tide AND the `map_tide` frame** — so the tide path is
  testable on this host today:
  - `simulation_ws/src/unh_marine_simulation/asv_sim/config/environment.yaml`
    carries a real harmonic model, *"constituents from NOAA Station 8423898
    (Fort Point, NH — Portsmouth Harbor)"*, with `speed_factor` documented as
    *"Set speed_factor > 1.0 to accelerate tide for testing (e.g., 3600.0
    compresses a 12-hour cycle into ~12 seconds)"*.
  - `asv_sim/asv_sim/asv_sim_node.py:40-41,62` publishes
    `~/environment/tide_level`.
  - `marine_simulation/launch/sim_robot_launch.py:338-358` brings up
    `mru_transform_node`, whose comment states it *"broadcasts
    `<ns>/base_link_north_up` and (via `sea_surface_estimator`) `<ns>/map_tide`"*.
  - **The same file gives a chart-datum ground truth**:
    `environment.tide.ellipsoid_to_mllw: -28.104` and
    `msl_above_mllw: 1.43`. That is within 7 cm of the measured −28.038 m at
    the Isles of Shoals, so the sim is a usable oracle for *both* anchor
    sources.
- **CORRECTION carried forward from the previous replan — the raster layers
  cannot call TF.** camp owns a `tf2_ros::Buffer`
  (`src/camp/ros/node_thread.h`, `camp_map/ros/layer.cpp`), but `RasterLayer`,
  `GggsTileLayer` and `RasterGlRenderer` live in **`libcamp_map`, which
  `CMakeLists.txt:239` declares "pure Qt/GDAL (ROS-free)"** with the boundary
  *"verified one-directional: the src/camp_map core has zero ROS includes"*
  (ADR-0002). They derive from `map::Layer`, **not** `camp::ros::Layer`, so
  they have no `node_`. A TF or PROJ call inside them would breach the layering
  and make the headless GL tests (`test_gggs_render.cpp`,
  `test_gggs_band_select.cpp`) host-dependent. **Every anchor source must push
  its value in; the layers never pull.** This is the structural reason the
  original per-render-query design could not have worked cleanly anyway.
- **There is no datum *service* to call, and this plan does not pretend
  otherwise.** A workspace-wide grep finds **no `.srv` mentioning datum at
  all**. What exists is:
  - `marine_vertical_datum` — a **ROS-free C++ library**
    (`core_ws/src/unh_marine_autonomy/marine_vertical_datum/include/…`)
    exposing `make_vdatum_query()` and
    `resolve_datum(...) -> std::optional<DatumResult>` with
    `DatumResult::chart_datum_z` (`datum_config.hpp:82-106`). `nullopt` is its
    designed "no in-datum coverage" signal, not a gap to work around.
  - `chart_datum_node` (`mru_transform`) — consumes that library and broadcasts
    a TF frame (`chart_datum_node.cpp:423,479`); it is a *boat-side* node, not
    something CAMP can query per point.
  - **uma#288 is not the service.** Its actual title is *"world/: canonical
    updater-managed home for geospatial support data (datum grids, user
    polygons, ENC/S-102 products) — ADR-0010 D3 amendment"* — it decides where
    grids **live**, not who resolves them. The "datum service (#288)" that
    camp's own comments await therefore **does not exist yet**; building it
    means CAMP linking `marine_vertical_datum` directly as a project-level
    capability. Phase C scopes that honestly rather than assuming a callable
    service.
- `marine_colormap::BreakpointMap` (`lookup.hpp:267`) clamps every degenerate
  input and never throws; a break outside the domain is documented as the
  *ordinary* case. `PaletteDomain::shoreline_position` (`palette.hpp:56`) is
  "metres, positive up", matching the stores' ellipsoidal up-positive heights,
  so anchor values are directly comparable with **no sign flip**. Only
  `oleron`/`hypsometric` carry it.
- `ColormapLegendWidget::setLut()` **already exists**
  (`colormap_legend_widget.hpp:65`) and overrides the linear palette sampling at
  `colormap_legend_widget.cpp:185`, so the colorbar is a real fix, not a
  documented gap.

## Design decisions

**D1 — Uniform shift, not GeoZui4D's asymmetry.** The vision quotes `gutm.cpp`
shifting only submerged terrain and leaving emergent terrain at its
datum-referenced elevation. **This PR does not implement that asymmetry, and the
reason is our frame.** GeoZui4D's land branch works because its land heights are
datum-referenced with 0 at the datum. Ours are **ellipsoidal**: land at the
waterline near Portsmouth is ≈ −28 m. Keeping `h` for land and `h − A` for water
would open a ~28 m discontinuity exactly at the shoreline — the opposite of what
this issue asks for. A uniform shift makes 0.0 the shoreline everywhere and
keeps the field continuous. Its cost: a hill's *displayed* height moves with the
anchor (irrelevant for a static chart-datum anchor; ±~1.5 m for a tide anchor).
Accepted and recorded in the ADR; the asymmetry becomes correct only once the
field carries orthometric land elevations, which is future work.

**D2 — The shift is expressed as an anchor, not as arithmetic on the data.** A
uniform shift is invisible to a linear ramp: `t = (v−lo)/(hi−lo)` is
shift-invariant, so subtracting `A` from every texel would change nothing. The
shift matters *only* because it moves the break. So anchor the palette's
`shoreline_position` at data value `A` in the unshifted ellipsoidal frame —
identical to shifting the field and breaking at 0.0, with no texture rewrite and
**no shader change**.

**D3 — Anchor sources, precedence, and the default.** The anchor is a plain
`std::optional<double>` from a pluggable source. Precedence when a layer's mode
is "automatic":

| # | Source | Answers | Nature |
|---|---|---|---|
| 1 | **Chart datum** (`marine_vertical_datum`, Phase C) | *"Is this land or water on the chart? What do charted soundings say?"* | Spatially varying surface; **static in time** |
| 2 | **`map_tide`** (Phase B) | *"How much water is over it right now?"* — the under-keel picture | Single measured scalar at the vessel; moves with the tide |
| 3 | **Manual** operator value (Phase A) | Operator's own number, per region | Always available, zero dependencies |
| 4 | **None** | — | Render unanchored + say so |

**Chart datum is the default; `map_tide` is an operator-selectable adjustment,
off by default, with permanent on-screen indication while active.** That is not
an arbitrary pick — it is the S-98 Edition 2.0.0 Appendix D model the vision
itself endorses: Water Level Adjustment is *"an operator-selectable function —
off by default, with permanent on-screen indication while active"*. It is also
the more defensible engineering choice: chart datum is a **surface**, correct
across a wide display, and stable so colours do not drift over a survey day,
whereas `map_tide` is one scalar (see D4). Both are offered as a selectable mode
because they answer genuinely different questions, and for a nearshore survey
among rocks the under-keel question is the live one.

**D4 — `map_tide` is the measured-local tier and is one number.** It is not a
surface. Over a display spanning Boston to the Isles of Shoals, one value is
applied everywhere, so it is wrong far from the boat by the spatial tide
gradient (order tens of cm over ~50 km, more up an estuary). Inside a nearshore
survey with the boat in view this is well within the useful band. The vision's
modelled global tier ("local costmap vs global costmap") is named as future
work, not built here. This asymmetry — chart datum spatial, tide scalar — is
itself an argument for chart datum as the default.

**D5 — Absent anchor never means 0.0.** Anchoring at 0.0 in the ellipsoidal
frame puts the land/water break ~28 m into deep water: worse than today. Falling
through the precedence to "none" renders **unanchored** (byte-identical to
today) and reports it in the layer status.

**D6 — Datum resolution never runs in the render path.** Phase C resolves off
the GUI thread, caches per coarse view region (datum surfaces vary slowly), and
pushes a value in. This is the specific thing that was rejected, and the design
must be structurally incapable of it — which the ADR-0002 boundary already
enforces, since `libcamp_map` cannot link PROJ.

## Approach — three phases, three stacked PRs

### Phase A — anchoring mechanism + manual anchor (PR1; ROS-free, no boat, no grids)

Fixes the reported symptom on its own and is fully verifiable headlessly.

1. **`src/camp_map/raster/anchored_lut.{h,cpp}` (new).** Pure; no GL, no I/O:
   ```cpp
   std::vector<marine_colormap::Rgba8> bake_anchored_lut(
     const marine_colormap::Palette& palette, float lo, float hi,
     std::optional<float> anchor_value, std::size_t n);
   ```
   With `anchor_value` **and** `palette.domain()->shoreline_position` both
   present: build `BreakpointMap(lo, hi, {{*anchor_value, *shoreline_position}})`,
   and for each `i` take `v = lo + (hi−lo)·i/(n−1)`, `t = map.normalize(v)`,
   `palette.sample(t)`. Otherwise return `bake_lut(palette, TransferParams{}, n)`
   verbatim. Calling `Palette::sample()` rather than `bake_lut` makes ADR-0008
   Consequence #1 (identity `TransferParams`) structurally unbreakable here.
2. **`RasterGlRenderer::setShorelineAnchor(std::optional<float>)`** + getter,
   mirroring `setColormap`'s dirty-flag pattern. `ensureLut()` now caches on
   `(name, lo, hi, anchor)` and re-bakes when any changes. Shader **untouched**.
3. **Manual per-layer anchor** — the interim form the issue itself proposes
   (*"a per-layer constant pivot parameter … user-set once per region"*). Anchor
   field in `ColormapRangeState` / the "Colormap range…" dialog, persisted beside
   `range_min`/`range_max` under the layer's `settingsKey()`. Offered only when
   the palette has a `shoreline_position`.
4. **Fix the colorbar (ADR-0009 must-fix, properly).** With an anchor active,
   `colormap_range_dialog.cpp` calls
   `legend->setLut(bake_anchored_lut(palette, lo, hi, anchor, 256))` instead of
   `legend->setPalette(index)`. `setLut()` already overrides the linear sampling
   at `colormap_legend_widget.cpp:185`, so the colour↔value mapping becomes
   *correct*, and `marine_colormap_widgets` needs no change. Add a readout label
   naming the active anchor **and its source** (S-98's "permanent indication").

### Phase B — `map_tide` anchor (PR2; crosses the ROS boundary; sim-verifiable now)

5. **`src/camp_map/raster/shoreline_anchor.{h,cpp}` (new, ROS-free).** A small
   `QObject` holding `std::optional<double> value` + a source enum + a
   `changed()` signal, owned by the map. **Zero ROS includes**, so ADR-0002's
   boundary and the headless GL tests stay intact and tests can set values
   directly. This is the single seam every source (tide, datum, manual) writes
   to — which is what makes the design source-agnostic rather than
   `map_tide`-shaped.
6. **`src/camp_map/ros/sea_surface_tracker.{h,cpp}` (new, `camp_map_ros`).**
   `QTimer`-driven, does `lookupTransform(map_frame, tide_frame,
   TimePointZero).translation.z`, writes the anchor holder. Copies
   `bathymetry_layer`'s guards: reject `map_frame == tide_frame`; never accept a
   default 0.0; publish only past a 0.1 m threshold. Frames from `QSettings`
   (`SeaSurface/map_frame`, `SeaSurface/tide_frame`), and when unset
   auto-discovered from the buffer's frame list (a unique frame ending in
   `map_tide` plus its sibling `map`) — the real frames are namespaced
   (`bizzy/map_tide`, `<ns>/map_tide` in sim) and no operator will hand-edit
   QSettings before a survey. Ambiguous or absent → no tide, reported honestly.
7. **Per-layer anchor mode + status.** Mode selector (Auto / Chart datum / Tide
   / Manual / None) with the D3 precedence under Auto. On `changed()` the layer
   records the value, drops `cached_image_`, requests a repaint. `GggsTileLayer`
   reports through **`updateStatus()`** — camp#195's single composer — as a new
   *part* naming the active source, never a direct `setStatus()`. Nothing
   publishes model state from inside `paint()`: `renderImage()` only records the
   anchor it used; composition and `setStatus()` happen in the `changed()` slot,
   outside paint (`MapItem::setStatus` → `Map::updateDisplay` → `dataChanged`).
8. **Sim verification** (this is what makes PR2 shippable pre-Tuesday): run
   `sim_robot_launch.py` with `environment.tide.speed_factor: 3600.0` so a
   12-hour cycle runs in ~12 s, and confirm the shoreline break tracks
   `<ns>/map_tide` live, oscillating about
   `ellipsoid_to_mllw + msl_above_mllw = −28.104 + 1.43 = −26.674 m` with the
   modelled amplitude. A tide cycle that took half a day to observe on the water
   takes seconds here.

### Phase C — chart-datum anchor (PR3; the capability CAMP wants anyway)

9. **`src/camp/chart_datum_provider.{h,cpp}` (new, in the `camp` app — not
   `libcamp_map`).** Links `marine_vertical_datum` directly (there is no service
   to call — see Ground truth). Builds the `VDatumQueryFn` **once** (its header
   is explicit that it must not be rebuilt per point and is not concurrency-safe
   across copies sharing a PROJ context), resolves on a **worker thread**, caches
   per coarse geographic cell, and pushes results into the Phase-B anchor holder.
   Grid paths from `QSettings`, defaulting to the uma#288 `world/datum/` layout —
   expanded via `QDir::homePath()`, never a literal `~` handed to PROJ.
   `resolve_datum()` returning `nullopt` is its designed "no coverage" signal and
   falls through the D3 precedence.
10. **Serve camp#180's readout from the same provider.** This is what makes
    Phase C a shared capability rather than a colormap-private one, and it
    retires the two `until the datum service (#288)` comments at
    `autonomousvehicleproject.h:86-87` and `projectview.cpp:213-215`. Whether the
    readout change lands in PR3 or a sibling PR is an open question — the
    provider must at minimum be *shaped* to serve it.

### Phase D — record the decisions (lands with PR1)

11. **`docs/decisions/0015-anchored-topo-bathy-lut.md`.** Records: the
    `BreakpointMap`-in-the-bake seam; the `shoreline_position` gate; the
    source-agnostic anchor with D3's precedence and the S-98-derived default;
    D1's deliberate departure from GeoZui4D; D4's scalar-vs-surface asymmetry;
    D5's fallback contract; D6's no-datum-in-the-render-path rule and the
    ADR-0002 boundary that enforces it. **Explicitly amends camp ADR-0008
    Decision #2**: the LUT is no longer range-independent.
12. **Fix the now-stale comments in this PR**: `raster_gl_renderer.cpp:181-184`
    (`ensureLut()`'s "the LUT carries only the palette ramp") and
    `raster_gl_renderer.cpp:82-83` (the fragment-shader comment asserting the
    same), plus a cross-reference note in ADR-0008.

## Files to Change

| File | Phase | Change |
|------|-------|--------|
| `src/camp_map/raster/anchored_lut.h/.cpp` (new) | A | `bake_anchored_lut()` — `BreakpointMap` bake + byte-identical unanchored fallback |
| `src/camp_map/raster/raster_gl_renderer.h/.cpp` | A | `setShorelineAnchor()`; `ensureLut()` keyed on `(name, lo, hi, anchor)`; stale comments fixed |
| `src/camp_map/raster/colormap_range_dialog.h/.cpp` | A | Manual anchor + persistence; `legend->setLut(...)`; anchor + source readout |
| `docs/decisions/0015-anchored-topo-bathy-lut.md` (new) | D | ADR per step 11 |
| `docs/decisions/0008-adopt-marine-colormap-lut-bake.md` | D | Note D#2's range-independence is amended by ADR-0015 |
| `src/camp_map/raster/shoreline_anchor.h/.cpp` (new) | B | ROS-free anchor holder: value + source + `changed()` |
| `src/camp_map/ros/sea_surface_tracker.h/.cpp` (new) | B | TF poller, frame auto-discovery, `#220` guards, 0.1 m threshold |
| `src/camp_map/raster/gggs_tile_layer.h/.cpp` | B | Anchor mode + precedence; `changed()` slot invalidates cache; status part inside `updateStatus()` |
| `src/camp_map/raster/raster_layer.h/.cpp` | B | Same wiring; status composed outside `paint()` |
| `src/camp/chart_datum_provider.h/.cpp` (new) | C | `marine_vertical_datum` off-thread, region-cached, pushes anchor |
| `src/camp/autonomousvehicleproject.{h,cpp}`, `src/camp/projectview.cpp` | C | Readout served from the provider; retire the two `until the datum service (#288)` comments |
| `package.xml` | C | `<depend>marine_vertical_datum</depend>` |
| `CMakeLists.txt` | A/B/C | New sources into `camp_map` / `camp_map_ros` / `CCOMAutonomousMissionPlanner`; new gtests |
| `test/test_anchored_lut.cpp` (new) | A | Anchor lands at `shoreline_position`; anchor outside range / at a boundary / zero-width range; non-topo-bathy palette byte-identical to today |
| `test/test_raster_gl_renderer.cpp` | A | Anchored vs unanchored LUT bytes; range change re-bakes |
| `test/test_range_persist.cpp` | A | Manual anchor round-trips through QSettings |
| `test/test_shoreline_anchor.cpp` (new) | B | D3 precedence ordering; D5 never-0.0; threshold suppression; invalid→valid transition |
| `test/test_chart_datum_provider.cpp` (new) | C | `nullopt` on missing grids; region cache hit/miss; never called from the GUI thread |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Capture decisions, not just implementations | ADR-0015 records all six decisions and amends ADR-0008 rather than quietly contradicting it |
| A change includes its consequences | Two stale code comments + ADR-0008's amended sentence land in PR1; the colorbar is fixed, not annotated; Phase C retires the two `#288` comments it makes obsolete |
| Test what breaks | Anchor placement + four degenerate cases; precedence ordering; never-0.0; the unanchored path proven byte-identical; sim gives an end-to-end oracle for both sources |
| Only what's needed | No shader change; no `marine_colormap_widgets` change; no invented ROS service; datum work deferred to its own PR |
| Never document from assumptions | Three findings changed the design: the ROS-free `libcamp_map` boundary, `setLut()` already existing, and there being no datum service at all |
| Report the degraded state; never fail silently | D5 + the status naming the active anchor source (also S-98's "permanent indication") |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| camp ADR-0002 (ROS-free `libcamp_map`) | Yes — the crux | Every source pushes through a ROS-free holder; TF in `camp_map_ros`, PROJ in the `camp` app; GL tests stay hermetic. The boundary is also what structurally prevents D6's rejected design |
| camp ADR-0007 (shared render path) | Yes | One seam (`ensureLut()`); all three consumers inherit it, gated on `shoreline_position` so non-topo-bathy layers pay nothing |
| camp ADR-0008 (LUT bake) | Yes — **amended** | Consequence #1 structurally enforced (anchored path never calls `bake_lut`); Decision #2's range-independence explicitly superseded |
| camp ADR-0009 (range dialog / colorbar) | Yes | Fixed, not deferred: `setLut()` makes the colorbar exact under an anchor, plus an anchor+source readout |
| camp ADR-0014 / camp#195 (status composition) | Yes | Status via `updateStatus()`; nothing published from `paint()` |
| uma ADR-0010 D5 (no `chart_datum` TF frame) | Yes | Honored: no `chart_datum` frame is created or consumed. A datum *value* computed in an operator tool is outside D5's navigation-loop scope — see the reconciliation above |

## Consequences

| If we change... | Also update... | Included? |
|---|---|---|
| `ensureLut()` becomes range-dependent | ADR-0008 D#2, `ensureLut()` comment, shader comment | Yes — all three in PR1 |
| LUT re-bakes on every Auto-range update | Cost note in ADR-0015 (256-entry CPU bake, negligible — state it rather than let review find it) | Yes |
| `libcamp_map` gains an anchor concept | `SonarLiveCacheLayer` inherits it via the shared renderer; no anchor source and no `shoreline_position` on its palettes, so it is unaffected | Yes — stated, no wiring |
| An external→map push channel appears | ADR-0002's one-directional claim stays true only because the holder is ROS-free and PROJ-free; say so in ADR-0015 | Yes |
| CAMP gains a datum capability | camp#180's cursor readout should consume it; the two `until the datum service (#288)` comments become stale | Yes — Phase C step 10 |
| Anchored LUT + `Linear` LUT filtering | Break smears over ~1 texel; anchor quantizes to `(hi−lo)/255`. Fine at nearshore spans | Yes — documented; `Nearest` deferred unless the break must be crisp |

## Documentation & Instruction Impact

- **Stale docs (must land with the PR that causes them)**: camp ADR-0008
  Decision #2 (LUT range-independence — amended, PR1);
  `raster_gl_renderer.cpp:181-184` and `:82-83` (PR1);
  `autonomousvehicleproject.h:86-87` and `projectview.cpp:213-215`
  (`until the datum service (#288)` — PR3). The original plan's "None" was wrong.
- **Agent-instruction candidates** (proposals only): two, both for camp's
  `.agents/README.md`. (a) The `libcamp_map`-is-ROS-free boundary as a design
  constraint agents hit when adding *any* data source to a raster layer — three
  successive plans for this issue assumed the layers could reach the ROS node.
  (b) That "the datum service (#288)" referenced in camp's comments does not
  exist and #288 is a storage-layout issue — the shorthand has already misled
  planning once.

## Schedule (survey Tuesday 2026-08-25; written Saturday evening)

**Retraction.** The previous version of this plan claimed the tide path *"is not
safely verifiable before Tuesday — it needs the already-owed ROC CAMP rebuild
and a live boat."* **That was wrong**, and the operator's correction is verified
above: `sim_robot_launch.py:338-358` brings up `mru_transform_node` publishing
`<ns>/map_tide`, and `environment.yaml` carries a real harmonic tide with a
documented `speed_factor` for accelerating it. A full tide cycle can be
exercised in ~12 seconds on this host, with `ellipsoid_to_mllw: -28.104` as an
oracle. No boat is required to verify Phase B.

- **PR1 (Phase A + D)** — achievable and verifiable by Monday. ROS-free, tested
  headlessly. Gives a working manual anchor: the reported symptom fixed, one
  number per region.
- **PR2 (Phase B)** — writable and **sim-verifiable before Tuesday**. Should be
  attempted for the survey, not deferred.
- **PR3 (Phase C)** — the chart-datum source. Larger (PROJ, grids, a threading
  model, a shared readout consumer) and **not** a Tuesday item. Deferring it
  costs nothing operationally: under D3 the layer falls through to the tide
  anchor, which is the more relevant question for a nearshore survey anyway.
- **Narrowed residual risk**: the one thing sim cannot settle is whether
  `map_tide` survives the bridge **to the ROC machine specifically**. `/tf` and
  `/tf_static` are bridged (`bizzyboat.yaml:312-313`) but this is unconfirmed on
  the operator box, and the ROC CAMP rebuild is separately owed. If it does not
  arrive there, PR1's manual anchor still works with zero dependencies — which
  is the reason PR1 leads.

## Open Questions

- [ ] **Is `map_tide` visible to CAMP's TF buffer on the ROC machine?** `/tf` is
      bridged (`bizzyboat.yaml:312`) but unconfirmed on the operator box. Needs
      one `tf2_echo <prefix>/map <prefix>/map_tide` there. Sim covers everything
      else about PR2, so this is the only field-side unknown.
- [ ] **Confirm D3's default**: chart datum as the default with `map_tide` as an
      operator-selectable adjustment (S-98 Appendix D's model). The alternative —
      tide as default, since under-keel clearance is the live question among
      rocks — is defensible and is a one-line change.
- [ ] **Phase C scope**: does PR3 also convert camp#180's cursor readout to the
      new provider (retiring both `#288` comments), or does the readout follow in
      a sibling PR? The provider must be *shaped* to serve it either way.
- [ ] **Frame auto-discovery policy**: "unique frame ending in `map_tide`" vs.
      requiring an explicit configured pair. Auto-discovery is friendlier before a
      survey; explicit is safer with two vehicles (`bizzy`, `izzy`) on one graph.
- [ ] **Confirm D1** — uniform shift rather than GeoZui4D's below-surface-only
      asymmetry. It makes displayed land elevation anchor-dependent; the
      alternative opens a ~28 m discontinuity at the shoreline in our frame.

## Estimated Scope

Three stacked PRs. **PR1 = Phase A + D** (mechanism, manual anchor, colorbar
fix, ADR, stale comments) — self-contained, headlessly testable, the Tuesday
deliverable. **PR2 = Phase B** (`map_tide` anchor, TF tracker, status wiring) —
sim-verified, targeted at Tuesday. **PR3 = Phase C** (chart-datum source via
`marine_vertical_datum`, shared with camp#180's readout) — post-survey.
