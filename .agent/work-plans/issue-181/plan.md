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
- `32812ce` — second replan: source-agnostic anchor, both sources in scope,
  tide path shown verifiable in simulation. Correct, but it left the default
  open and proposed the chart-datum source last.
- **This version** — operator has decided: **chart datum is the default**, on a
  multi-platform-ambiguity argument that outranks the S-98 one. The tide source
  becomes **platform-scoped**, and the PR order is **reversed** so the default
  source is not the last thing built.

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
- **The `map_tide` read pattern is settled** in **uma**
  `core_ws/src/unh_marine_autonomy/.../bathymetry_layer.cpp:653-672`
  (**a different repo from camp** — do not look for it in this tree):
  `lookupTransform(map_frame, map_tide_frame, TimePointZero).translation.z` is
  the sea-surface **ellipsoidal** height, because `map`'s z=0 is the WGS84
  ellipsoid — the same datum the stores use. Its guards are worth copying:
  refuse `map_frame == map_tide_frame` (the degenerate self-lookup of
  **uma#220** — a uma issue, not camp#220 — that
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
  `CMakeLists.txt:239` declares "pure Qt/GDAL (ROS-free)"**, with the boundary
  *"verified one-directional: the src/camp_map core has zero ROS includes"*
  stated separately at `CMakeLists.txt:332`
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
- **camp already enumerates platforms — do not invent a parallel selector.**
  `src/camp/platform_manager/platform_manager.cpp:25` subscribes to
  **`/marine/platforms`** (`marine_interfaces::msg::PlatformList`), keying
  platforms by name and building one `Platform` per entry. `Platform.msg`
  carries **`platform_namespace`** alongside `name`, and
  `platform.cpp:107-108` already reads it. camp also already has an
  **active-platform** concept: `PlatformManager::currentPlatform` →
  `AutonomousVehicleProject::updateActivePlatform` →
  `activePlatform()` (`mainwindow.cpp:105`, `autonomousvehicleproject.h:160`).
  So Roland's recollection is right — `/marine/platforms` support is partially
  present today, and it is enough to scope the tide anchor off. **This also
  dissolves the frame-auto-discovery open question**: `platform_namespace`
  composes directly into `<platform_namespace>/map_tide`, so no
  "unique frame ending in map_tide" heuristic is needed at all.
- `marine_colormap::BreakpointMap` (`lookup.hpp:267`) clamps every degenerate
  input and never throws; a break outside the domain is documented as the
  *ordinary* case. `PaletteDomain::shoreline_position` (`palette.hpp:53-56`) is a position **in
  normalized colour space**, NOT a data value — the earlier draft of this plan
  called it "metres, positive up", which is the doc comment on `natural_min`/
  `natural_max` (`palette.hpp:43-46`), not on `shoreline_position`. The
  distinction matters because it is what `bake_anchored_lut()`'s two arguments
  mean: `anchor_value` is a data value, `shoreline_position` is where it lands
  in the ramp. **The no-sign-flip claim is verified concretely, not assumed**:
  `hypsometric_domain()` (`topobathy_palettes.cpp:385-393`) sets
  `shoreline_position = 0.4f` with its own derivation written beside it,
  `(0 - -6000) / 15000` over `natural_min=-6000 .. natural_max=+9000` — i.e. the
  shoreline sits at **elevation zero on a positive-up metre scale**. That is the
  same convention as the GGGS store values (camp#180: "ellipsoidal up-positive
  elevation") and as `DatumResult::chart_datum_z` ("chart datum height rel.
  ellipsoid (m)"), so anchor values are directly comparable with no sign flip.
  `oleron_domain()` (`:377-383`) sets `0.5f` and deliberately carries no natural
  range. Only these two palettes carry a shoreline at all.
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

**D3 — Anchor sources: a fallback *order*, and separately a *default*.** These
are two different things and conflating them is how a fallback silently becomes
a policy. The anchor is a plain `std::optional<double>` written by a pluggable
source.

*Fallback order* (what happens when the active source yields nothing):

| # | Source | Answers | Nature |
|---|---|---|---|
| 1 | **Chart datum** (`marine_vertical_datum`, Phase C) | *"Is this land or water on the chart? What do charted soundings say?"* | Spatially varying surface; static in time; **a property of the location** |
| 2 | **A platform's `map_tide`** (Phase B) | *"How much water is over it right now?"* — the under-keel picture | Single measured scalar at one vessel; moves with the tide; **a property of a vehicle** |
| 3 | **Manual** operator value (Phase A) | Operator's own number, per region | Always available, zero dependencies |
| 4 | **None** | — | Render unanchored + say so |

*Default active source*: **chart datum.** Not merely first in the chain — the
mode a layer starts in. Switching to a platform's tide must be easy (a mode
selector on the layer, one click), but it is a deliberate operator act, shown
on screen while active.

**The decisive reason is multi-platform ambiguity, and it is not visible in the
code today.** `map_tide` is **per-platform** — the frame is `<ns>/map_tide`.
With one platform on the graph "the tide" reads as unambiguous; with two it is
not, and the display would have to decide *whose* tide it tracks. Chart datum
has no such ambiguity: it is a property of the location, not of a vehicle. A
display-wide reference that silently depends on which boat happens to be up is
a latent correctness bug, not just an ergonomic wrinkle. This argument is
independent of, and stronger than, the S-98 one.

The **S-98 Ed. 2.0.0 Appendix D** reading (which the vision doc itself endorses)
independently agrees and is worth keeping as corroboration: Water Level
Adjustment is *"an operator-selectable function — off by default, with permanent
on-screen indication while active"*. Two unrelated lines of reasoning landing on
the same default is a good sign. A third: chart datum is a **surface**, correct
across a wide display, and static, so colours do not drift over a survey day
(D4).

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

**D7 — The tide source is platform-scoped from day one.** The source is
*"platform X's `map_tide`"*, never *"the `map_tide` frame"*, **even while only
one platform exists**. A single-platform assumption baked in now is a rework
later, and D3's whole rationale is that the ambiguity is real. The selector key
is `Platform.msg::platform_namespace`, which composes directly into
`<platform_namespace>/map_tide` — so the platform identity and the frame name
are the same fact, not two things to keep in sync. The selector hangs off
camp's **existing** `/marine/platforms` enumeration and its existing
`activePlatform()` concept (see Ground truth); this plan adds **no** platform
enumeration, no view-locking, and no new platform UI — that is Roland's
not-yet-opened camp work and is explicitly out of scope. What is honest today:
default the tide source to the **active platform**, fall back to the sole
platform when exactly one is present, and when neither resolves, report
*"no platform selected"* on screen rather than silently picking one. If a
richer selector is wanted, it belongs in that unopened work, and this design
will consume it without change because the key is already
`platform_namespace`.

## Approach — three phases, stacked PRs (note the PR order is not the phase order)

### Phase A — anchoring mechanism + manual anchor (**PR1**; ROS-free, no boat, no grids)

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

5. **`src/camp_map/raster/shoreline_anchor.{h,cpp}` (new, ROS-free).** A small
   `QObject` holding `std::optional<double> value` + a source enum + a
   `changed()` signal, owned by the map. **Zero ROS includes**, so ADR-0002's
   boundary and the headless GL tests stay intact and tests can set values
   directly. This is the single seam every source (tide, datum, manual) writes
   to — which is what makes the design source-agnostic rather than
   `map_tide`-shaped.
6. **Per-layer anchor mode + status.** Mode selector (Chart datum [default] /
   Platform tide → *which platform* / Manual / None), with the D3 fallback order
   applied beneath whichever mode is active. On `changed()` the layer
   records the value, drops `cached_image_`, requests a repaint. `GggsTileLayer`
   reports through **`updateStatus()`** — camp#195's single composer — as a new
   *part* naming the active source **and, for tide, the platform**, never a
   direct `setStatus()`. Nothing
   publishes model state from inside `paint()`: `renderImage()` only records the
   anchor it used; composition and `setStatus()` happen in the `changed()` slot,
   outside paint (`MapItem::setStatus` → `Map::updateDisplay` → `dataChanged`).

**Why the seam is in PR1 and not with its first non-manual source** — the
`3a276f2` Plan Review caught that the PR2↔PR3 reorder had stranded it: the
holder (step 5) and the per-layer mode/status wiring (step 6) are what a source
*pushes into*, so leaving them in the last PR would have made PR2's
"chart datum is the default" inert on arrival — a default with nothing to
activate. They move here, where the manual anchor already needs them. That also
means the manual anchor is not a special case wired straight into the dialog: it
is the **first source**, writing to the same holder the datum and tide sources
will, which is what keeps the design source-agnostic rather than
retrofitted-around-manual.

### Phase B — platform-scoped `map_tide` anchor (**PR3**; crosses the ROS boundary; sim-verifiable)

7. **`src/camp_map/ros/sea_surface_tracker.{h,cpp}` (new, `camp_map_ros`).**
   `QTimer`-driven, does `lookupTransform(map_frame, tide_frame,
   TimePointZero).translation.z`, writes the anchor holder. Copies
   `bathymetry_layer`'s guards: reject `map_frame == tide_frame`; never accept a
   default 0.0; publish only past a 0.1 m threshold. **Frames are derived from
   the selected platform, not discovered** (D7): the tracker is handed a
   `platform_namespace` and builds `<ns>/map_tide` and `<ns>/map` from it. The
   previous plan's "unique frame ending in `map_tide`" heuristic is **dropped** —
   `/marine/platforms` already carries the authoritative namespace, and a
   heuristic that guesses wrong with two vehicles up is exactly the ambiguity
   D3 exists to remove. No platform selected → no tide, reported honestly.

8. **Sim verification** (this is what makes PR2 shippable pre-Tuesday): run
   `sim_robot_launch.py` with `environment.tide.speed_factor: 3600.0` so a
   12-hour cycle runs in ~12 s, and confirm the shoreline break tracks
   `<ns>/map_tide` live, oscillating about
   `ellipsoid_to_mllw + msl_above_mllw = −28.104 + 1.43 = −26.674 m` with the
   modelled amplitude. A tide cycle that took half a day to observe on the water
   takes seconds here.

### Phase C — chart-datum anchor (**PR2**; the default source, and the capability CAMP wants anyway)

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
    Phase C a shared capability rather than a colormap-private one. **The two
    `until the datum service (#288)` comments are no longer part of this work** —
    camp#203 / PR #204 corrected them on 2026-08-22, ahead of this plan, so the
    only remaining task here is serving the readout itself. Whether that lands in
    PR2 or a sibling PR is an open question — the provider must at minimum be
    *shaped* to serve it.

### Phase D — record the decisions (lands with **PR1**)

11. **`docs/decisions/0015-anchored-topo-bathy-lut.md`.** Records: the
    `BreakpointMap`-in-the-bake seam; the `shoreline_position` gate; the
    source-agnostic anchor with D3's fallback order and, separately, the
    chart-datum **default** — recording the multi-platform-ambiguity argument as
    the primary reason and S-98 Appendix D as corroboration; D7's
    platform-scoped tide source;
    D1's deliberate departure from GeoZui4D; D4's scalar-vs-surface asymmetry;
    D5's fallback contract; D6's no-datum-in-the-render-path rule and the
    ADR-0002 boundary that enforces it. **Explicitly amends camp ADR-0008
    Decision #2**: the LUT is no longer range-independent.

    **Separate the recorded *policy* from when it is the live *runtime*
    default** (Plan Review, `3a276f2`). The policy — chart datum is the
    reference a layer starts in — is decided and recorded by this ADR at PR1.
    The runtime reality arrives in stages: PR1 ships the mode ordering with only
    Manual and None satisfiable, PR2 makes chart datum an actually-resolvable
    source, PR3 adds platform tide. **There is therefore no runtime chart-datum
    mode between PR1 and PR3**, and the ADR must say so plainly rather than
    implying the default is live the moment it is written down. An ADR that
    records an aspiration in the present tense is how a plan drifts from its
    implementation.
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
| `src/camp_map/raster/shoreline_anchor.h/.cpp` (new) | A | ROS-free anchor holder: value + source + `changed()` |
| `src/camp_map/ros/sea_surface_tracker.h/.cpp` (new) | B | TF poller scoped to a `platform_namespace`, `#220` guards, 0.1 m threshold |
| `src/camp_map/raster/gggs_tile_layer.h/.cpp` | A | Anchor mode + fallback order; `changed()` slot invalidates cache; status part (source + platform) inside `updateStatus()` |
| `src/camp_map/raster/raster_layer.h/.cpp` | A | Same wiring; status composed outside `paint()` |
| `src/camp/chart_datum_provider.h/.cpp` (new) | C | `marine_vertical_datum` off-thread, region-cached, pushes anchor (the DEFAULT source) |
| `src/camp/autonomousvehicleproject.{h,cpp}`, `src/camp/projectview.cpp` | C | Readout served from the provider (the `#288` comments were already corrected by camp#203 / PR #204) |
| `package.xml` | C | `<depend>marine_vertical_datum</depend>` |
| `CMakeLists.txt` | A/B/C | New sources into `camp_map` / `camp_map_ros` / `CCOMAutonomousMissionPlanner`; new gtests |
| `test/test_anchored_lut.cpp` (new) | A | Anchor lands at `shoreline_position`; anchor outside range / at a boundary / zero-width range; non-topo-bathy palette byte-identical to today |
| `test/test_raster_gl_renderer.cpp` | A | Anchored vs unanchored LUT bytes; range change re-bakes |
| `test/test_range_persist.cpp` | A | Manual anchor round-trips through QSettings |
| `test/test_shoreline_anchor.cpp` (new) | A | D3 fallback ordering AND that the default active source is chart datum; D5 never-0.0; threshold suppression; invalid→valid transition |
| `test/test_chart_datum_provider.cpp` (new) | C | `nullopt` on missing grids; region cache hit/miss; never called from the GUI thread |
| `src/camp/platform_manager/platform_manager.h` | B | Expose the selected platform's `platform_namespace` to the tracker (read-only accessor; no new enumeration) |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Capture decisions, not just implementations | ADR-0015 records all six decisions and amends ADR-0008 rather than quietly contradicting it |
| A change includes its consequences | Two stale code comments + ADR-0008's amended sentence land in PR1; the colorbar is fixed, not annotated; Phase C retires the two `#288` comments it makes obsolete |
| Test what breaks | Anchor placement + four degenerate cases; precedence ordering; never-0.0; the unanchored path proven byte-identical; sim gives an end-to-end oracle for both sources |
| Only what's needed | No shader change; no `marine_colormap_widgets` change; no invented ROS service; no new platform enumeration or view-locking — the existing `/marine/platforms` model is consumed as-is |
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
| camp platform model (`/marine/platforms`, `activePlatform()`) | Yes | The tide source hangs off the existing enumeration and active-platform concept; no parallel selector, no view-locking, no new platform UI (Roland's unopened work) |
| uma ADR-0010 D5 (no `chart_datum` TF frame) | Yes | Honored: no `chart_datum` frame is created or consumed. A datum *value* computed in an operator tool is outside D5's navigation-loop scope — see the reconciliation above |

## Consequences

| If we change... | Also update... | Included? |
|---|---|---|
| `ensureLut()` becomes range-dependent | ADR-0008 D#2, `ensureLut()` comment, shader comment | Yes — all three in PR1 |
| LUT re-bakes on every Auto-range update | Cost note in ADR-0015 (256-entry CPU bake, negligible — state it rather than let review find it) | Yes |
| `libcamp_map` gains an anchor concept | `SonarLiveCacheLayer` inherits it via the shared renderer; no anchor source and no `shoreline_position` on its palettes, so it is unaffected | Yes — stated, no wiring |
| An external→map push channel appears | ADR-0002's one-directional claim stays true only because the holder is ROS-free and PROJ-free; say so in ADR-0015 | Yes |
| CAMP gains a datum capability | camp#180's cursor readout should consume it; the two `until the datum service (#288)` comments become stale | Yes — Phase C step 10 |
| The tide source becomes platform-scoped | camp's `PlatformManager` needs a read-only accessor for the selected platform's namespace; a richer selector belongs to Roland's unopened `/marine/platforms` work and this design consumes it unchanged | Yes — stated, minimal touch |
| Chart datum becomes the DEFAULT source | Grid provisioning on the machine that runs CAMP is back in scope — it was moot only while the datum source was out of scope | Yes — open question, gates PR2 |
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

**Re-sequencing, stated plainly rather than shuffled silently.** With chart
datum as the *default active* source (D3), building it last would have shipped a
default that does not exist — every layer would fall through to tide, making the
"default" fictional and the fallback the de-facto policy. So **PR2 and PR3 swap:
the chart-datum source comes before the tide source.** The coordinator's
suspicion is correct.

That reordering does **not** change what ships for Tuesday, and the reason is
worth stating because it is the load-bearing part: **PR1's manual anchor already
delivers the default's semantics.** For a single-region survey the number the
operator types *is* the chart datum (−28.038 m at the Isles of Shoals, and the
sim's own `ellipsoid_to_mllw: -28.104` agrees to 7 cm). PR1 is chart-datum
anchoring done by hand; PR2 is the same thing done automatically and spatially.
So the default is honoured from PR1 onward, and neither PR2 nor PR3 is required
for Tuesday.

- **PR1 (Phase A + D)** — achievable and verifiable by Monday. ROS-free, tested
  headlessly, zero dependencies. **This is the Tuesday deliverable**: the
  reported symptom fixed, one number per region, and that number is the chart
  datum.
- **PR2 (Phase C — chart-datum source, the default)** — post-survey. Larger
  (PROJ, grids, a threading model, a shared readout consumer) and it re-opens a
  provisioning question that was moot while the datum source was out of scope:
  the grids must exist on the machine that runs CAMP. Correctness here is worth
  more than speed; rushing a spatially-varying vertical reference before a
  survey among rocks is the wrong trade.
- **PR3 (Phase B — platform-scoped tide)** — post-survey, and **sim-verifiable
  in an evening** whenever it is picked up: `speed_factor: 3600.0` runs a
  12-hour cycle in ~12 s against `mru_transform_node`'s `<ns>/map_tide`. If
  circumstances change and exactly one of PR2/PR3 can land before Tuesday, **PR3
  is the one that fits** — it needs no grids and no provisioning, only the sim.
  That is a contingency, not the recommendation.
- **Narrowed residual risk**: the one thing sim cannot settle is whether
  `map_tide` survives the bridge **to the ROC machine specifically**. `/tf` and
  `/tf_static` are bridged (`bizzyboat.yaml:312-313`) but this is unconfirmed on
  the operator box, and the ROC CAMP rebuild is separately owed. If it does not
  arrive there, PR1's manual anchor still works with zero dependencies — which
  is the reason PR1 leads regardless of how PR2/PR3 are ordered.

## Open Questions

- [ ] **Are the VDatum/geoid grids present on the machine that runs CAMP?**
      Back in scope now that chart datum is the default (it was moot while the
      datum source was out of scope). Gates PR2. The `world/datum/{geoid,vdatum}`
      layout is confirmed on this dev host only.
- [ ] **Is `map_tide` visible to CAMP's TF buffer on the ROC machine?** `/tf` is
      bridged (`bizzyboat.yaml:312`) but unconfirmed on the operator box. Needs
      one `tf2_echo <ns>/map <ns>/map_tide` there. Sim covers everything else
      about PR3, so this is the only field-side unknown for the tide path.
- [ ] **Does the tide anchor's platform selector belong here or in Roland's
      unopened `/marine/platforms` work?** This plan scopes it to "the active
      platform, else the sole platform, else say none" using the existing
      enumeration and a read-only namespace accessor. If a first-class selector
      (or view-locking) is wanted, it belongs in that work and this design
      consumes it unchanged — the key is already `platform_namespace`.
- [ ] **Phase C scope**: does PR2 also convert camp#180's cursor readout to the
      new provider (retiring both `until the datum service (#288)` comments), or
      does the readout follow in a sibling PR? The provider must be *shaped* to
      serve it either way.
- [ ] **Confirm D1** — uniform shift rather than GeoZui4D's below-surface-only
      asymmetry. It makes displayed land elevation anchor-dependent; the
      alternative opens a ~28 m discontinuity at the shoreline in our frame.

**Resolved since the last version**: the anchor default (chart datum, on D3's
multi-platform argument) and frame auto-discovery (dissolved — `/marine/platforms`
carries `platform_namespace`, so no heuristic is needed).

## Estimated Scope

Three stacked PRs, **in this order**: **PR1 = Phase A + D** (mechanism, manual
anchor, colorbar fix, ADR, stale comments) — self-contained, headlessly
testable, the Tuesday deliverable and already chart-datum-anchoring by hand.
**PR2 = Phase C** (chart-datum source via `marine_vertical_datum`, the default
active source, shared with camp#180's readout) — brought forward from last in
the previous plan, because a default built last is not a default. **PR3 = Phase
B** (platform-scoped `map_tide` anchor, TF tracker, status wiring) — sim-verified
whenever picked up.
