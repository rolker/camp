# ADR-0015: Anchor the topo-bathy shoreline to a water level, not the range midpoint

## Status

Accepted

Implements CAMP issue [#181](https://github.com/rolker/camp/issues/181). Amends
**[ADR-0008](0008-adopt-marine-colormap-lut-bake.md) Decision #2** (see D8).
Builds on [ADR-0002](0002-web-mercator-scene-and-layer-model.md) (the `libcamp_map`
ROS-free boundary, which constrains where an anchor value may come from) and
[ADR-0009](0009-colormap-range-colorbar-placement.md) (the colorbar this makes
truthful).

> **Numbering.** This is **camp ADR-0015**, a project ADR in this repo's series.
> The workspace repo has its own independent ADR series; numbers collide by
> coincidence.

## Context

The reported symptom: an operator selected the palette that should pin at chart
datum, and it did not pin.

The cause is structural, not a bug in any one function. `RasterGlRenderer`'s
fragment shader normalizes **linearly** — `t = (v - u_min) / (u_max - u_min)` —
and samples a LUT baked from the palette alone. A topo-bathy ramp's land/sea
colour transition therefore lands wherever the **range midpoint** happens to
fall. On a survey box whose depths run −60 m to +10 m ellipsoidal, that puts the
"shoreline" colour at −25 m; the actual chart datum at the Isles of Shoals is
**−28.038 m** (geoid18 −26.587 + MLLW −1.451). The display is not merely
imprecise, it is *arbitrary*: pan to a different area, the range changes, and the
shoreline colour moves with it.

`marine_colormap` anticipated this — `PaletteDomain::shoreline_position` exists
for it, and `BreakpointMap` is documented with "chart datum at 0.0" as its worked
example — but the capability was CPU-only. A baked-LUT (GPU) consumer like camp
had no way to reach it. That gap is closed in the library by
[marine_colormap#23](https://github.com/rolker/marine_colormap/issues/23), which
adds `bake_lut(pal, params, n, const BreakpointMap&)` and
`bake_shoreline_anchored_lut(...)`.

This ADR records **camp's** decisions on top of that mechanism: where the anchor
value comes from, what happens when it is unavailable, and what the operator is
told.

## Decision

1. **The mechanism lives in `marine_colormap`, not in camp.** camp calls
   `bake_shoreline_anchored_lut()` / `has_shoreline()`. An earlier iteration of
   this work carried a private ~40-line `bake_anchored_lut()` in
   `camp_map/raster/`; it is deleted. The insight it embodied — fold the
   `BreakpointMap` into the *bake* so the shader's existing linear `t` still
   indexes the table, leaving the GLSL untouched — is general to every baked-LUT
   consumer (rviz, rqt, the sibling shading library), and duplicating it invites
   divergence between renderers that must agree.

2. **The LUT cache key widens to `(palette name, lo, hi, anchor)`.** An anchored
   table is a function of the range, so keying on the palette name alone would
   serve a stale table after a range change. That failure is silent — wrong
   colours, no crash, no log line — which is exactly the class of bug that
   reaches an operator unnoticed. `ensureLut()` deliberately **excludes `lo`/`hi`
   from the key while unanchored**, so an unanchored layer does not re-bake on
   every Auto-range tick.

3. **One source-agnostic seam: `ShorelineAnchor`.** Every source writes into a
   single ROS-free holder (value + source + `changed()`), and resolution walks a
   fixed precedence: **chart datum > platform tide > manual > none.** Declaration
   order *is* the precedence. The manual anchor is deliberately not a dialog
   special case — it is the *first source*, writing to the same holder the datum
   and tide sources use. Adding a source later is one setter, with no change to
   the resolution logic or the layers.

4. **The default reference is chart datum — because "the tide" is ambiguous with
   more than one platform.** `map_tide` is **per-platform** (`<ns>/map_tide`).
   With one vehicle on the graph that reads as unambiguous; with two, the display
   would have to decide *whose* tide it tracks. Chart datum has no such
   ambiguity: it is a property of the location, not of a vehicle. A display-wide
   vertical reference that silently depends on which boat happens to be up is a
   latent correctness bug, not an ergonomic wrinkle.

   **S-98 Ed. 2.0.0 Appendix D** independently agrees and is kept as
   corroboration, not as the justification: Water Level Adjustment is an
   operator-selectable function, off by default, with permanent on-screen
   indication while active. Two unrelated lines of reasoning reaching the same
   default is worth recording. A third: chart datum is a **surface** — correct
   across a wide display and static in time, so colours do not drift over a
   survey day.

   Switching to a platform's tide must remain **one click**, and must name the
   platform while active.

5. **Recorded policy is not the same as the live runtime default.** The policy in
   D4 is decided now. The runtime reality arrives in stages: the first PR ships
   the mode ordering with only *Manual* and *None* satisfiable, since no code
   pushes a chart-datum or tide value yet. Those modes are **present, correctly
   ordered, and honestly reported as unavailable** — not hidden, and not silently
   falling through as though they had been tried. **There is no runtime
   chart-datum mode until the datum source lands.** Stating this plainly is
   deliberate: an ADR that records an aspiration in the present tense is how a
   plan drifts from its implementation.

6. **An absent anchor is never `0.0`.** Unresolved means unresolved: the holder
   returns `std::nullopt` and the layer renders unanchored and says so. Anchor
   values are **ellipsoidal heights**, so substituting `0.0` would place the
   land/sea break roughly **28 m into deep water** at the Shoals — a
   plausible-looking display that is wrong in the direction that matters when the
   question is under-keel clearance among rocks. A fall-through to a lower-
   precedence source is **reported** via the active-source label, never applied
   silently (D4's S-98 "permanent indication", applied to provenance).

7. **The layers never pull; sources push.** `libcamp_map` is ROS-free by design
   (ADR-0002), and the raster layers derive from `map::Layer`, not
   `camp::ros::Layer` — they have no node. A TF or PROJ call inside them would
   breach the layering and make the headless GL tests host-dependent. So the tide
   tracker (which crosses the ROS boundary) and the chart-datum provider (which
   links `marine_vertical_datum` in the app layer) both **push** values into the
   holder. Nothing datum-related runs in the render path. The tide source is
   scoped to a **selected platform's** `platform_namespace`, taken from camp's
   existing `/marine/platforms` enumeration — not discovered by a
   "unique frame ending in `map_tide`" heuristic, which would guess wrong exactly
   when two vehicles are up.

8. **This amends ADR-0008 Decision #2.** That decision states the LUT "carries
   *only* the palette colour ramp", range-independent, with range-normalize left
   entirely to the shader. **That remains true on the unanchored path and is no
   longer true when an anchor is active**: the anchor is folded into the bake, so
   the table depends on the range. ADR-0008's underlying concern — that
   `TransferParams` must stay identity so the shader's range is not
   double-applied — is *preserved and in fact strengthened*: the anchored path
   routes through `Palette::sample()` with no `TransferParams` to get wrong, so
   double-application is structurally impossible there.

## Consequences

- **ADR-0008 Decision #2 must be read with D8 above.** A cross-reference note is
  added there; the two ADRs are not independently readable on this point.
- **Stale comments removed.** `raster_gl_renderer.cpp`'s `ensureLut()` and
  fragment-shader comments asserted the LUT carries only the palette ramp. They
  now state the range dependence and its silent-failure mode.
- **The colorbar becomes truthful.** With an anchor active the legend is fed the
  same baked table (`setLut()`), so the colour↔value mapping shown to the
  operator matches what is rendered. Before this, an anchored render would have
  been described by an unanchored legend — a worse failure than no legend.
- **Palette gate.** The anchor control is offered only for palettes declaring a
  `shoreline_position` (`oleron`, `hypsometric`). Anchoring a general-purpose
  ramp is a byte-identical no-op, so no existing layer's colours change.
- **A second vertical reference now exists in the UI.** Store values remain
  ellipsoidal heights and are still labelled as such (camp#180); the anchor is a
  *display* reference, not a reprojection of the data. The two must stay visibly
  distinct or the operator will read one as the other.
- **Grid provisioning becomes a dependency** for the chart-datum source: VDatum
  and geoid grids must be present on whichever machine runs CAMP. Verified on the
  dev host only; the operator station is unverified.
