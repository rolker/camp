# ADR-0017: Uncertainty-driven opacity for the live coverage layer

## Status

Accepted

Implements CAMP issue [#239](https://github.com/rolker/camp/issues/239). Extends
[ADR-0007](0007-raster-field-source-interface.md) (the unified render path) and
[ADR-0008](0008-adopt-marine-colormap-lut-bake.md) (LUT bake, identity transfer);
constrained by [ADR-0010](0010-bounded-eviction-overview-pyramid.md) (resident
footprint and the overview pyramid) and placed in the UI per
[ADR-0009](0009-colormap-range-colorbar-placement.md). The perceptual rationale is in
[`../uncertainty_opacity_design.md`](../uncertainty_opacity_design.md).

> **Numbering.** This is **camp ADR-0017**, a *project* ADR in this repo's
> `docs/decisions/` series, independent of the workspace ADR numbering.

## Context

`SonarLiveCacheLayer` colours one named band per tile through one colormap LUT. The
tiles carry `depth`, `uncertainty` and `backscatter`, but depth and uncertainty could
not be shown together. The wanted display: a freshly pinged cell appears faintly, and
as repeat passes bring the CUBE uncertainty down the depth colour firms up, so mapping
visibly *uncovers* the bathymetry. It must be a real, opt-in CAMP feature with the
default rendering unchanged.

## Decision

1. **Colour from one band, opacity from another, in the fragment shader.** Colour is
   the selected band through the LUT exactly as before; each cell's opacity is
   computed in the shader from a second R32F texture holding the `uncertainty` band.
   `RasterFieldItem` gains an optional `alpha_texture`; `renderToImage()` gains a
   defaulted `const OpacityRamp*`. With no ramp, or an item without an alpha texture,
   the output is bit-identical to before (pinned by a golden image captured from the
   renderer *before* the change).
2. **Alpha is never baked into the LUT.** ADR-0008 requires identity `TransferParams`
   and a LUT that carries only the palette; the opacity ramp therefore lives in the
   shader. Output is premultiplied (`rgb * a, a`) to match the renderer's
   `GL_ONE, GL_ONE_MINUS_SRC_ALPHA` blend.
3. **The default ramp is tied to a survey standard, not an app-wide number.** A cell is
   fully opaque where its uncertainty is at or below the standard's allowed uncertainty
   *for that cell's own depth*, and fully transparent at three times that, linear
   between. The shipped standard is **IHO S-44 Order 1a** (`a = 0.5 m`, `b = 0.013`;
   allowed uncertainty `sqrt(a^2 + (b*d)^2)`). `a`, `b` and the multiple (3) are named
   constants in `raster/opacity_ramp.h` and editable in the dialog; the operator can
   instead pin manual absolute thresholds. The depth is read as a magnitude so a
   negative-down depth band works. The shader takes the cell's depth from the COLOUR
   band's value, so the Standard ramp is applied only while the colour band is
   `depth` (otherwise the layer status says to pick depth or use manual thresholds);
   a Manual ramp is depth-independent and works with any colour band. Non-finite
   (NaN or infinite) uncertainty or depth is transparent, in the shader and in the CPU
   reference alike.
4. **Confidence level.** The CUBE `uncertainty` layer is scaled to 1.96 sigma (a 95 %
   interval; see cube_bathymetry#163), which is also the confidence level of S-44's
   TVU limits, so the two are comparable as they stand. A boat whose CUBE is configured
   to a different scale would shift the effective ramp; that is not detected here.
5. **Missing-uncertainty policy.** A cell with finite depth and NaN/NoData uncertainty
   is fully transparent (nothing is known about its quality, so it is not asserted). A
   tile that has no uncertainty band at all draws as plain opaque depth, and the layer
   status says `(opacity: no uncertainty band)`, so a depth-only source does not blank.
   When only SOME resident tiles lack the band the status counts them
   (`N of M tiles lack uncertainty, drawn opaque`), because plain opaque depth would
   otherwise read as high confidence.
6. **Overview pyramid.** `foldChild()` already folds every band independently by MEAN
   over finite samples, so both bands exist at every level and a coarse placeholder
   samples both textures through the same sub-rect (parent and child are the same
   W x H, ADR-0010 D3). The folded uncertainty is a mean of child values, a display
   approximation and not a propagated uncertainty; a parent cell with a depth mean but
   no uncertainty sample stays uncertainty-NoData and therefore transparent.
7. **Resident footprint.** The alpha texture is counted by `entryBytes()` (so the
   ADR-0010 budget stays honest) and freed at every site that frees the value texture.
   It is released (and leaves the budget) when the mode is turned off or cannot apply.
   It has its own dirty flag, because `textureFor()` clears the value texture's flag
   before the alpha upload runs.
8. **Opt-in, persisted.** The mode is off by default. It persists under the layer's
   `settingsKey()` (`MapItem/live:<namespace>`): `opacity_mode`, `opacity_band`
   (default `uncertainty`), `opacity_ramp_kind` (`standard`|`manual`), `opacity_a`,
   `opacity_b`, `opacity_multiple`, `opacity_lo`, `opacity_hi`. A stored ramp with a
   garbled, non-finite or nonsensical number, or a Manual ramp missing a bound or with
   `hi <= lo`, falls back as a whole to the shipped standard (a garbled string would
   otherwise convert to 0 and hide every cell); a colour band equal to
   the opacity band restores with the mode off.
9. **Band naming.** The opacity band is the fixed name `uncertainty` for this slice
   (camp#104 band semantics are not a prerequisite); the setting is stored so it can
   become selectable later.

## Consequences

- One extra R32F upload per resident tile while the mode is on; none otherwise.
- Faint cells blend against whatever is beneath the layer (the OpenStreetMap base in
  the demo); wash-out on a light base is judged on the water, not in code.
- The GGGS tile layer and `RasterLayer` are untouched; the renderer extension is reusable
  by a follow-up that gives GGGS tiles the same mode.
- `setBandName()` previously left overview textures on the old band; fixed here because
  the new mode depends on both pools following the band selection.
- The alpha texture roughly doubles the per-tile texture cost while the mode is on. Eviction
  does not know which tiles the last render needed, so under a budget tight enough that the
  visible tiles alone exceed it, eviction can shed visible fine tiles to their overviews
  (the camp#172 reload path then brings them back on the next pan). The default budget
  leaves ample headroom; viewport-aware eviction is not part of this change.
- The ramp dialog is a modal window per ADR-0009; a modeless variant remains possible.
