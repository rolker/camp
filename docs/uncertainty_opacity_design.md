# Design note: showing uncertainty as opacity over a depth colormap

Companion to [ADR-0017](decisions/0017-uncertainty-driven-opacity.md) (camp#239). The
ADR records what was decided; this note records why opacity was chosen and what to
watch for. It is a design rationale, not a perception experiment: nothing here has
been user-tested in CAMP yet.

## The problem

A survey display wants to answer two questions about every cell at once: how deep is it,
and how much should I trust that number? In a live mapping display there is a third
reason to show both. A cell that has just been touched by one ping has a depth, but a
poor one. As repeat passes arrive, CUBE's uncertainty falls and the depth firms up. The
display should let the operator watch the bathymetry being *uncovered*.

## The choice: uncertainty drives opacity, depth drives colour

Colour keeps its existing job, a sequential colormap over depth read through the layer's
LUT. The second variable, uncertainty, is mapped to the opacity of the same cell: low
uncertainty is opaque, high uncertainty fades toward the base map.

Why opacity rather than a second colour channel or a separate uncertainty layer:

- The two quantities are not independent in meaning. A depth that is not yet trusted
  should look *less present*, and fading does exactly that. A separate uncertainty
  colour layer asks the operator to correlate two images by eye.
- A bivariate colour scheme (two hues mixed) would spend the colour channel twice and
  would compete with the depth colormap that operators already read.
- Opacity keeps the depth colormap intact where the data is good, so the display
  degrades gracefully: the best-known areas look exactly like the existing single-band
  display.

This follows the general line in visualisation research on uncertainty: the variable
that carries "how sure we are" is best encoded in a channel that reads as *more or less
there*, and the primary variable keeps the channel it already owns. The reference work
for this project is Colin Ware, *Information Visualization: Perception for Design*
(Morgan Kaufmann). That book is the framework for the colour and luminance reasoning
used here (a sequential colormap for a magnitude, using separate perceptual channels for
separate variables). The specific claims above about opacity as the uncertainty channel
are this design's reasoning; **they have not been checked against the book's text and
carry no page or chapter citation on purpose.** If a claim here is later verified (or
contradicted) against a specific passage, add the citation next to it.

## What sets the ramp: a survey standard, per cell depth

A fixed uncertainty threshold would be wrong in both shallow and deep water, because
what is acceptable depends on depth. The default therefore follows IHO S-44: a cell is
fully opaque where its uncertainty is at or below the allowed total vertical uncertainty
for *its own depth*, `sqrt(a^2 + (b*d)^2)`, and fully transparent at three times that.
The shipped values are Order 1a (`a = 0.5 m`, `b = 0.013`). Another order, or another
multiple, is a matter of typing different numbers into the dialog; a manual absolute
range is also available.

**Confidence level.** The CUBE `uncertainty` layer is the CUBE standard deviation scaled
by `stddev_to_confidence_interval_scale`, default 1.96, i.e. a 95 % interval
(cube_bathymetry#163). The S-44 limits are themselves 95 % figures, so comparing the two
directly is reasonable. If a boat is run with a different scale the comparison shifts by
that factor and the ramp should be adjusted; CAMP does not know the scale.

## Behaviour at the edges

- **Depth with no uncertainty sample** is fully transparent: if nothing is known about a
  cell's quality, the display does not assert its depth.
- **A tile with no uncertainty band at all** draws as plain opaque depth, so a
  depth-only source does not disappear; the layer status says so.
- **Coarse (overview) levels** carry both bands, each folded as the MEAN of the finer
  cells. The folded uncertainty is therefore an average, not a propagated uncertainty. At
  low zoom the opacity is a faithful *summary* of the area, not a statistically exact
  uncertainty; a single bad cell inside a good area is diluted.
- **Blending.** A faint cell shows whatever is beneath the layer. On a light base map a
  faint pale depth colour can wash out; the demo base is OpenStreetMap, and the
  legibility there is a rehearsal judgement, not something this code can guarantee. The
  colormap is unchanged, so the operator can pick a palette that holds up on the base in
  use.

## Not covered

- No colour bar / legend for the opacity ramp (a deliberate omission in this slice).
- The GGGS tile layer does not have the mode yet; the renderer extension is shared and
  reusable.
- No user study. Whether operators read fading as "uncertain" as intended is a thing to
  watch at the first real use.
