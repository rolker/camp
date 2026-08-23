# ADR-0008: Adopt `marine_colormap`; bake its palettes into the existing GL LUT

## Status

Accepted

Implements CAMP issue [#141](https://github.com/rolker/camp/issues/141) — replace
camp's internal `camp::map::ColorMap` (three hand-sampled ramps: Grayscale, Viridis,
Turbo) with the shared `marine_colormap` package (six palettes: grayscale, bronze,
thermal, viridis, turbo, quality). Builds on
[ADR-0007](0007-raster-field-source-interface.md) (the unified
`RasterFieldSource` + `RasterGlRenderer` render path this migration retargets) and
[ADR-0002](0002-web-mercator-scene-and-layer-model.md) (the Web-Mercator scene/layer
model the raster layers stay within).

> **Numbering.** This is **camp ADR-0008** — a *project* ADR in this repo's
> `docs/decisions/` series, where ADR-0001 is the TopicBridge/executor contract.
> It is **not** the workspace ADR-0008 ("ROS 2 conventions") referenced from the
> issue-review record; the two series are independent and the numbers collide by
> coincidence. (ADR-0007 had to make the same disambiguation.)

## Context

camp grew its own `ColorMap` (camp#63) so each scalar-field renderer would stop
reimplementing a value→colour ramp. It carried three ramps; viridis/turbo were
**sparse approximations** — viridis sampled at 7 stops, turbo at 8 — interpolated
piecewise-linearly. It served two render paths: the GPU LUT bake in
`RasterGlRenderer::ensureLut()` (GGGS tiles, live sonar coverage, single-chart
`RasterLayer`) and the CPU `colorNormalized()` sample in `GridMap`.

Meanwhile `marine_colormap` landed (jazzy) as the project-wide colormap library —
already adopted by rviz and rqt. It ships a **name-keyed, append-only registry** of
six palettes, where viridis/turbo are the **canonical 256-entry matplotlib/Google
tables** (not sparse stops), plus `bake_lut(palette, TransferParams, n)` for GPU LUT
upload and `Palette::sample(t)` for CPU lookup. Keeping camp's private ramps means a
second, drifting colormap definition and a viridis/turbo that visibly disagree with
every other tool in the workspace.

Two design questions had to be settled before adopting it:

1. **How to integrate on the GPU.** `marine_colormap` also offers a full GPU shader
   path (`marine_colormap::shader` + a `marine_colormap_response` uniform block) that
   applies range-normalize, gain, contrast, alpha-ramp and below/no-data sentinels in
   the fragment shader. camp#134 (ADR-0007) had *just* unified camp's own fragment
   shader, whose contract is load-bearing: NaN **and** finite-sentinel NoData
   `discard`, `Nearest` value-texture filtering (so the equality NoData test never
   blends across a sentinel boundary — the camp#122 halo fix), and per-band range
   normalize over the *true* data span. Swapping in marine_colormap's shader would put
   that contract at risk for no functional gain.

2. **Whether to accept a colour change.** Because the canonical viridis/turbo tables
   differ from camp's sparse approximations (e.g. viridis at t=0.5 is ≈(33,145,140)
   canonical vs ≈(34,168,132) in camp's 7-stop ramp — green off by ~23), adopting
   marine_colormap is **not** a no-op recolour: existing depth/backscatter renders
   shift slightly toward the published palettes.

## Decision

1. **Bake marine_colormap palettes into the existing 256×1 RGBA LUT; keep the
   camp#134 shader.** `RasterGlRenderer::ensureLut()` now calls
   `marine_colormap::bake_lut(find_palette(name), TransferParams{}, 256)` and copies
   the returned `Rgba8` entries into the same LUT texture the fragment shader already
   samples. The shader is **unchanged** — NaN/finite-NoData discard, `Nearest`
   filtering and per-band range normalize are all preserved. marine_colormap's GPU
   shader and `marine_colormap_response` block are **not** used.

2. **Bake with identity `TransferParams{}`.** `min=0, max=1, gain=1, contrast=1`,
   alpha-ramp off, no below/no-data colour. This is load-bearing: range-normalize
   stays the shader's job (per-band `u_min`/`u_max` over the true span), and the
   NoData sentinels stay the shader's `discard` branches. The LUT carries *only* the
   palette colour ramp — exactly what camp's `colorNormalized(i/255)` loop used to
   produce. (This is also why `bake_lut`'s own min/max/gain/contrast must stay
   identity: applying them here would double-apply the range the shader already owns.)

   > **Amended by [ADR-0015](0015-anchored-shoreline-colormap.md) D8 (camp#181).**
   > "The LUT carries *only* the palette colour ramp" — and with it the LUT's
   > range-independence — **holds on the unanchored path only**. When a shoreline
   > anchor is active the anchor is folded into the bake, so the table becomes a
   > function of the range and the cache key must widen to
   > `(palette name, lo, hi, anchor)`. The concern this decision exists to protect
   > — identity `TransferParams`, so the shader's range is not double-applied — is
   > preserved: the anchored path samples the palette directly and has no
   > `TransferParams` to get wrong.

3. **CPU path uses `Palette::sample(t)`.** `GridMap` replaces
   `colormap.colorNormalized(t)` with `find_palette(name)->sample(t)` quantized to a
   `QColor`. The `marine_colormap` contract guarantees the CPU sample and the baked
   LUT agree at the same normalized position, so GGGS/live/chart (GPU) and grid (CPU)
   render the same palette.

4. **The colormap selection is a `std::string` palette name.** `setColormap` takes a
   name; layers store `std::string colormap_name_` and persist the name via
   `settingsKey()` as before. An unknown name falls back to `"grayscale"`. Persisted
   names are read **case-insensitively** (lowercased, then validated against the
   registry) so any legacy capitalized `"Viridis"`/`"Turbo"` still resolves. The full
   six-palette registry is exposed in every colormap context menu via
   `marine_colormap::palette_names()`.

5. **Accept the canonical colour shift.** The viridis/turbo change is intended: it
   aligns camp with the published tables and with rviz/rqt. The parity test is
   reframed accordingly (see Consequences).

## Consequences

- **`bake_lut` TransferParams MUST stay identity.** Any future use of gain, contrast,
  alpha-ramp or below/no-data must go through the shader (or a deliberate shader
  rewrite), not by baking them into this LUT — otherwise the range the shader applies
  is double-counted. The colorbar/range UI (camp#142) must respect this split.
- **viridis/turbo renders shift.** Existing screenshots/goldens of camp depth or
  backscatter renders change. The colormap test is reframed to match: **grayscale**
  is asserted exact (it matches), while **viridis/turbo** are a **golden/
  characterization snapshot of the new canonical `bake_lut` output** — not a
  comparison against the retired sparse ramp. The old "locks in render equivalence"
  framing is dropped: there is no equivalence to lock, by design.
- **Dependency surface.** `marine_colormap` is a new `<depend>`; it links **PRIVATE**
  to `camp_map` (no marine_colormap type appears in any installed camp_map header —
  the colormap is a `std::string` name) and is added to `camp_map_ros`'s
  `ament_target_dependencies` for the `GridMap`/`SonarLiveCacheLayer` consumers.
- **camp#63 reconciliation.** The "camp-internal `ColorMap`" facility described in
  camp#63 is retired; this ADR is its successor record. `color_map.{h,cpp}` are
  deleted.
- **Registry is name-keyed and append-only.** Persisting the palette *name* (not a
  registry index) means new marine_colormap palettes can be added without remapping
  camp's saved selections.
