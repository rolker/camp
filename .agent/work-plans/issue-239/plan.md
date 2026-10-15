# Plan: Live coverage: depth colormap with uncertainty-driven opacity, so mapping uncovers the bathymetry

## Issue

https://github.com/rolker/camp/issues/239

## Context

`SonarLiveCacheLayer` renders ONE named band per tile: `textureFor()` uploads the
selected band (`band_name_`) as an R32F texture, `itemsIntersecting()` hands it to the
shared `RasterGlRenderer` as a `Scalar` `RasterFieldItem`, and the unified fragment
shader maps `(v - u_min)/(u_max - u_min)` through the 256x1 LUT with premultiplied
alpha fixed at 1 (NaN and finite-sentinel NoData are `discard`ed). Tiles already carry
`depth`, `uncertainty`, `backscatter` (`sonar_live_tile.h`), and `foldChild()` folds
every band independently (MEAN of finite samples) so both bands exist at every overview
level (ADR-0010 D3). What is missing is a way to combine two bands: depth supplies the
colour, uncertainty supplies the opacity, so a freshly pinged cell shows faintly and
firms up as CUBE uncertainty falls.

Constraints from the repo's ADRs:
- ADR-0007: the render path is shared with `GggsTileLayer` and `RasterLayer`; changes
  must be additive and leave their output byte-identical.
- ADR-0008: `bake_lut` TransferParams stay identity; gain/alpha-ramp must go through the
  shader, never into the LUT. So the opacity ramp is a shader computation.
- ADR-0010: `entryBytes()` and the eviction/reload paths own texture lifetime; a second
  texture per entry must be counted and freed with the first.
- ADR-0009: range UI lives in a context-menu dialog; this feature extends that menu.
- camp#145 / camp#104 (band semantics) are NOT prerequisites: the live layer's bands are
  named, so the opacity band is selected by name (default `"uncertainty"`).

## Approach

1. **Pure opacity mapping** — `src/camp_map/raster/opacity_ramp.h` (header-only,
   ROS/GL/Qt-free): `OpacityRamp` is either `Standard` (opaque at the survey standard's
   allowed uncertainty for the cell's own depth, `sqrt(a^2 + (b*d)^2)`, transparent at
   `multiple` times that; defaults IHO S-44 Order 1a `a=0.5 m`, `b=0.013`, multiple 3,
   as named constants) or `Manual` (absolute `[lo, hi]`). `opacityFromUncertainty(u,
   depth, ramp)` = 1 at/below the opaque threshold, 0 at/above the transparent one,
   linear between; a degenerate ramp is a hard step; non-finite `u` or depth gives 0.
   This is the single reference implementation the unit test pins and the GLSL mirrors.
2. **Renderer, additive** — `RasterFieldItem` gains `QOpenGLTexture* alpha_texture =
   nullptr`, `bool has_alpha_nodata`, `float alpha_nodata`. `renderToImage()` gains a
   trailing defaulted parameter `const OpacityRamp* opacity = nullptr` (existing call
   sites untouched). The shader gets `uniform sampler2D u_alpha_tex` (unit 2),
   `uniform int u_alpha_on` (set per item), `u_alpha_kind`, `u_alpha_a/_b/_mult` (Standard)
   and `u_alpha_lo/_hi` (Manual), plus `u_has_alpha_nodata/u_alpha_nodata`. With
   `u_alpha_on == 0` (every existing caller) `a == 1.0` and the output is bit-identical.
   With it on, a Scalar item's colour is looked up as today, the opaque/transparent
   thresholds come from the ramp (Standard: from the cell's own depth `v`), and the
   output is premultiplied `vec4(c.rgb * a, a)` (matches the existing
   `GL_ONE, GL_ONE_MINUS_SRC_ALPHA` blend).
   Depth NoData still discards first. The alpha texture uses Nearest filtering like the
   value texture. Items without an `alpha_texture` in an alpha-mode draw are drawn per
   the missing-band policy in step 3 (a per-item `u_alpha_on` toggle, not a global one).
3. **Missing-uncertainty policy (decided here, flagged as an open question)** — cell
   level: finite depth with NaN/NoData uncertainty is rendered fully transparent
   (`a = 0`: nothing is known about its quality, so it is not asserted). Tile level: a
   tile with no uncertainty band draws as plain opaque depth (today's behaviour) so a
   depth-only source does not blank, and the layer status shows
   `(opacity: no uncertainty band)`. Both policies are documented in the design note.
4. **Layer plumbing** (`sonar_live_cache_layer.{h,cpp}`):
   a. Add `Entry::alpha_texture` + reuse `texture_dirty` (any patch marks both dirty;
      cost is one extra 960x960 R32F upload only while the mode is on).
      `textureFor()` stays; add `alphaTextureFor(Entry&)` uploading `opacity_band_name_`.
   b. `itemsIntersecting()` fills `alpha_texture`/`alpha_nodata` when the mode is on.
      Placeholder (coarse overview) items use the SAME sub-rect `u0..v1` for both
      textures because parent and child are the same W x H (ADR-0010 D3 invariant).
   c. Texture lifetime: every `texture.reset()` site (six found by `grep texture.reset` in
      `sonar_live_cache_layer.cpp`: lines 301, 303, 694, 1055, 1236, 1263 — destructor,
      prune, eviction, reload; re-grep at implementation time) also resets
      `alpha_texture`; changing `opacity_band_name_` marks every entry (fine AND overview)
      dirty, like `setBandName()`;
      `entryBytes()` counts the alpha texture so the ADR-0010 budget stays honest.
   d. `setBandName()` currently marks only `tiles_` dirty; verify whether
      `overview_tiles_` textures go stale after a band switch (a possible existing bug).
      If confirmed, fix it in this PR (one line + test) since the new mode depends on it.
   e. State: `bool opacity_mode_ = false`, `std::string opacity_band_name_ =
      "uncertainty"`, `OpacityRamp opacity_ramp_`, `bool opacity_ramp_manual_ = false`.
      Setters (`setOpacityMode`, `setOpacityRamp`, `resetOpacityRamp`) invalidate
      `cached_image_`, `writeSettings()`, `update()`.
   f. **Default ramp (owner decision, supersedes the earlier open question)**: never
      derived from live data. The default is IHO S-44 Order 1a per cell depth, transparent
      at 3x (named constants in `opacity_ramp.h`); the dialog can edit `a`, `b` and the
      multiple or pin manual absolute thresholds. The design note and ADR state that the
      CUBE uncertainty layer is scaled to 1.96 sigma (cube_bathymetry#163). No
      dialog-on-enable is needed any more because the default is a real standard.
   g. **Colour band vs opacity band**: the mode colours by `band_name_` (the existing Band
      picker) and takes opacity from `opacity_band_name_`. They must differ: the menu entry
      is disabled (with a tooltip) when `band_name_ == opacity_band_name_`, and `readSettings()`
      restoring that combination restores with the mode off. Auto-range folding
      (`foldAutoRange`) continues to follow `band_name_` only, so the colour range stays the
      depth range.
5. **Persistence** — in `readSettings()/writeSettings()` under `settingsKey()`
   (`live:<ns>`, the layer's `layer_id`): `opacity_mode` (bool, default false),
   `opacity_band` (string, default `uncertainty`), `opacity_ramp_mode`
   (`standard`|`manual`), `opacity_a`, `opacity_b`, `opacity_multiple`, `opacity_lo`,
   `opacity_hi`. Absent keys restore today's behaviour;
   a manual ramp is honoured only if both bounds are present and finite (same rule as
   the camp#142 range restore).
6. **UI** — in the layer context menu: a checkable "Show uncertainty as opacity" (enabled
   only when the held tiles carry the opacity band), and "Opacity range..." opening a
   small modal dialog (Standard/Manual radio, spin boxes for a, b, multiple and lo, hi, Reset-to-default, live-applied, in the
   style of `colormap_range_dialog`; put it in `raster/opacity_range_dialog.{h,cpp}` so
   `GggsTileLayer` can reuse it later). The existing "Colormap range..." keeps governing
   the depth colour range. The Band submenu keeps choosing the colour band.
7. **Design note + ADR** — `docs/decisions/0017-uncertainty-driven-opacity.md` records
   the decision (shader-side opacity from a second texture; ADR-0008 consequence;
   policies in step 3; why not GggsTileLayer yet). `docs/uncertainty_opacity_design.md`
   is the perceptual design note: opacity ("value-suppressing"/value-by-alpha) over a
   depth colormap versus a bivariate colour scheme, the over-chart blending caveat (a
   faint cell shows the basemap through it), and references. Colin Ware's work
   (*Information Visualization: Perception for Design*) is the primary reference; every
   citation is verified against the source before it is written (no citing from memory),
   and anything unverifiable is left out or marked as such.
8. **Tests** — see Files to Change. GL tests follow the repo's skip-without-context
   pattern; the pure mapping, the persistence round-trip, and the bytes/lifetime
   accounting run headless.
9. **Manual verification (recorded in the PR, not claimable by unit tests)** — run the
   demo bag/sim stream, enable the mode, confirm a new cell appears faintly and firms up,
   confirm default-off layers are visually unchanged, and compare GGGS/chart layers
   before/after for identical output.

## Files to Change

| File | Change |
|------|--------|
| `src/camp_map/raster/opacity_ramp.h` | New: `OpacityRamp` (Standard/Manual) + `opacityFromUncertainty()` pure reference |
| `src/camp_map/raster/raster_field_source.h` | `RasterFieldItem`: optional `alpha_texture`, alpha NoData fields |
| `src/camp_map/raster/raster_gl_renderer.{h,cpp}` | Shader uniforms/sampler, premultiplied alpha path, defaulted `opacity` param |
| `src/camp_map/raster/opacity_range_dialog.{h,cpp}` | New: Standard/Manual opacity ramp dialog |
| `src/camp_map/ros/live_coverage/sonar_live_cache_layer.{h,cpp}` | Mode state, alpha texture upload/lifetime/bytes, items(), menu, persistence |
| `CMakeLists.txt` | Add new sources + tests |
| `test/test_opacity_ramp.cpp` | New: ramp endpoints, linear midpoint, swapped/equal bounds, NaN/NoData policy |
| `test/test_range_persist.cpp` | Add opacity-mode round-trip + default-off + corrupt-entry cases for the live layer |
| `test/test_raster_gl_renderer.cpp` + `test/fixtures/scalar_render_golden.png` | Golden captured from the renderer BEFORE the change (commit bea9d00) plus alpha-mode tests: premultiplied pixels, NaN/sentinel uncertainty, shader-vs-CPU sweep, no-texture/no-ramp draws opaque (all skip without GL) |
| `test/test_sonar_live_eviction.cpp` | Add: a band switch marks overview textures dirty too (the existing `setBandName()` gap, fixed here). The alpha-texture byte accounting is exercised only through the shared code path, not a dedicated layer-level GL test (deviation) |
| `test/test_sonar_live_tile_nodata.cpp` | Overview fold carries both bands; parent cell with depth but no uncertainty stays NoData |
| `docs/decisions/0017-uncertainty-driven-opacity.md` | New ADR |
| `docs/uncertainty_opacity_design.md` | New design note (Ware reference, verified citations) |
| `docs/camp_user_manual.md` | Document the menu entries and persistence |
| `.agents/README.md` | A paragraph (this repo has no parameter table) with the persisted keys and the Entry/alpha-texture pitfalls |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Opt-in, default off; ramp is operator-adjustable and visible in a dialog; the missing-band status is shown in the layer status, not hidden |
| Capture decisions, not just implementations | ADR-0017 + design note in the same PR |
| A change includes its consequences | User manual, `.agents/README.md` parameter table, ADR cross-references, persistence tests all in this PR |
| Only what's needed | Live layer only; no GGGS two-band mode, no band-semantics contract (#104), no colorbar redesign |
| Improve incrementally | Renderer extension is additive with a defaulted parameter; one PR, atomic commits per step |
| Test what breaks | Tests target the risky parts: alpha mapping, premultiplication, texture lifetime/budget, persistence defaults, overview fold |
| Workspace vs. project separation | All changes are inside the camp repo |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| camp ADR-0007 (RasterFieldSource) | Yes | Item contract extended additively; GGGS and chart output unchanged, asserted by a test |
| camp ADR-0008 (marine_colormap LUT bake) | Yes | Identity TransferParams kept; opacity computed in the shader, not baked |
| camp ADR-0009 (colorbar placement) | Yes | New dialog follows the context-menu placement |
| camp ADR-0010 (eviction / overview) | Yes | Second texture counted and freed; both bands present at all levels via `foldChild()`; fidelity statement (mean, not propagated uncertainty) written down |
| camp ADR-0006 (live cache persistence) | Yes | New keys under the layer's `settingsKey()`; disk cache already stores all bands, so nothing to change |
| camp ADR-0001 (threading) | Yes | Everything stays on the GUI thread; no new callbacks |
| Workspace ADRs | N/A | No workspace infrastructure touched |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Shared `RasterFieldItem` / shader | GggsTileLayer, RasterLayer output unchanged (test) | Yes |
| Per-entry GL textures | `entryBytes()`, every `texture.reset()` site | Yes |
| Persisted layer keys | `.agents/README.md` table, user manual, round-trip test | Yes |
| Overview fold semantics | Design note statement on folded uncertainty | Yes (documentation only; no fold change) |
| `setBandName()` dirty marking | Possible existing overview staleness | Yes, verify and fix if real |
| GggsTileLayer two-band mode | Same dialog/shader are reusable | No - follow-up |
| camp#145 quality-ramp default | Orthogonal; mention in design note | No - separate issue |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `docs/camp_user_manual.md` (new menu entries,
  persistence behaviour); `.agents/README.md` verified-parameter table (new keys);
  `docs/decisions/0008` and `0007` get a one-line cross-reference to ADR-0017 in their
  consequences (the ADRs themselves are not otherwise edited).
- **Agent-instruction candidates** (proposals only): "Opacity/alpha for a layer goes in
  the shader, never the LUT (ADR-0008)" may merit a line in `.agents/README.md`
  pitfalls; the operator decides.

## Open Questions

All answered by the owner (2026-10-14); recorded as decisions:

- Default ramp: IHO S-44 Order 1a per cell depth, opaque at the allowed uncertainty, transparent at 3x; a, b and the multiple are named constants; manual override in the dialog; CUBE uncertainty is 1.96 sigma (cube_bathymetry#163), stated in the ADR and design note.
- Missing uncertainty: cell level transparent, tile without the band draws plain opaque depth.
- Basemap: OpenStreetMap; wash-out judged at rehearsal.
- Band name: fixed `uncertainty` for this slice.
- Legend: not in this PR.
- Ware: the book is the anchor; no page/chapter citation was verifiable, so none is given and the note says so.
- GGGS layer: follow-up filed as rolker/camp#241, not implemented.

## Estimated Scope

Single PR, about 8 atomic commits (pure ramp + test; renderer + test; layer plumbing; persistence + test; dialog + menu; ADR + design note; user manual + README). Moderate size; the renderer change is the risky part and is reviewable on its own commit.

## Implementation Notes

- The shader uniforms are `u_alpha_lo/u_alpha_hi` (plus kind/a/b/mult) rather than the plan's original `u_u_opaque/u_u_transparent`, per the plan review.
- The "alpha mode off is byte-identical" test compares against a golden PNG captured from the renderer before the change (bea9d00), with a per-channel tolerance of 2 for driver rounding; `CAMP_UPDATE_GOLDEN=1` regenerates it.
- `Entry` is brace-initialised positionally in six places; the new members were appended after `last_access_seq` and those sites gained explicit initialisers (otherwise `-Wmissing-field-initializers`).
- The alpha texture has its own dirty flag: `textureFor()` clears `texture_dirty` before `alphaTextureFor()` runs, so one shared flag would leave the alpha texture stale after a patch.
- Confirmed the plan's step 4d suspicion: `setBandName()` marked only fine tiles dirty; overview textures kept the old band. Fixed via `markAllTexturesDirty()`.
