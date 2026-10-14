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

1. **Pure opacity mapping** — add `src/camp_map/raster/opacity_ramp.h` (header-only,
   ROS/GL/Qt-free): `struct OpacityRamp { float opaque; float transparent; }` and
   `float opacityFromUncertainty(float u, const OpacityRamp&)` = 1 at/below `opaque`, 0
   at/above `transparent`, linear between; `opaque >= transparent` is normalised (swap,
   and equal values give a hard step); NaN/NoData handled by the explicit policy in
   step 3. This is the single reference implementation the unit test pins and the GLSL
   mirrors line for line.
2. **Renderer, additive** — `RasterFieldItem` gains `QOpenGLTexture* alpha_texture =
   nullptr`, `bool has_alpha_nodata`, `float alpha_nodata`. `renderToImage()` gains a
   trailing defaulted parameter `const OpacityRamp* opacity = nullptr` (existing call
   sites untouched). The shader gets `uniform sampler2D u_alpha_tex` (unit 2),
   `uniform int u_alpha_mode`, `uniform float u_u_opaque, u_u_transparent`. With
   `u_alpha_mode == 0` (every existing caller) the code path is unchanged. With it on, a
   Scalar item's colour is looked up as today, then `a = clamp((u_u_transparent - u) /
   max(u_u_transparent - u_u_opaque, 1e-6), 0, 1)` and the output is premultiplied
   `vec4(c.rgb * a, a)` (matches the existing `GL_ONE, GL_ONE_MINUS_SRC_ALPHA` blend).
   Depth NoData still discards first. The alpha texture uses Nearest filtering like the
   value texture. Items without an `alpha_texture` in an alpha-mode draw are drawn per
   the missing-band policy in step 3 (a per-item `u_alpha_mode` toggle, not a global one).
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
   c. Texture lifetime: every `texture.reset()` site (destructor, prune, eviction,
      reload — 8 sites found by `grep texture.reset`) also resets `alpha_texture`;
      `entryBytes()` counts the alpha texture so the ADR-0010 budget stays honest.
   d. `setBandName()` currently marks only `tiles_` dirty; verify whether
      `overview_tiles_` textures go stale after a band switch (a possible existing bug).
      If confirmed, fix it in this PR (one line + test) since the new mode depends on it.
   e. State: `bool opacity_mode_ = false`, `std::string opacity_band_name_ =
      "uncertainty"`, `OpacityRamp opacity_ramp_`, `bool opacity_ramp_manual_ = false`.
      Setters (`setOpacityMode`, `setOpacityRamp`, `resetOpacityRamp`) invalidate
      `cached_image_`, `writeSettings()`, `update()`.
   f. **Default ramp**: when the mode is enabled and no persisted ramp exists, snapshot
      the held uncertainty band's auto-range `[min, max]` as the initial pinned ramp
      (`opaque = min`, `transparent = max`) and persist it, so it does not drift as
      repeat passes lower the uncertainty (a live-tracking ramp would defeat "firms up").
      If there is no uncertainty data yet, wait for the first tile. Final constants (or a
      survey-standard rule) are an open question for the owner.
5. **Persistence** — in `readSettings()/writeSettings()` under `settingsKey()`
   (`live:<ns>`, the layer's `layer_id`): `opacity_mode` (bool, default false),
   `opacity_band` (string, default `uncertainty`), `opacity_ramp_mode`
   (`auto`|`manual`), `opacity_lo`, `opacity_hi`. Absent keys restore today's behaviour;
   a manual ramp is honoured only if both bounds are present and finite (same rule as
   the camp#142 range restore).
6. **UI** — in the layer context menu: a checkable "Show uncertainty as opacity" (enabled
   only when the held tiles carry the opacity band), and "Opacity range..." opening a
   small modal dialog (two `QDoubleSpinBox`es + Reset-to-default, live-applied, in the
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
| `src/camp_map/raster/opacity_ramp.h` | New: `OpacityRamp` + `opacityFromUncertainty()` pure reference |
| `src/camp_map/raster/raster_field_source.h` | `RasterFieldItem`: optional `alpha_texture`, alpha NoData fields |
| `src/camp_map/raster/raster_gl_renderer.{h,cpp}` | Shader uniforms/sampler, premultiplied alpha path, defaulted `opacity` param |
| `src/camp_map/raster/opacity_range_dialog.{h,cpp}` | New: two-spinbox opacity range dialog |
| `src/camp_map/ros/live_coverage/sonar_live_cache_layer.{h,cpp}` | Mode state, alpha texture upload/lifetime/bytes, items(), menu, persistence |
| `CMakeLists.txt` | Add new sources + tests |
| `test/test_opacity_ramp.cpp` | New: ramp endpoints, linear midpoint, swapped/equal bounds, NaN/NoData policy |
| `test/test_range_persist.cpp` | Add opacity-mode round-trip + default-off + corrupt-entry cases for the live layer |
| `test/test_raster_gl_renderer.cpp` | Add: alpha mode renders expected premultiplied pixels (skips without GL); alpha mode off is byte-identical to before |
| `test/test_sonar_live_eviction.cpp` | Add: alpha texture counted in `entryBytes()` and freed on eviction |
| `test/test_sonar_live_tile_nodata.cpp` (or new) | Overview fold carries both bands; parent cell with depth but no uncertainty |
| `docs/decisions/0017-uncertainty-driven-opacity.md` | New ADR |
| `docs/uncertainty_opacity_design.md` | New design note (Ware reference, verified citations) |
| `docs/camp_user_manual.md` | Document the menu entries and persistence |
| `.agents/README.md` | Add the new persisted keys to the verified-parameter table |

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

- Default opacity ramp: snapshot of the uncertainty band's auto-range at enable time (as planned), a fixed survey-standard rule (which standard?), or a persisted app-wide default?
- Missing uncertainty at cell level renders transparent and a tile with no uncertainty band renders as plain opaque depth: is that the wanted policy?
- Which base should faint cells blend against for the demo (dark chart vs light)? Faint depth colour on a light basemap may wash out.
- Is a fixed band name `"uncertainty"` acceptable until camp#104 band semantics land, or should the opacity band be user-pickable in the first slice?
- Should the opacity range be shown on the colorbar legend in this PR, or is the separate dialog enough?
- Ware reference: which of his works should the design note anchor on (the book, or a specific uncertainty-display paper)? Citations will be verified before writing.
- Should the GGGS tile layer get the same mode in a follow-up issue (the issue says the live layer only)?

## Estimated Scope

Single PR, about 8 atomic commits (pure ramp + test; renderer + test; layer plumbing; persistence + test; dialog + menu; ADR + design note; user manual + README). Moderate size; the renderer change is the risky part and is reviewable on its own commit.
