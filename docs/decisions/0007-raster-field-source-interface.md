# ADR-0007: `RasterFieldSource` render abstraction + unified GL raster renderer

## Status

Accepted

Implements CAMP issue [#134](https://github.com/rolker/camp/issues/134) — the
generic raster render abstraction deferred from ADR-0006 (which delivered the
live tile cache, issue #121 Part B, and explicitly left "Part A — the generic
`RasterFieldSource` render abstraction" to this issue). Builds on
[ADR-0002](0002-web-mercator-scene-and-layer-model.md) (the Web-Mercator scene +
layer model — every raster layer stays a Web-Mercator scene object; the geo→mercator
warp moves *into* the shared renderer, not out of the scene model) and
[ADR-0006](0006-live-tile-cache-persistence.md) (the live tile cache, whose
persistence + opt-in-subscription contract this render migration leaves untouched).

## Context

Three raster layers each carried their own display path:

- `raster/gggs_tile_layer.cpp` — GDAL-backed GGGS tiles. Offscreen FBO + a GLSL
  120 fragment shader (single-band R32F value texture → colormap LUT, per-tile
  NoData discard), per-vertex geo→Web-Mercator tessellation, Nearest value-texture
  filter.
- `ros/live_coverage/sonar_live_cache_layer.cpp` — in-memory live tiles. A
  **verbatim copy** of that shader + tessellation, explicitly marked
  `// NOTE: shader duplicated from GggsTileLayer; will unify via RasterFieldSource
  in camp#134`. Only the data source differs (dequantized Float32 vs. GDAL read).
- `raster/raster_layer.cpp` — a single reprojected chart. A **separate**
  QPainter/mipmap path: CPU `ColorMap` shading for scalar bands, a CPU RGBA
  composite for palette/RGB, no GPU shader, no per-cell NoData discard on the GPU.

Two consequences followed from the duplication:

1. **The NaN NoData bug lived in two shaders.** GLSL `v == u_nodata` is *false*
   when `v` is NaN (NaN compares unequal to everything). Chart and backscatter
   stores use NaN as their NoData sentinel, so those cells rendered **opaque**.
   The CPU range fold already excluded NaN (`std::isfinite`), so the CPU range and
   the GPU discard disagreed. Fixing it in two places invites re-drift.

2. **Every render change had to be made three times** (or twice plus a divergent
   CPU path), and the two GL copies could silently fall out of sync.

## Decision

Extract a single, source-agnostic GL raster render path:

- **`raster/raster_field_source.h`** — `RasterFieldSource`, the abstract supplier
  of renderable raster items, plus the `RasterFieldItem` value type the renderer
  consumes:

  ```cpp
  struct RasterFieldItem {
    enum class Format { Scalar, Rgba };
    QOpenGLTexture* texture;        // R32F (Scalar) or pre-composited RGBA8 (Rgba)
    Format format;
    bool geographic;                // true: warp [west,east]x[south,north] (deg)
    double west, east, south, north;//  false: already Web-Mercator metres (a quad)
    bool has_nodata; float nodata;  // Scalar finite-sentinel discard
  };

  class RasterFieldSource {
    virtual QStringList bands() const = 0;
    virtual RasterBandMeta metadata(const QString& band) const = 0;
    virtual QList<RasterFieldItem> items() = 0;
    virtual QPair<float,float> dataRange() const = 0;
  };
  ```

- **`raster/raster_gl_renderer.{h,cpp}`** — `RasterGlRenderer`, which owns the
  per-GL-context state (the offscreen context + surface, the FBO, the **one**
  compiled program, and the colormap LUT texture) and the **unified fragment
  shader**. The geo→Web-Mercator vertex tessellation and the Nearest/Linear
  data-texture filter selection live here, once.

### The unified shader (where the NaN fix lands, once)

```glsl
if(u_mode == 0) {                                   // Scalar: R32F -> LUT
  float v = texture2D(u_tex, v_texcoord).r;
  if(v != v) discard;                               // NaN NoData (GLSL 1.20-portable)
  if(u_has_nodata != 0 && v == u_nodata) discard;   // finite sentinel (sidescan/0)
  ...colormap LUT...
} else {                                            // Rgba: sample directly, bypass LUT
  vec4 c = texture2D(u_tex, v_texcoord);
  if(c.a == 0.0) discard;                           // transparent NoData
  gl_FragColor = vec4(c.rgb * c.a, c.a);            // premultiplied (GL_ONE, 1-SRC_A)
}
```

`isnan()` is GLSL 1.30+; `v != v` is the portable 1.20 idiom and matches the
target `#version 120`.

### Scalar vs. RGBA (the format/mode branch)

`RasterFieldItem::Format` resolves issue #134's open question. Scalar bands
(GGGS, live, scalar charts) upload as R32F and shade through the colormap LUT.
Palette/RGB charts are **composited to RGBA8 on the CPU** (as RasterLayer does
today) and sampled directly — the LUT is bypassed and the NaN/finite discard does
not apply (transparency comes from the composited alpha). Per-band channel select
for colour files is **out of scope** (a future issue); RasterLayer continues to
composite on CPU.

### Ownership & threading contract

- **Textures are owned by the source** (`GggsTile`, the live `Entry`, `RasterLayer`),
  not the renderer. The `QOpenGLTexture*` in a `RasterFieldItem` is non-owning and
  valid only for the duration of the `renderToImage()` call it is passed to.
- **The renderer owns the GL context.** A source releases its own textures between
  a `renderer.makeCurrent()` / `renderer.doneCurrent()` pair (teardown, band
  switch). `items()` is called on the GUI/GL thread with that context current, so
  a source may lazily upload textures inside it.
- **No GL off the GUI thread.** Async loads (GDAL `RasterIO`, dequantize) produce
  CPU buffers off-thread; the texture upload happens on the GUI thread in the
  ready/handler slot, exactly as before.

### CPU range / GPU discard consistency

All three sources fold their auto-range over **finite, non-NoData** cells
(`std::isfinite` in `GggsTile::loadPixels`, `SonarLiveTile::refoldRange`, and
`RasterLayer::loadAndReprojectFile`). The unified shader now discards NaN (`v != v`)
*and* the finite sentinel, so the CPU range and the GPU discard agree on exactly
which cells are "data".

## Consequences

| If we change… | Also update… | Included? |
|---|---|---|
| The raster shader / discard logic | Only `raster_gl_renderer.cpp` — one place | Yes |
| `RasterLayer` from QPainter → GL | Mipmap pyramid + `QPixmap` chain removed; a scalar colormap change now re-bakes the LUT instead of re-warping the file | Yes |
| Both GL layers lose their own shader | The `// NOTE: shader duplicated` marker in `sonar_live_cache_layer.cpp` is removed | Yes |
| `marine_colormap` GPU-lib migration | Per-band colormap defaults | **No — deferred** |

- **ADR-0006 is untouched.** `SonarLiveCacheLayer`'s persistence (write-through /
  warm-load), opt-in tile-stream subscription, and GUI-thread reconciler invariant
  are all outside the render path. This migration swaps only `renderImage()`'s GL
  internals for the shared renderer; subscription, reconcile, prune, and
  write-through code is unchanged.
- **`marine_colormap` deferral.** camp keeps its internal `map::ColorMap` (baked to
  the LUT). Migrating to the `marine_colormap` GPU-shader library
  (`unh_marine_autonomy#175` / I4) — and the per-band colormap defaults it would
  bring — is separable and deferred (a follow-up issue).
- **`bands()` / `metadata()`** are part of the captured interface (they formalize
  the per-layer band/NoData access the context-menu band pickers already use); the
  renderer itself consumes only `items()` + `dataRange()`. Unifying the band-picker
  UI across the three layers behind this interface is left to a follow-up.

## ADR references (corrected)

The plan's first draft mislabeled the ADR table. For the record:

| ADR | Title | Relevance |
|---|---|---|
| camp ADR-0001 | *`TopicBridge` and the executor contract* | ROS-callback → GUI-thread marshalling that `SonarLiveCacheLayer` relies on; unchanged here |
| camp ADR-0002 | *Web-Mercator scene, two-model split, depth-as-layer, shared map library* | All three layers remain Web-Mercator scene objects |
| camp ADR-0006 | *Live tile cache — persistence + opt-in subscription* | Governs `SonarLiveCacheLayer`; its contract is untouched by this render migration |

(There is no camp "Adopt ADRs" ADR — that is a *workspace* ADR; the earlier table
wrongly attributed it to camp ADR-0001.)
