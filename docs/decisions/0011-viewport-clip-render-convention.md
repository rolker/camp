# ADR-0011: Viewport-clip render convention at the raster seam

## Status

Accepted

Implements the **visible-region half** of CAMP issue
[#103](https://github.com/rolker/camp/issues/103). Extends
[ADR-0007](0007-raster-field-source-interface.md) (the `RasterFieldSource` /
`RasterGlRenderer` seam all three raster layers render through) with a calling
convention; the renderer's API is unchanged. The pyramid-LOD half of #103 is
explicitly deferred — it depends on store overview pyramids
(unh_marine_autonomy#188 and a shared pyramid library) that do not exist yet.

## Context

All three raster layers (`GggsTileLayer`, `RasterLayer`,
`SonarLiveCacheLayer`) shared the same `paint()` shape: map the **whole layer
extent** to on-screen pixels, clamp to `kMaxImageEdge` (4096), render every
tile into one offscreen FBO of that size, and draw it over `boundingRect()`.

For a store larger than 4096 on-screen pixels this is wrong at both ends of
the zoom range:

- **Zoomed in** (the field-observed failure, 2026-07-23): the viewport covers
  a small fraction of the extent, but the FBO still spans the whole extent at
  ≤4096 px — each visible pixel is backed by a fraction of an FBO pixel, so
  the display is blurry even though the tile pixels are full-resolution in
  memory.
- **Zoomed out** (the deferred LOD half): every tile is loaded and drawn no
  matter how few screen pixels it lands on.

## Decision

**paint() renders only the viewport-visible clip of the layer, sized to the
clip's on-screen pixels.** Concretely, via the shared helper
`raster/viewport_clip.h` (`deriveViewportClip()`):

1. **Viewport derivation**: from the **attached view** — the first view's
   `mapToScene(viewport)` mapped into the item and intersected with
   `boundingRect()`. The painter's state is NOT a reliable viewport signal
   and is only a secondary fallback: `QStyleOptionGraphicsItem::exposedRect`
   silently defaults to `boundingRect()` without `ItemUsesExtendedStyleOption`
   (set nowhere in camp), and `painter->clipBoundingRect()` is empty on live
   full-viewport repaints (field-verified — the blur came back). With no view
   and no painter clip, fall back to `boundingRect()` (pre-#103 whole-extent
   behavior for offscreen/QA renders). First view only — CAMP has one MapView.
2. **Scene conversion**: `clip_local` → `clip_scene` under the camp_map raster
   convention (item at the NW corner with `fromScale(1,-1)`, so local y
   increases southward while scene y increases northward).
3. **FBO sizing**: the clip's on-screen size, clamped to `kMaxImageEdge`
   (which now applies to the viewport, so it is rarely hit).
4. **Render**: each layer's clip-aware `renderImage(size, clip_bounds)` passes
   `clip_scene` — not the full `scene_bounds_` — as the `scene_bounds`
   argument of `RasterGlRenderer::renderToImage()`. The MVP crops to the clip;
   no renderer API change.
5. **Item filtering**: tiled sources filter their tiles to those whose
   Web-Mercator extent intersects the clip **before** the lazy texture upload,
   so offscreen tiles are neither uploaded nor drawn. `SonarLiveCacheLayer`
   filters both pools with the same predicate, preserving the
   overviews-first draw order (the ADR-0010 LOD fallback).
6. **Cache key**: `cached_image_` is keyed by FBO size **and** clip rect. A
   pan now re-renders each frame (pre-#103 it reused the whole-extent image
   via the world transform); with a viewport-sized FBO that re-render is
   cheap, and it is required for correctness once the image covers only the
   clip.

## Consequences

- Zoomed-in rendering of stores larger than 4096 on-screen pixels is crisp —
  the confirmed #103 field regression.
- Offscreen tiles cost no texture upload or draw, which is the natural
  precondition for camp#172's on-demand reload of evicted live tiles: the
  visible-region loop is where a reload request belongs. Not implemented here.
- The full-extent `renderImage(size)` overloads remain (delegating with
  `scene_bounds_`) so headless tests and QA renders are unchanged.
- The pyramid-LOD half of #103 (choose tile LEVEL by view scale) layers on
  top of this convention unchanged: LOD selection will change *which* items a
  source returns for the clip, not how the clip is derived or rendered.
