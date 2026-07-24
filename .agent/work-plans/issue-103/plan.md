# Plan: Visible-region render for GggsTileLayer (issue #103 — visible-region half)

## Issue

https://github.com/rolker/camp/issues/103

## Context

`GggsTileLayer`, `RasterLayer`, and `SonarLiveCacheLayer` all share the same
`paint()` pattern: compute the on-screen pixel size of the **whole layer extent**
(`mapRect(boundingRect())`), clamp to 4096 px, render all tiles into a single
FBO of that size, and draw the result over `boundingRect()`.  At high zoom this
is catastrophically wrong: the visible viewport covers a small fraction of the
total extent, so each visible pixel is backed by only a fraction of a pixel in
the 4096-px image → blurry.  Confirmed field-observed (Roland, 2026-07-23).

The fix is to size the FBO to the **viewport** (visible portion of the item),
pass that clip rect as `scene_bounds` to `renderToImage()`, and draw the result
over just the exposed portion.  The seam (`RasterFieldSource` / `RasterGlRenderer`,
camp ADR-0007) already supports this — `renderToImage()` takes a `scene_bounds`
QRectF that drives the MVP; we just need to pass the clip rect instead of the
full extent.  No change to `RasterGlRenderer`'s API is required.

The LOD/pyramid half of #103 is explicitly deferred — it depends on
unh_marine_autonomy#188 and a shared pyramid library.  The PR will be
"Part of #103", not "Closes #103".

## Approach

### Coordinate-system note

`scene_bounds_` in each layer is a QRectF in Web-Mercator metres where Qt
convention applies (top() < bottom()). Because the scene y-axis increases
northward, top() = southern edge (min y) and bottom() = northern edge (max y).
The item transform `fromScale(1, -1)` makes item-local y increase southward
(y=0 at the north edge).  The conversion from item-local `QRectF clip_local`
to scene `QRectF clip_scene` is:

```
clip_scene.setLeft(scene_bounds_.left() + clip_local.left());
clip_scene.setRight(scene_bounds_.left() + clip_local.right());
clip_scene.setBottom(scene_bounds_.bottom() - clip_local.top());   // north
clip_scene.setTop   (scene_bounds_.bottom() - clip_local.bottom());// south
```

### Step-by-step

1. **`GggsTileLayer::paint()`** — compute exposed clip rect and use it:
   - `clip_local = option->exposedRect.intersected(boundingRect())`; fall back
     to `boundingRect()` if empty (defensive).
   - Convert `clip_local` → `clip_scene` (scene Web-Mercator metres) using the
     formula above.
   - FBO size: `mapRect(clip_local)` instead of `mapRect(boundingRect())`; clamp
     to `kMaxImageEdge` (unchanged — the clamp now applies to the viewport, not
     the whole extent, so it is rarely hit).
   - Cache key: `size != cached_size_ || clip_scene != cached_clip_` — add
     `QRectF cached_clip_` member.
   - Render: `renderImage(size, clip_scene)` (new internal overload).
   - Draw: `painter->drawImage(clip_local, cached_image_)` (was `boundingRect()`).

2. **`GggsTileLayer::renderImage(size, clip_bounds)`** — clip-aware overload:
   - Filter tiles to those whose scene-space extent (pre-computed via
     `geoToMap(minLat, minLon)` / `geoToMap(maxLat, maxLon)`) intersects
     `clip_bounds`.  Build the `QList<RasterFieldItem>` from the filtered set only.
   - Call `renderer_.renderToImage(draw, clip_bounds, ...)` (pass `clip_bounds` as
     `scene_bounds`, not `scene_bounds_`).
   - Keep the existing public `renderImage(const QSize& size)` (used by headless
     tests) as a delegate to `renderImage(size, scene_bounds_)`.

3. **`GggsTileLayer`** — add member: `QRectF cached_clip_;` (after `cached_size_`).
   Invalidate `cached_image_` when `cached_clip_` changes (mirror the size check).

4. **`RasterLayer::paint()`** — same clip derivation, FBO sizing, cache key,
   drawImage.  No tile filtering (single-texture layer); `renderImage()` can
   accept the clip bounds and pass them straight to `renderToImage()`.
   Add `QRectF cached_clip_` member.

5. **`SonarLiveCacheLayer::paint()` + `renderImage()`** — same pattern; filter
   live-cache Entries by their scene-space extents intersecting `clip_scene`.
   Add `QRectF cached_clip_` member.

6. **New camp ADR-0011** — document the viewport-clip calling convention at the
   RasterGlRenderer seam: callers pass the visible sub-rect of `scene_bounds_`
   as the `scene_bounds` argument to `renderToImage()`, sized to the viewport,
   so the FBO covers only what the user sees.  Cross-references ADR-0007.

7. **Tests** — add a `test_gggs_tile_layer.cpp` test (or extend
   `test_raster_gl_renderer.cpp`) that:
   - Creates a two-tile GGGS store where tiles A and B are geographically adjacent.
   - Calls `renderImage(size, clip_A)` — only tile A should contribute pixels.
   - Asserts tile B's region is transparent/absent in the result.
   Keep existing tests passing; `renderImage(size)` (full-extent) is unchanged.

## Files to Change

| File | Change |
|------|--------|
| `src/camp_map/raster/gggs_tile_layer.h` | Add `QRectF cached_clip_`; add `QImage renderImage(const QSize&, const QRectF&)` private overload |
| `src/camp_map/raster/gggs_tile_layer.cpp` | Update `paint()` for clip-derived FBO; implement clip-filtered `renderImage(size, clip)` |
| `src/camp_map/raster/raster_layer.h` | Add `QRectF cached_clip_` |
| `src/camp_map/raster/raster_layer.cpp` | Update `paint()` for clip-derived FBO; update `renderImage()` to accept optional clip |
| `src/camp_map/ros/live_coverage/sonar_live_cache_layer.h` | Add `QRectF cached_clip_`; update `renderImage()` signature |
| `src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp` | Update `paint()` for clip-derived FBO; filter entries by clip |
| `docs/decisions/0011-viewport-clip-render-convention.md` | New camp ADR-0011 |
| `test/test_gggs_tile_layer.cpp` | New test for clip-filtered render (or add to test_raster_gl_renderer.cpp) |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Renders exactly what the operator sees; crisp vs. blurry is immediately visible |
| Only what's needed | No texture-cache eviction, no LOD machinery — just clip the FBO and filter items; those are the deferred LOD half |
| A change includes its consequences | All three seam callers updated in one PR; ADR-0011 captures the convention; existing tests preserved |
| Improve incrementally | Visible-region render alone closes the confirmed blur regression; good boundary |
| Test what breaks | New headless test verifying tile filtering by clip rect |
| Capture decisions, not just implementations | ADR-0011 records the viewport-clip calling convention at the seam |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| camp ADR-0007 (RasterFieldSource seam) | Yes | Fix at the seam: change what `scene_bounds` we pass to `renderToImage()`, no API change to the renderer itself |
| camp ADR-0010 (bounded eviction / overview pyramid) | Noted | No eviction machinery added; the clip filtering does not interfere with `SonarLiveCacheLayer`'s eviction logic (entries not in the clip are simply not rendered, not evicted) |
| workspace ADR-0001 (capture decisions) | Yes | New camp ADR-0011 records the viewport-clip convention |
| workspace ADR-0002 (worktree isolation) | Yes | Working in the existing issue-103 worktree |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `renderToImage()` calling convention (what we pass as `scene_bounds`) | All three layer `renderImage()` / `paint()` call sites | Yes — all three in this PR |
| Cache key for each layer | `cached_clip_` member added to each | Yes |
| camp ADR-0007 calling convention | New camp ADR-0011 cross-reference addendum | Yes |
| `GggsTileLayer::renderImage(size)` public API | Tests that call it headlessly | Preserved — public overload delegates to clip variant with full `scene_bounds_` |
| camp#172 (on-demand reload of evicted tiles) | This PR's visible-region paint loop is the natural trigger site for camp#172's reload | Not in scope — design leaves the paint loop open for camp#172 to add a reload call |

## Open Questions

- [ ] `option->exposedRect` under Qt's BSP update scheme may equal `boundingRect()`
  for large layers even when only part is visible — should we add a viewport-rect
  fallback via `painter->clipBoundingRect()`?  Likely no: the `mapRect(clip_local)`
  FBO sizing still corrects the resolution; the tile filter still works; even if we
  render the full extent it's at the right pixel density.

## Estimated Scope

Single PR ("Part of #103"). Touches three `.cpp`/`.h` pairs plus one new ADR and
one new test file. ~200–300 lines of implementation change.
