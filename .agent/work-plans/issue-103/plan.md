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
   - `clip_local = painter->clipBoundingRect().intersected(boundingRect())`;
     fall back to `boundingRect()` if empty (defensive).  NOT
     `option->exposedRect`: that defaults to `boundingRect()` unless
     `QGraphicsItem::ItemUsesExtendedStyleOption` is set (set nowhere in camp),
     which would make the whole fix a silent no-op.
     `painter->clipBoundingRect()` reflects the view's actual exposed region
     with no flag dependency (plan-review must-fix 1).
   - Convert `clip_local` → `clip_scene` (scene Web-Mercator metres) using the
     formula above.
   - FBO size: `mapRect(clip_local)` instead of `mapRect(boundingRect())`; clamp
     to `kMaxImageEdge` (unchanged — the clamp now applies to the viewport, not
     the whole extent, so it is rarely hit).
   - Cache key: `size != cached_size_ || clip_scene != cached_clip_` — add
     `QRectF cached_clip_` member.  **Behavior change (accepted):** a pan now
     changes `clip_scene` every frame, so pan re-renders instead of reusing the
     cached image via the world transform as today.  Cheap for a viewport-sized
     FBO and required for correctness; the inherited "pan reuses the cached
     image" comment is now stale and must be updated in the same edit
     (plan-review suggestion 2).
   - Render: `renderImage(size, clip_scene)` (new overload).
   - Draw: `painter->drawImage(clip_local, cached_image_)` (was `boundingRect()`).

2. **`GggsTileLayer::renderImage(size, clip_bounds)`** — clip-aware overload:
   - Filter tiles to those whose scene-space extent (pre-computed via
     `geoToMap(minLat, minLon)` / `geoToMap(maxLat, maxLon)`) intersects
     `clip_bounds`.  Build the `QList<RasterFieldItem>` from the filtered set only.
   - Call `renderer_.renderToImage(draw, clip_bounds, ...)` (pass `clip_bounds` as
     `scene_bounds`, not `scene_bounds_`).
   - Keep the existing public `renderImage(const QSize& size)` (used by headless
     tests) as a delegate to `renderImage(size, scene_bounds_)`.
   - The clip overload is **public**, mirroring the existing public
     `renderImage(size)` — the headless clip-filter test calls it directly
     (plan-review must-fix 3).

3. **`GggsTileLayer`** — add member: `QRectF cached_clip_;` (after `cached_size_`).
   Invalidate `cached_image_` when `cached_clip_` changes (mirror the size check).

4. **`RasterLayer::paint()`** — same clip derivation, FBO sizing, cache key,
   drawImage.  No tile filtering (single-texture layer); `renderImage()` can
   accept the clip bounds and pass them straight to `renderToImage()`.
   Add `QRectF cached_clip_` member.

5. **`SonarLiveCacheLayer::paint()` + `renderImage()`** — same pattern; filter
   live-cache Entries by their scene-space extents intersecting `clip_scene`.
   Add `QRectF cached_clip_` member.  **Ordering constraint:** `items()` appends
   overview tiles first, then fine tiles (the ADR-0010 LOD fallback,
   `sonar_live_cache_layer.cpp:855-863`).  The clip filter must filter **both**
   pools while preserving that overviews-first order, so a clipped region whose
   fine tiles are evicted still draws its overview fallback instead of a blank
   gap (plan-review suggestion 3).

6. **New camp ADR-0011** — document the viewport-clip calling convention at the
   RasterGlRenderer seam: callers pass the visible sub-rect of `scene_bounds_`
   as the `scene_bounds` argument to `renderToImage()`, sized to the viewport,
   so the FBO covers only what the user sees.  Cross-references ADR-0007.

7. **Tests** — extend the already-registered `test/test_gggs_render.cpp`
   (its synthetic-store + `renderImage()` seam matches exactly; avoids a new
   `ament_add_gtest` CMake block — plan-review must-fix 2):
   - Creates a two-tile GGGS store where tiles A and B are geographically adjacent.
   - Calls `renderImage(size, clip_A)` — only tile A should contribute pixels.
   - Asserts tile B's region is transparent/absent in the result.
   - **Resolution assertion** (plan-review suggestion 1): render the same clip
     at viewport-sized FBO and assert the per-pixel density is that of the clip,
     not the full extent (e.g. a 1-px feature in tile A spans ≥ the expected
     pixel count when clipped vs. blurred full-extent render).
   Keep existing tests passing; `renderImage(size)` (full-extent) is unchanged.
   **Caveat (plan-review suggestion 1):** the headless test exercises the
   clip-filter seam, not `paint()`'s `clipBoundingRect()` derivation — the part
   that closes the field bug.  That derivation gets (a) the
   `test_gggs_render.cpp` `/tmp/*.png` visual-inspection extension and (b) a
   manual GUI verification note in the PR (open a large store, zoom in, confirm
   crisp render).

## Files to Change

| File | Change |
|------|--------|
| `src/camp_map/raster/gggs_tile_layer.h` | Add `QRectF cached_clip_`; add `QImage renderImage(const QSize&, const QRectF&)` **public** overload |
| `src/camp_map/raster/gggs_tile_layer.cpp` | Update `paint()` for clip-derived FBO (via `painter->clipBoundingRect()`); implement clip-filtered `renderImage(size, clip)`; update stale pan-cache comment |
| `src/camp_map/raster/raster_layer.h` | Add `QRectF cached_clip_` |
| `src/camp_map/raster/raster_layer.cpp` | Update `paint()` for clip-derived FBO; update `renderImage()` to accept optional clip |
| `src/camp_map/ros/live_coverage/sonar_live_cache_layer.h` | Add `QRectF cached_clip_`; update `renderImage()` signature |
| `src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp` | Update `paint()` for clip-derived FBO; filter both tile pools by clip, preserving overviews-first order |
| `docs/decisions/0011-viewport-clip-render-convention.md` | New camp ADR-0011 |
| `test/test_gggs_render.cpp` | Extend with clip-filter + resolution tests (already registered in CMake — no CMakeLists change needed) |

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

- [x] ~~`option->exposedRect` vs `painter->clipBoundingRect()`~~ — RESOLVED by
  plan review (must-fix 1): `option->exposedRect` defaults to `boundingRect()`
  without `ItemUsesExtendedStyleOption` (set nowhere in camp), and with
  `clip_local == boundingRect()` the `mapRect(clip_local)` FBO sizing reproduces
  today's buggy full-extent sizing — the original "likely no" reasoning was
  wrong.  The plan now derives the viewport via
  `painter->clipBoundingRect().intersected(boundingRect())` (no flag
  dependency).

## Estimated Scope

Single PR ("Part of #103"). Touches three `.cpp`/`.h` pairs plus one new ADR and
one new test file. ~200–300 lines of implementation change.
