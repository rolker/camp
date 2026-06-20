# Plan: GGGS-tiled raster map layer — GPU display-time warp (bathy + sidescan)

## Issue

https://github.com/rolker/camp/issues/90 (broadened; Part of rolker/unh_marine_autonomy#175 and #171)

## Context

camp#90 originally scoped a bathy-only, CPU directory-watching grid layer. Per the
2026-06-19 decisions it is **broadened** into the unified **GGGS-tiled raster layer**
(#175 / I4) that renders **both** products from the on-disk tile stores:
- **sidescan** — 1-band `uint16` backscatter (`marine_sidescan_mosaic` #173, just merged)
- **bathy** — 3-band `Float64` depth / uncertainty / timestamp (`marine_bathymetry_store`)

Tiles are native-geographic (WGS84) GeoTIFFs named `<level>_<row>_<col>.tif`. CAMP's map
core (`src/camp2`, Qt 5.15 + GDAL) is **entirely CPU** today: `RasterLayer` GDAL-warps each
GeoTIFF to EPSG:3857 on load (`raster_layer.cpp:77,125`), colormaps on the CPU
(`color_map.cpp`), and paints `QPixmap`s in a `QGraphicsScene` (`MapView : QGraphicsView`,
no GL viewport). #175's decision is to render GGGS tiles via a **GPU display-time warp**
instead: upload each native lat/lon tile once, warp into Web-Mercator per frame.

**Why GPU warp** (separable): geo→Web-Mercator is `x=R·λ` (longitude linear) and
`y=R·asinh(tan φ)` (the only nonlinearity, 1-D in latitude) → a tile is a vertically
tessellated textured mesh (N×1 strip, 4–16 subdivisions sub-pixel). Keeps the store
canonically GGGS-geographic (ADR-0002 §D2), no producer-side pre-warp, free reproject on
pan/zoom, and band-select + colormap become **shader uniforms** (instant switch, no reload).

## Approach — slices (stacked PRs on `feature/issue-90`)

**Slice 1 — GPU warp via offscreen FBO + registered static tile (de-risks #175 acceptance 1).**
**Rendering decision revised during implementation (2026-06-20):** native GL inside a
QGraphicsView item (`beginNativePainting`) does **not** get a current GL context on this
stack — confirmed under a Wayland software backingstore, where the view composites to a
shared-memory buffer, not a GL surface (it crashed on `QOpenGLFunctions` with a null
context). Per Roland's call, the layer now renders the GPU warp to an **offscreen
framebuffer it owns** and presents the result with `QPainter::drawImage()` — portable across
X11 / Wayland / software GL, and needing **no `QOpenGLWidget` viewport** (that change is
reverted; no app-wide viewport swap, so the camp#98 risk is moot).
1. New `GggsTileLayer : camp::map::Layer` (pure map, `CAMP_MAP_SOURCES`, no ROS). Load a tile
   *directory*: glob `*.tif`, read each tile's **native** band + geotransform via GDAL (no
   warp VRT), upload band as an `R32F` texture; `boundingRect()` = union of tile extents via
   `web_mercator::geoToMap`.
2. The layer owns a `QOpenGLContext` + `QOffscreenSurface`. `renderImage(size)` binds a
   `QOpenGLFramebufferObject`, draws each tile as a latitude-tessellated mesh through a
   `QOpenGLShaderProgram` whose **vertex shader replicates `web_mercator::geoToMap` exactly**
   (R=6378137, `asinh(tan φ)` as `log(t+√(t²+1))`), with an **ortho** mapping the extent to
   NDC; fragment shader = auto-ranged grayscale, no-data 0 → discard; premultiplied-alpha.
   `toImage()` returns the warped raster (row 0 = south, so it draws upright once MapView's
   `scale(s,−s)` view flip puts north up).
3. `paint()` sizes the render to the extent's on-screen pixels, caches it (re-render on zoom,
   reuse on pan), and `drawImage(boundingRect(), image)` — registers with the CPU
   raster/tile/marker layers; no GL needed from the viewport.
4. Verified headlessly (offscreen GL, no window): `test_gggs_render` warps a synthetic tile
   and the real store to a QImage; pixel analysis confirms geometry + north-up orientation.

**Slice 2 — warp correctness + multi-tile.** Tune tessellation so Mercator-y error < 0.5 px
across a tile; confirm shared-edge seams (identical `geoToMap(φ)`) are crack-free; LOD/zoom
behavior; many tiles without per-frame re-upload (texture cache keyed by `GridIndex` from the
filename).

**camp#63 (between Slice 2 and Slice 3) — GPU colormap facility.** Land the GPU half of
camp#63 *after* Slice 1 provides the GL viewport: adopt `marine_colormap::colormap_glsl()` +
`bake_lut()` (merged in `marine_colormap#6`) as a reusable CAMP GL colormap helper (palette as
1-D LUT, range/gain as uniforms), with the offscreen CPU/GPU **parity test** from the rqt
waterfall consumer (`rqt_operator_tools#48`), plus the CPU shared-core swap so CAMP colors
match rviz/rqt. Sequencing settled 2026-06-20: **I4 Slice 1 → camp#63 → I4 Slice 3** (the
facility needs the GL viewport Slice 1 builds; Slice 3 then consumes it — no throwaway LUT).

**Slice 3 — bands + colormap + watcher.** Depth/Uncertainty band-select (bathy) rendered via
**camp#63's GPU colormap facility** (palette LUT + range/gain uniforms, applied after the warp
in the same shader); sidescan stays single-band. `QFileSystemWatcher` on the store dir →
re-upload only changed tiles (matches the mosaicker's incremental flush; this is the original
camp#90 Tier-1 directory-watcher, now on the GPU layer). Context-menu band/colormap controls +
`readSettings`/`writeSettings` like `RasterLayer`.

**Slice 4 (deferred — own sub-issue).** Live dirty-region transport (I3 / #86 Phase 6;
original camp#90 Tier-2 live consumer).

## Files to Change

| File | Change |
|------|--------|
| `src/camp2/raster/gggs_tile_layer.{h,cpp}` | New layer: dir load, owns offscreen GL context + FBO, warp shaders, `renderImage()`, `drawImage()` in paint (no viewport change) |
| `src/camp2/raster/gggs_tile.{h,cpp}` | Per-tile: GDAL native read, geotransform extent, lazy R32F texture |
| inline shaders (in `gggs_tile_layer.cpp`) | geoToMap warp vertex + grayscale fragment (colormap = Slice 3) |
| `CMakeLists.txt` | Add sources to `CAMP_MAP_SOURCES`; **add `Qt5::Gui` (and `OpenGL` if used) to the `find_package(Qt5 …)` COMPONENTS and the `camp_map` link list** — `QOpenGLShaderProgram`/`QOpenGLFunctions` are in `Qt5::Gui`, `QOpenGLWidget` in `Qt5::Widgets`; don't rely on transitive Widgets |
| Layer registration (`background_manager.cpp` and/or an "Open tile store" action) | Expose the layer |
| `test/test_gggs_tile_layer.cpp` | Extent-from-geotransform + **geoToMap parity** (CPU mirror that calls `web_mercator::geoToMap` *directly* so it can't drift from the shader reference; tolerance < 1e-6 relative over φ ∈ [−85°, 85°], sampling Massabesic ~43°N) + no-data unit tests |
| `README` / repo docs | Document the layer |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | Slices 1–3 here; live transport (Slice 4) is a separate sub-issue gated on I3 |
| Decoupling | Layer reuses GDAL IO + `web_mercator` + `ColorMap` stops; no `gggs` dep (geometry from GeoTIFF geotransform). New GL path is additive — CPU `RasterLayer` stays the static-chart fallback |
| Simulation-First | De-risk against on-disk static tile sets (sidescan from the #173 Massabesic run; a bathy store epoch) before any live wiring |
| Safety First | Display/search aid, not control; but mis-registration misleads a search → shader must match `geoToMap` exactly, unit-tested via a CPU mirror |

## ADR Compliance

ADR refs below are **`unh_marine_autonomy` ADR-0002** (the bathy-store / GGGS ADR) — *not*
camp's local `docs/decisions/0002` (Web-Mercator scene, two-model split), which a bare "0002"
citation would mis-resolve to from inside the camp worktree.

| ADR | Triggered | How addressed |
|---|---|---|
| unh_marine_autonomy 0002 §D2 (canonical GGGS-geographic) | Yes | No producer-side pre-warp; tiles stay native-geographic, warped only at display |
| unh_marine_autonomy 0002 §D5 (bathy bands depth/unc/ts) | Yes | Band-select uniform renders depth or uncertainty (Slice 3); ts not rendered (F32 epoch caveat noted) |

## Consequences

| If we change… | Also update… | In plan? |
|---|---|---|
| `MapView` viewport → QOpenGLWidget | Verify all existing CPU layers still render; watch camp#98 (OOM/GDAL leak on zoom/pan) doesn't worsen | Yes (Slice 1 verify step) |
| GPU colormap (camp#63) | camp#63's GPU facility (marine_colormap GLSL+LUT) is built between Slice 2 and Slice 3 and consumed by Slice 3 — no interim/throwaway LUT | Yes — sequenced I4 Slice1 → #63 → Slice3 |

## Open Questions

- [x] **GL integration approach** → `QOpenGLWidget` viewport + `beginNativePainting()` + raw
  `QOpenGLShaderProgram` (only Qt-5.15-viable path; QRhi is Qt6). Slice 1 must verify CPU
  layers still render and watch camp#98 interaction. (Resolved 2026-06-20.)
- [x] **Colormap source** → not an interim LUT; Slice 3 consumes **camp#63's GPU colormap
  facility** (`marine_colormap` GLSL + `bake_lut`), built between Slice 2 and Slice 3.
  Sequencing: I4 Slice 1 → camp#63 → I4 Slice 3. (Resolved 2026-06-20.)
- [x] **Slicing** → stacked PRs on `feature/issue-90`, Slice 1 first. (Resolved 2026-06-20.)

## Estimated Scope

**Multiple stacked PRs** on `feature/issue-90` (Slices 1–3), with **camp#63** (GPU colormap
facility) landed between Slice 2 and Slice 3. Slice 4 (live transport) = a separate follow-on
sub-issue of #171, gated on I3.
