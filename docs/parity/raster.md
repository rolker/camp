# Parity: background-raster

**camp**: `backgroundraster.{h,cpp}`, `georeferenced.{h,cpp}`,
`backgrounddetails.{h,cpp,ui}`
**camp2**: `raster/raster_layer.{h,cpp}`, `map_view/web_mercator.{h,cpp}`

## Architectural difference

camp's `BackgroundRaster` is a **chart-native, pixel-space** scene object: it
opens the GDAL file once, keeps the dataset's *native projection* (WKT →
`OGRCoordinateTransformation` in `Georeferenced`), and the whole Qt scene lives
in that raster's pixel frame. Geo↔scene is the per-file GDAL affine + WKT
reprojection (`georeferenced.cpp:84-92`). It loads **synchronously in the
constructor**, builds an RGBA `QImage` + mipmap pyramid, and — critically —
inspects bands for a **Float32 depth band**, retains the raw depth array, and
exposes depth queries used by A\*, survey generation, and the cursor readout.
There is exactly one BackgroundRaster (the project's depth raster).

camp2's `RasterLayer` is a **Web-Mercator layer** in a multi-layer scene. The
scene is EPSG:3857, so on load it **reprojects every file into Web Mercator**
via `GDALAutoCreateWarpedVRT` (`raster_layer.cpp:78`) and positions the pixmap
with a scene transform. Geo↔scene is the closed-form `web_mercator` functions.
Loading is **asynchronous** (`QtConcurrent` + abort flag). It treats all bands
as color/index imagery — **no depth concept whatsoever**. Opacity/visibility/
status/tree display come from the `map::Layer`/`MapItem` base.

## Feature matrix

| Feature | camp (file:line) | camp2 (file:line) | Delta | Resolution |
|---|---|---|---|---|
| GDAL open | `backgroundraster.cpp:11` (sync ctor) | `raster_layer.cpp:70` (async) | differs | keep camp2 async |
| Band → RGB(A) compositing | `backgroundraster.cpp:27-117` | `raster_layer.cpp:101-143` | parity | none (near-verbatim copy) |
| **Depth band (Float32) extraction** | `backgroundraster.cpp:31-68` (`GDT_Float32`, `m_depth_data`, min/max, depth shading) | none | **camp-only (port)** | PR3c: detect Float32 band, retain depth array + bounds |
| **Depth query `getDepth()`** | `backgroundraster.cpp:229-240`, `.h:36-37`; consumed by `astar.cpp:109-232`, `surveyarea.cpp:329`, `trackline.cpp:256`, `projectview.cpp:220-224` | none | **camp-only (port)** | PR3c: `getDepth(QGeoCoordinate)` over enabled depth layers, top wins |
| `depthValid()` predicate | `backgroundraster.cpp:134-137` | none | **camp-only (port)** | PR3c with depth port |
| Depth color shading (land green / depth red-ramp) | `backgroundraster.cpp:48-62` | none | camp-only (port) | PR3c: carry into depth render path. Hardcoded ramp is a candidate to migrate onto the reusable colormap facility ([#63](https://github.com/rolker/camp/issues/63)) |
| Color table / palette apply | `backgroundraster.cpp:73,85-92` | `raster_layer.cpp:103,112-119` | parity ✓ | none |
| Grayscale / per-band interp | `backgroundraster.cpp:95-108` | `raster_layer.cpp:122-135` | parity | none |
| Mipmap pyramid | `backgroundraster.cpp:119-123` | `raster_layer.cpp:145-149` | parity | none |
| LOD selection in paint | `backgroundraster.cpp:151-168` | `raster_layer.cpp:46-49` (`levelOfDetailFromTransform`) | differs | keep camp2 (cleaner Qt idiom) |
| Reprojection model | per-file native WKT + affine (`georeferenced.cpp:26-92`) | reproject-to-WebMercator on load (`raster_layer.cpp:78`) | differs | keep camp2 (intended new arch) |
| geoToPixel / pixelToGeo | `georeferenced.cpp:84-92` (per-raster) | scene-level `web_mercator::geoToMap/mapToGeo` (`web_mercator.cpp:16-33`) | differs | depth port needs a native-frame geo→pixel sampler (see Open Q) |
| Pixel size in meters | `backgroundraster.cpp:19-22`, `pixelSize()` `:204` | `metersPerUnit` scale-at-latitude (`web_mercator.cpp:35-38`) | differs | confirm `pixelSize()` consumers get an equivalent |
| `scaledPixelSize()` / `mapScale()` | `backgroundraster.cpp:209-222`; consumed by `markers.cpp:148`, `ais_contact.cpp:245`, `platform.cpp:55` | no analog; scene transform + `metersPerUnit` | differs | verify camp2 overlay ports no longer need a per-bg scale accessor |
| metersPerUnit / distortion correction | none (native proj already metric) | `web_mercator.cpp:35-38` (`cos(lat)`) | camp2-only (keep) | required for Web-Mercator |
| Async load + abort-on-destroy | none (blocking ctor) | `raster_layer.cpp:60-64,26-32,139-141` | camp2-only (keep) | robustness |
| Loading status text | none | `raster_layer.cpp:62,164` | camp2-only (keep) | none |
| Opacity control | none | `MapItem::setOpacity` (`map_item.h:111`) | camp2-only (keep) | none |
| Persistence (filename) | `backgroundraster.cpp:187-202` (write/read) | `map::Layer::readSettings/writeSettings` (`layer.h:27-28`) | differs | confirm filename round-trips through layer settings |
| Details/inspector UI | `backgrounddetails.cpp:17-22` (Path + Projection) | none raster-specific | camp-only (minor) | optional: surface filename/projection in layer properties |

## Open questions

- **Depth sampling frame after reprojection.** camp samples depth in the file's
  *native* pixel grid. camp2 warps imagery to Web Mercator and discards the
  native grid. A ported `getDepth(geo)` must keep the un-warped Float32 band +
  its native transform for sampling (bilinear warp would corrupt exact depth —
  prefer nearest-neighbor / native sampling). Decide in PR3c.
- **Single-depth-raster vs multi-layer.** camp assumes one project-wide depth
  raster (`getDepthRaster()`, `autonomousvehicleproject.cpp:226`). The plan's
  multi-depth-layer model needs a designation/selection mechanism for which
  layer(s) the `getDepth(geo)` query walks (resolved decision: enabled depth
  layers in tree order, first valid wins).
- **Depth band heuristic.** camp picks the *first* `GDT_Float32` band
  (`backgroundraster.cpp:32`). Robustness for post-OSM/WMTS chart files
  unverified.
- **Scale-accessor consumers.** markers/AIS/platform/grids call
  `bg->scaledPixelSize()`/`mapScale()`/`pixelSize()`. Whether their camp2 ports
  obtain scale via `metersPerUnit`/scene transform (making these unnecessary)
  vs needing a RasterLayer-level shim — settle as each overlay migrates.
- **Depth sign convention.** camp treats `depth <= 0` as land
  (`backgroundraster.cpp:49`); confirm camp2 target chart data uses the same
  positive-down convention.
