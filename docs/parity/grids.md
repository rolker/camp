# Parity: grids (nav_msgs/OccupancyGrid + grid_map_msgs/GridMap)

**camp**: `grids/grid.{h,cpp,ui}` — *one class, both message types*
**camp2**: `ros/grids/{grid_layer,occupancy_grid,grid_map}.{h,cpp}` — *split classes*

## Architectural difference

**camp** uses a single `Grid` class (`grid.h:10`, a `ROSWidget` +
`GeoGraphicsItem`) handling **both** types, dispatched by the `type_` string in
`visibilityChanged()` (`grid.cpp:209-212`) → `occupancyGridCallback`
(`grid.cpp:60`) or `gridMapCallback` (`grid.cpp:95`). Both produce a single
`GridData` (`grid.h:35`) with one `QImage`, rendered via `painter->drawImage`
in `paint()`. The widget owns its own "show" checkbox + topic label
(`grid.ui:18-29`) and **subscribes lazily only while visible**.

**camp2** splits by type into `Layer` subclasses under a shared ROS `Layer`
base (`ros/layer.h:17`): `OccupancyGrid` (`occupancy_grid.h:22`), `GridMap`
(`grid_map.h:31`, processes **all** layers, creating a child `GridLayer` per
grid_map layer), and `GridLayer` (`grid_layer.h:15`, thin per-layer pixmap).
Rendering is via `QGraphicsPixmapItem` children positioned through
`setWebMercatorPositionAndScale`, with pixel work on a `QtConcurrent` worker.
Opacity/visibility/status are base-class concerns.

## Feature matrix

| Feature | camp (file:line) | camp2 (file:line) | Delta | Resolution |
|---|---|---|---|---|
| Both grid types | one `Grid`, dispatch by string (`grid.cpp:209-212`) | `OccupancyGrid` + `GridMap` classes | differs | keep camp2 split |
| **OccupancyGrid colormap** (0–100 + unknown) | unknown=grey128a; 100=magenta; 99=cyan; else red↔blue ramp, a=`value+100` (`grid.cpp:73-80`) | identical, byte-for-byte (`occupancy_grid.cpp:65-72`) | **parity ✓** | none — already ported verbatim |
| **GridMap colormap** | `"speed"` layer: <0=red; else green ramp `value/3` (`grid.cpp:124-134`); other layers: green-on-alpha (`grid.cpp:140-141`) | generic grayscale from normalized value; NaN→transparent (`grid_map.cpp:101-105`) | differs (**camp-only speed ramp**) | **DECIDED 2026-06-02: drop speed special-case → adopt camp2 grayscale default.** Speed ramp was situational, not worth preserving. Selectable named colormaps deferred to a reusable facility, [#63](https://github.com/rolker/camp/issues/63) |
| GridMap value range | **fixed** (speed `/3.0`; others `[0,1]`) (`grid.cpp:132,140`) | **auto** per-layer min/max scan (`grid_map.cpp:76-104`) | camp2-only (keep) | keep auto; optional fixed/configurable range lands with the colormap facility ([#63](https://github.com/rolker/camp/issues/63)) |
| OccupancyGrid value range | fixed 0–100 (`grid.cpp:80`) | same (`occupancy_grid.cpp:72`) | parity | none |
| Transparency / alpha | graded per type (`grid.cpp:74-141`) | OccGrid identical; GridMap opaque + NaN-transparent (`grid_map.cpp:74,101`) | differs | OccGrid parity; GridMap resolve with colormap; keep NaN-transparency |
| Layer-level opacity | none | `MapItem::setOpacity` (`map_item.h:111`) | camp2-only (keep) | inherit |
| Frame/pose → geo placement | tf→earth→ECEF→lat/long (`grid.cpp:152-180`); OccGrid center offset by half-extent (`grid.cpp:85-86`) | `transformToWebMercator` + `setWebMercatorPositionAndScale` (`occupancy_grid.cpp:46,96`; `grid_map.cpp:59,117`) | differs | keep camp2; **verify OccGrid centering equivalence** (Open Q) |
| Resolution / origin | `info.resolution` (`grid.cpp:64,122`) | same (`occupancy_grid.cpp:55`, `grid_map.cpp:66`) | parity | none |
| Cell rendering | per-cell `setPixelColor` → `drawImage` (`grid.cpp:33-44`) | per-cell → `QPixmap` child, Y-flip transform (`occupancy_grid.cpp:97`, `grid_layer.cpp:34`) | differs | keep camp2 pixmap-item approach |
| Multi-layer GridMap | renders **one** layer (prefers `"speed"`, else first) (`grid.cpp:111-118`) | renders **all** layers as child `GridLayer`s (`grid_map.cpp:68-110`) | camp2-only (keep) | keep multi-layer; add a layer *selector* only if wanted (UX) |
| Threading | synchronous in callback (`grid.cpp:60,95`) | `QtConcurrent`, drops frames while busy (`occupancy_grid.cpp:33-36`) | camp2-only (keep) | keep — unblocks executor |
| QoS | default depth-1 (`grid.cpp:210`) | depth-1 + `durability_best_available()` (`occupancy_grid.cpp:20-21`) | camp2-only (keep) | keep — picks up transient-local |
| Show / lazy subscribe | own checkbox; subscribes only while checked (`grid.cpp:206-218`) | subscribes at construction (always on); visibility via layer model | differs | **verify** layer framework lazy-subscribes; else port (bandwidth regression) |
| Status / range readout | none | `setStatus` type + per-layer range (`occupancy_grid.cpp:28`, `grid_layer.cpp:38-41`) | camp2-only (keep) | inherit |
| Convert/empty-layer warns | throttled 2000 ms (`grid.cpp:101,107`) | throttled `2` / `2.0` (`grid_map.cpp:48,53`) | differs (**suspected bug**) | confirm throttle unit; 2 ms ≈ unthrottled — likely camp2 defect to fix |

## Open questions (flag for manual bag-replay verification)

- ~~**GridMap colormap parity.**~~ **RESOLVED 2026-06-02:** the camp `"speed"`
  ramp was situational (origin not recalled) and is **deliberately dropped** —
  camp2's grayscale auto-range becomes the grid_map default. A reusable
  selectable-colormap facility (`camp::map::ColorMap`: grayscale + a few generic
  ramps, value+range→QColor, per-layer selector, also a depth-shading consumer)
  is tracked separately in [#63](https://github.com/rolker/camp/issues/63), out
  of #59 scope. No bag-replay colormap gate remains for the port.
- ~~**Fixed vs auto range for speed.**~~ Moot — grayscale auto-range is the
  accepted default; configurable/fixed range lands with [#63](https://github.com/rolker/camp/issues/63).
- **OccupancyGrid centering equivalence.** camp shifts the *geo center* by half
  the extent (`grid.cpp:85-86`); camp2 transforms raw `info.origin` and shifts
  the *pixmap* by `+height` then Y-flips (`occupancy_grid.cpp:96-98`). Should be
  equivalent — verify alignment against a known bag (half-cell/half-grid offset
  bug plausible).
- **GridMap non-speed alpha.** camp used value-as-alpha green
  (`grid.cpp:140-141`); camp2 uses opaque grayscale + NaN-transparency. Whether
  the value-as-alpha look is wanted is a UX call.
- **Warn-throttle unit** (`grid_map.cpp:48,53` `2`/`2.0` vs camp `2000`): if ms,
  camp2 is effectively unthrottled — confirm the API unit and fix.
- **Lazy subscribe / visibility gating:** not visible in camp2 grid files (no
  checkbox); confirm the layer framework starts/stops the subscription on
  show/hide, else port camp's bandwidth-saving lazy subscribe.

> `grid_manager.*` (retiring) not read; confirm it holds nothing beyond topic
> discovery/instantiation before deleting (PR5).

## PR3a-i orientation note (Web-Mercator scene)

After PR3a-i the scene is Web Mercator with a Y-flipped view (north up). camp's
`Grid::paint` `drawImage`s the costmap pixmap (`grids/grid.cpp:41`), which the flipped
view mirrors vertically. Like markers, this is **not** patched in camp (Bucket A
retire-and-replace): camp2's `ros/grids` (occupancy_grid / grid_map) is written for the
Web-Mercator scene and supersedes it in PR5. Confirm the replacement costmap renders
right-side-up as part of the PR5 parity gate.
