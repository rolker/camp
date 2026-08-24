---
issue: 209
---

# Issue #209 — CAMP segfaults when a costmap layer is removed

## Local Review
**Status**: complete
**When**: 2026-08-24 14:45 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**PR**: #210 at `2670cff`
**Mode**: post-PR
**Depth**: Deep (reason: concurrency/lifecycle + cross-thread teardown in a GUI/ROS boundary)
**Must-fix**: 4 | **Suggestions**: 8
**Round**: 1 | **Ship**: continue — a startup regression in `2670cff` leaves every grid layer blank until toggled

### Findings
- [ ] (must-fix) `visible_{false}` never receives an ItemVisibleHasChanged for a layer born visible, so every costmap and grid_map layer renders nothing until manually unchecked/rechecked — `src/camp_map/ros/grids/occupancy_grid.h:37`, `src/camp_map/ros/grids/grid_map.h:104`
- [ ] (must-fix) Live-tile caches already on disk still carry the single-slot GeoTIFF sentinel; legacy finite empties fold into overviews and get re-stamped with a NaN tag, so the purple tile survives the fix — `src/camp_map/ros/live_coverage/sonar_live_tile.cpp:335`
- [ ] (must-fix) `~OccupancyGrid` has no shutdown gate: `subscription_.reset()` does not stop an in-flight callback, which races `process_future_` and can launch a worker into the dying object — `src/camp_map/ros/grids/occupancy_grid.cpp:100`
- [ ] (must-fix) `SonarLiveBand` header still documents NoData as the dequantized producer sentinel, the exact contract the branch inverted — `src/camp_map/ros/live_coverage/sonar_live_tile.h:28`
- [ ] (suggestion) `grid.data[row_start + col]` is unbounded against a short payload — heap over-read — `src/camp_map/ros/grids/occupancy_grid.cpp:163`
- [ ] (suggestion) `setColormap()` rasterises a hidden layer, bypassing the new gate — `src/camp_map/ros/grids/grid_map.cpp:252`
- [ ] (suggestion) `visible_` is public in OccupancyGrid but private in GridMap; nothing external writes it — `src/camp_map/ros/grids/occupancy_grid.h:37`
- [ ] (suggestion) `visible_` sits under the "guarded by mutex_" comment block though it is deliberately lock-free — `src/camp_map/ros/grids/grid_map.h:99`
- [ ] (suggestion) New gtest target omits the include dirs / ament deps / Qt / GDAL links every sibling target carries; builds only via transitive propagation — `CMakeLists.txt:1047`
- [ ] (suggestion) Indexed8 scanline padding bytes are never written (uninitialised memory crosses a queued signal) — `src/camp_map/ros/grids/occupancy_grid.cpp:161`
- [ ] (suggestion) No test instantiates either grid layer; a mirror-vs-isVisible assertion would have caught the blocking finding — `test/`
- [ ] (suggestion) A latched OccupancyGrid publisher would stay blank after a toggle (no cached message, no re-render on show); deployed costmaps republish, so latent only — `src/camp_map/ros/grids/occupancy_grid.cpp:113`
