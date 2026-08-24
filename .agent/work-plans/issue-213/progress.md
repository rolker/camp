---
issue: 213
---

# Issue #213 — SIGSEGV risk: Polygon layer has no destructor, so its render worker outlives it (same defect as #209)


## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-24 16:43 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-213 at `c97153c` (uncommitted working tree; HEAD == origin/jazzy)
**Mode**: pre-push
**Depth**: Light (26-line, 2-file diff) with the adversarial pass primed on the Deep concurrency/lifecycle lens
**Must-fix**: 2 | **Suggestions**: 3
**Round**: 1 | **Ship**: continue — a new `std::terminate` path in the destructor must be closed before push

### Findings
- [ ] (must-fix) Worker exception rethrown by `waitForFinished()` escapes the destructor (implicitly noexcept) -> `std::terminate`; `frameOriginInWebMercator` throws `tf2::TransformException` on TF cold start. Wrap the transform in try/catch as `occupancy_grid.cpp:148` and `grid_map.cpp:160` already do — `src/camp_map/ros/geometry/polygon.cpp:57` (join at `:41`)
- [ ] (must-fix) `waitForFinished()` can steal-and-run the still-queued worker on the GUI thread inside the destructor, making `emit newPolygonData` a direct call so `updatePolygon()` mutates scene state during teardown and bypasses the new gate. Re-check `shutdown_` immediately before the emit — `src/camp_map/ros/geometry/polygon.cpp:63`
- [ ] (suggestion) Gate narrows but does not close the launch window: a callback preempted after the `shutdown_` check can still assign `process_future_` after the join returned. Same residual as merged OccupancyGrid; recorded as the camp#212 `shared_ptr<CallbackState>` follow-up. GridMap's `mutex_` + `shutdown_` handshake (`grid_map.cpp:47`) closes it in ~15 lines if wanted tonight — `src/camp_map/ros/geometry/polygon.cpp:41`
- [ ] (suggestion) `memory_order_relaxed` is functionally adequate (nothing is published through the flag), but the comments assert a store ordering relaxed does not provide; use seq_cst or soften the wording — `src/camp_map/ros/geometry/polygon.cpp:36`, `polygon.h:49`
- [ ] (suggestion, pre-existing) The constructed `rclcpp::QoS qos(10); qos.durability_volatile();` is never passed to `create_subscription` (plain depth `10` is used), so the configured QoS is silently discarded — `src/camp_map/ros/geometry/polygon.cpp:24`

### Verified clean
- No cross-thread Qt access in `processPolygon`: it touches only `QPolygonF`/`QPointF`, and the TF cache is mutex-guarded in `ros::Layer`. No camp#208-style `isVisible()` race, so the absence of a `visible_` mirror and an `itemChange` override is correct — Polygon has no per-update rasterisation cost to gate.
- Queued `newPolygonData` events pending at destruction are safe (`~QObject` removes posted events for the receiver).
- Destructor runs on the GUI thread (`map/layer.cpp:70` `deleteLater()`); the join is short for polygon-sized work, so no meaningful UI stall.
- Static analysis: no C++ linter is configured in camp's pre-commit; ament_cpplint findings on these files are pre-existing house-style divergences, none introduced by this diff.

## Integrated Review
**Status**: complete
**When**: 2026-08-24 19:22 -04:00
**By**: Claude Code Agent (Claude Opus 5)

**PR**: #214 at `1cbe25f`
**Sources**: 2 (Copilot R1 @ `1cbe25f`, Local Review (Pre-Push) @ `c97153c`)
**Cross-source confirmations**: 1
**CI**: all-pass (build-and-test, copilot-pull-request-reviewer)

### Findings
- [ ] (cross-confirmed) `process_future_` is read/written from two threads without synchronization — the ROS executor thread calls `isRunning()` and assigns it in `polygonCallback`, while the GUI thread reads it in `~Polygon()`. That is a C++ data race (UB), and it leaves the launch window open: a callback already executing when `subscription_.reset()` returns can assign a NEW worker after the destructor captured/joined. Fix by mirroring `GridMap` (`grid_map.cpp:47-64`): add a `QMutex mutex_`, reset the subscription FIRST, then under the lock set `shutdown_` and copy `process_future_` into a local `pending`, and `waitForFinished()` on the copy OUTSIDE the lock. `polygonCallback` takes the same lock to check the gate and assign; `processPolygon`'s pre-emit re-check takes it too (no deadlock — the destructor releases the lock before joining) — `src/camp_map/ros/geometry/polygon.cpp:48`, `polygon.h:49`
- [ ] (suggestion, pre-existing, Local Review) The constructed `rclcpp::QoS qos(10); qos.durability_volatile();` is never passed to `create_subscription` (plain depth `10` is used), so the configured QoS is silently discarded — `src/camp_map/ros/geometry/polygon.cpp:24`

### Resolved this round
- (must-fix, Local Review) Worker exception rethrown by `waitForFinished()` escaping the noexcept destructor -> `std::terminate`. Fixed in `1cbe25f`: `processPolygon` wraps the transform in try/catch and warns.
- (must-fix, Local Review) `waitForFinished()` steal-and-run making the emit a direct call during teardown. Fixed in `1cbe25f`: `shutdown_` re-checked immediately before the emit.
- (suggestion, Local Review) Comments overclaimed what `memory_order_relaxed` guarantees. Fixed in `1cbe25f`: wording softened to say the flag is advisory and the join is what synchronizes.

### False positives
- None. Copilot's single finding is valid and corroborates the pre-push review's own residual note.
