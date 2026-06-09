---
issue: 70
---

# Issue #70 — Markers replacement shipped without parity: DELETEALL spec violation + 5 unported camp behaviors (#59 regression)

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-09 07:12 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: approved

**Branch**: feature/issue-70 at `e27652d`
**Mode**: pre-push
**Depth**: Standard (reason: ~120-line C++ change touching QGraphicsItem/Map-model object lifecycle)
**Must-fix**: 1 (fixed before push) | **Suggestions**: 2

### Findings
- [x] (must-fix) Raw `delete` of Marker/MarkerNamespace bypassed the Map QAbstractItemModel (ADR-0003), leaving the Layers-tab tree view with dangling rows (use-after-free). Confirmed independently by Claude + Copilot adversarial passes. Fixed: added `Layer::removeFromMap()` (canonical detach-through-model path) and routed all marker/namespace deletion + the existing "Remove" action through it — `src/camp2/map/layer.cpp`, `src/camp2/ros/markers/{markers,marker_namespace}.cpp`
- [x] (suggestion) Throttled log calls constructed local system-time `rclcpp::Clock`s; switched to the node clock so intervals track sim time under use_sim_time — `src/camp2/ros/markers/markers.cpp`
- [ ] (suggestion) MODIFY action renders nothing (clears visuals, draws only for ADD). Pre-existing on jazzy, NOT a regression of this change; worth a follow-up to treat MODIFY as ADD in the guard + render switch — `marker.cpp:33`, `markers.cpp`
