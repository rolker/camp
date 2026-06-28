---
issue: 129
---

# Issue #129 — Running tasks on chart + selection sync (P2)

## Issue Review
**Status**: complete
**When**: 2026-06-28 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #129
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Actions
- [ ] Resolve frame-handling strategy before or early in plan-task: determine which frame `poses[].header.frame_id` is populated with (earth/map frame vs. something requiring a TF lookup per update) and document the decision — either in code comments or a brief design note in the plan.
- [ ] Confirm or fix the P1 carry-over guard: `RunningTasksView::taskSelected` must not fire for synthetic group rows (only `hasTask` leaf nodes); verify this is enforced and add a test if not already present.
- [ ] Clarify `getGeoCoordinate` access path: `RunningTasksView` uses node-injection (not `camp_ros::ROSWidget`), so the CAMP-only glue layer (which has access to `camp_ros`) must own the frame conversion — confirm the architectural split is explicit in the plan so the core widget stays clean.
- [ ] Decide and document redraw cadence: rebuild-all vs. diff graphics items on each `TaskFeedback` republish; document the choice (a brief comment in the overlay class suffices unless significant).
- [ ] Dependency gate: confirm PR #128 (P1) is merged before starting implementation.

## Plan Authored
**Status**: complete
**When**: 2026-06-28 12:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-129/plan.md` at `e5094f8`
**Branch**: feature/issue-129 at `e5094f8`
**Phases**: single

### Open questions
- [ ] Which frame does the boat populate `poses[].header.frame_id` with in practice — determines TF timeout tuning.
- [ ] Should tree→map selection optionally recenter the view on the task's first pose?

## Plan Review
**Status**: complete
**When**: 2026-06-28 15:57 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-129/plan.md` at `e5094f8`
**PR**: PR-less (`--issue` mode)
**Verdict**: approve-with-suggestions

### Findings
- [ ] (must-fix) No test planned for the `hasTaskPoses` guard — `review-issue` action #2 required one, P1 added none, and the gtest suite is a primary gate. Add `test/test_running_tasks_model.cpp` (empty-poses group row vs. leaf with poses) wired via `ament_add_gtest` (mirror `test_collision_monitor_converter`). — `plan.md:67-82`
- [ ] (suggestion) Simplify scene access — `Platform` is a `GeoGraphicsItem` (`ShipTrack : public GeoGraphicsItem`), so use `Platform::scene()` directly rather than parent-chain/passed-in pointer. — `plan.md:57-62`
- [ ] (suggestion) Close Open Question #1 in-plan — `TaskInformation.poses` is `geometry_msgs/PoseStamped[]` (per-pose headers); `getGeoCoordinate` already transforms each to `earth` with stale-stamp retry, so the frame is schema-answered. — `plan.md:108-113`
- [ ] (suggestion) Document the redraw cadence (rebuild-all) in the overlay class — `review-issue` action #4. — `plan.md:50-55`
- [ ] (suggestion) Note/guard the selection-sync loop (idempotent map→tree→map round-trip), mirroring the `QSignalBlocker` pattern in `applyPendingTasks`. — `plan.md:52-55`
