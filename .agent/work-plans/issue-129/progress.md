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
