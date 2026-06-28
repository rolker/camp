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
- [x] (must-fix) No test planned for the `hasTaskPoses` guard — `review-issue` action #2 required one, P1 added none, and the gtest suite is a primary gate. Add `test/test_running_tasks_model.cpp` (empty-poses group row vs. leaf with poses) wired via `ament_add_gtest` (mirror `test_collision_monitor_converter`). — `plan.md:67-82`
- [x] (suggestion) Simplify scene access — `Platform` is a `GeoGraphicsItem` (`ShipTrack : public GeoGraphicsItem`), so use `Platform::scene()` directly rather than parent-chain/passed-in pointer. — `plan.md:57-62`
- [x] (suggestion) Close Open Question #1 in-plan — `TaskInformation.poses` is `geometry_msgs/PoseStamped[]` (per-pose headers); `getGeoCoordinate` already transforms each to `earth` with stale-stamp retry, so the frame is schema-answered. — `plan.md:108-113`
- [x] (suggestion) Document the redraw cadence (rebuild-all) in the overlay class — `review-issue` action #4. — `plan.md:50-55`
- [x] (suggestion) Note/guard the selection-sync loop (idempotent map→tree→map round-trip), mirroring the `QSignalBlocker` pattern in `applyPendingTasks`. — `plan.md:52-55`

## Implementation
**Status**: complete
**When**: 2026-06-28 17:30 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Commit**: `3c916c7` on `feature/issue-129`
**Build**: clean (1 package, warnings only — pre-existing)
**Tests**: 5/5 passing (`test_running_tasks_model`)

### Summary

Implemented all three P2 deliverables — map overlay, two-way selection sync,
and CAMP glue — plus all five plan-review findings:

**Model / View changes:**
- `running_tasks_model.h/.cpp`: added `hasTaskPoses(QModelIndex)` — returns
  true when the node's task has at least one pose.
- `running_tasks_view.h/.cpp`: guarded `onCurrentRowChanged` with
  `hasTaskPoses`; added `tasksUpdated(current, tasks)` signal emitted at end
  of `applyPendingTasks` for the overlay to consume without a second ROS
  subscription.

**New files:**
- `task_overlay_item.h/.cpp`: `GeoGraphicsItem` + `QObject` subclass
  (dual-inherit, `Q_INTERFACES(QGraphicsItem)`). Single pose→filled circle,
  multi-pose→polyline. Color states: selected=orange, current=green,
  done=gray, pending=blue-muted. Emits `clicked(id)` on mouse press.
  `RunningTaskType` added to `GeoGraphicsItem` enum.
- `running_tasks_overlay.h/.cpp`: `camp_ros::ROSWidget` subclass. Takes
  `QGraphicsScene*` (from `Platform::scene()` — plan-review finding #2) and
  `RunningTasksView*`. Rebuild-all strategy on every `tasksUpdated`
  (documented in class comment — finding #4). Idempotent selection guard
  mirrors `QSignalBlocker` pattern (finding #5). `getGeoCoordinate` handles
  `TaskInformation.poses` (geometry_msgs/PoseStamped[] per-pose headers with
  stale-stamp retry — finding #3 closed in code comment).
- `test/test_running_tasks_model.cpp`: 5 gtests for `hasTaskPoses` (finding
  #1); wired via `ament_add_gtest` mirroring `test_collision_monitor_converter`.

**Platform wiring:**
- `platform.h`: forward-declare `RunningTasksOverlay*`; add member.
- `platform.cpp`: construct overlay in `onNodeUpdated` using `scene()`
  directly (finding #2); call `nodeStarted` on re-entry.

**CMakeLists.txt:**
- Added two new `.cpp` sources; added `test_running_tasks_model` gtest target.

### Plan-review findings addressed
- [x] (must-fix) `test/test_running_tasks_model.cpp` wired via `ament_add_gtest`; 5 tests pass
- [x] (suggestion) `Platform::scene()` used directly — no parent-chain walk
- [x] (suggestion) Frame schema closed in `running_tasks_overlay.cpp` comment
- [x] (suggestion) Rebuild-all cadence documented in `RunningTasksOverlay` class comment
- [x] (suggestion) Selection-sync loop guarded with `id == selected_id_` early return

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-28 16:37 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-129 at `6820371`
**Mode**: pre-push
**Depth**: Deep (reason: 736 lines / 15 files — both >200 lines and ≥10 files; cross-layer GUI+ROS)
**Must-fix**: 3 | **Suggestions**: 5
**Round**: 1 | **Ship**: continue — genuine correctness bugs (multi-pose items unclickable; latent double-free on scene/overlay teardown)

### Findings
- [x] (must-fix) Multi-pose `shape()` returns a zero-area open polyline → Qt hit-tests via `shape().contains()`, so survey_line/transit overlay items never receive clicks; map→tree selection broken for all multi-pose tasks (contradicts class doc). Stroke the path (`QPainterPathStroker`) for `shape()` — `src/camp/running_tasks/task_overlay_item.cpp:47-50`
- [x] (must-fix) Double ownership of `TaskOverlayItem`: `scene_->addItem(item)` gives the scene ownership (parent nullptr) yet `clearItems()`/dtor also `delete item`; if the scene is destroyed before the overlay → use-after-free + double-free. `scene_` is a raw pointer with no `QPointer` guard. Follow the codebase convention (Qt parent ownership; this is the only `addItem` call site) and/or use `QPointer<QGraphicsScene>` — `src/camp/running_tasks/running_tasks_overlay.cpp:88,93-101`, `running_tasks_overlay.h:53`
- [x] (must-fix) Marker radii are fixed scene units (6.0 hit circle, 3.0 endpoints), not zoom-scaled like `Waypoint::shape()` (`1/mapScale()`) → markers sub-pixel when zoomed out / oversized when zoomed in; degrades single-pose hit area. Deviates from ADR-0003 constant-pixel-footprint convention (`metresPerPixel`) — `src/camp/running_tasks/task_overlay_item.cpp:30,90`
- [x] (suggestion) Rebuild-all deletes/recreates every `TaskOverlayItem` on each ~1 Hz republish even when unchanged → avoidable allocation churn + scene-region invalidation/flicker (cross-pass confirmed; documented as intentional) — `src/camp/running_tasks/running_tasks_overlay.cpp:50-91` (deferred: rebuild-all is an approved plan-time tradeoff; the cheap "skip if message unchanged" mitigation is unsafe because pose frames may be `map` — `getGeoCoordinate` re-resolves earth coords via TF every cycle, so identical pose messages still need redrawing; a correct incremental diff is larger than this thin fix phase warrants — left for a follow-up if profiling shows it matters)
- [x] (suggestion) `rebuildItems` guards `transform_buffer_` but not `node_`; `getGeoCoordinate`'s catch path derefs `node_->get_logger()` — crash if `node_` null while buffer set (low risk in practice) — `src/camp/running_tasks/running_tasks_overlay.cpp:54`
- [x] (suggestion) `onItemClicked` comment claims it "mirrors the QSignalBlocker pattern" but uses only an id-equality guard (no `QSignalBlocker`) — comment/code mismatch — `src/camp/running_tasks/running_tasks_overlay.cpp:44-46`
- [x] (suggestion) `selected_id_` is never cleared/validated when the selected task disappears from a later republish; harmless but tree/overlay can drift — `src/camp/running_tasks/running_tasks_overlay.cpp:30-38`
- [x] (suggestion) Overlay is created only inside `onNodeUpdated` when `scene()` is non-null; if `scene()` is null on first call and `onNodeUpdated` never fires again the overlay is silently never created (no fallback) — `src/camp/platform_manager/platform.cpp:243`

## Implementation
**Status**: complete
**When**: 2026-06-28 16:54 +00:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-129 at `4338718`
**Addressed**: Local Review (Pre-Push) (2026-06-28 16:37 +00:00, `feature/issue-129` at `6820371`)
**Commits**: `70febed`, `872e961`, `4338718`

### Actions
- [x] (must-fix) Multi-pose `shape()` is now stroked (`QPainterPathStroker`) into a constant-pixel-width clickable band, so survey_line/transit items hit-test and map→tree selection works — `src/camp/running_tasks/task_overlay_item.cpp:75-88` (`70febed`)
- [x] (must-fix) Scene/overlay teardown double-free fixed: `scene_` is now `QPointer<QGraphicsScene>` and `clearItems()` only deletes items while the scene is alive — `src/camp/running_tasks/running_tasks_overlay.h:55`, `running_tasks_overlay.cpp:101-117` (`872e961`)
- [x] (must-fix) Marker/hit radii are zoom-scaled to a constant display-pixel footprint via new `sceneRadius()` (ADR-0003) — `src/camp/running_tasks/task_overlay_item.cpp:32-42,50-55,67-73,125-131` (`70febed`)
- [x] (suggestion) `rebuildItems()` now also guards `node_` (getGeoCoordinate's catch path derefs `node_->get_logger()`) — `src/camp/running_tasks/running_tasks_overlay.cpp:55-58` (`872e961`)
- [x] (suggestion) `onItemClicked` comment corrected — it uses an id-equality guard, not `QSignalBlocker` — `src/camp/running_tasks/running_tasks_overlay.cpp:42-45` (`872e961`)
- [x] (suggestion) `selected_id_` is cleared when the selected task disappears from a republish — `src/camp/running_tasks/running_tasks_overlay.cpp:95-98` (`872e961`)
- [x] (suggestion) Overlay creation factored into `ensureRunningTasksOverlay()`, also called from periodic `update()`, so a late-attaching scene still gets an overlay — `src/camp/platform_manager/platform.{h,cpp}` (`4338718`)
- [x] (suggestion) Rebuild-all allocation churn / flicker — **deferred**: an approved plan-time tradeoff; the cheap "skip if message unchanged" mitigation is unsafe (pose frames may be `map`, so `getGeoCoordinate` re-resolves earth coords via TF each cycle even for an identical message), and a correct incremental diff exceeds this thin fix phase — `src/camp/running_tasks/running_tasks_overlay.cpp:51-99` (deferred: see reason)

### Verification
- Build/gtest **not run in this worktree**: the lower ROS layers are unbuilt here
  (`underlay_ws`/`core_ws`/… `install/` are empty, so camp's deps —
  `marine_ais_msgs`, `marine_nav_interfaces`, etc. — are unavailable to CMake).
  The fixes were verified by inspection against the cited Qt/codebase patterns
  (`ShipTrack::drawTriangle`, `Waypoint::shape`, `QPointer` ownership). The
  re-review (`review-code`) should build/run `test_running_tasks_model` in a
  fully-provisioned environment.
- No changes to `running_tasks_model` (the gtest target), so existing model
  tests are unaffected.

### Next step
Lifecycle: **Implementation** → **review-code** (re-review the fixes). Hand off to a
fresh-context sub-agent:

    .agent/scripts/dispatch_subagent.sh --mode in-process --issue 129 --skill review-code

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-28 17:12 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-129 at `c35c139`
**Mode**: pre-push
**Depth**: Deep (reason: 876 lines / 15 files — both >200 lines and ≥10 files; cross-layer GUI+ROS)
**Must-fix**: 0 | **Suggestions**: 3
**Round**: 2 | **Ship**: recommended — 0 must-fix; round-1's 3 must-fix all verified fixed; only low-impact suggestions remain

Round-1 must-fixes verified correctly resolved (2 independent adversarial lenses + codebase-convention check):
multi-pose `shape()` now stroked (clickable); teardown double-free fixed via `QPointer<QGraphicsScene>`;
marker/hit radii zoom-scaled via `sceneRadius()`. Static analysis (cppcheck) clean. Build/gtest not re-run
here (lower ROS layers unbuilt); model/test untouched since last 5/5 pass — re-run in a provisioned env before merge.

### Findings
- [ ] (suggestion) Duplicate task `id` in a republish overwrites `items_[id]` without removing the prior item from the scene → orphaned leak; add `if (items_.contains(id)) continue;` — `src/camp/running_tasks/running_tasks_overlay.cpp:81`
- [ ] (suggestion) `TaskOverlayItem::setSelected(bool)` shadows non-virtual `QGraphicsItem::setSelected`; rename to `setHighlighted` to avoid the footgun (not a live bug — nothing calls `scene->selectedItems()`) — `src/camp/running_tasks/task_overlay_item.h:38`
- [ ] (suggestion) `boundingRect()`/`shape()` are zoom-dependent but no `prepareGeometryChange()` on zoom → cached rect can briefly clip when zooming out; cosmetic, self-heals at ~1 Hz rebuild, matches existing `Waypoint` pattern — `src/camp/running_tasks/task_overlay_item.cpp:51-88`

### Next step
Lifecycle: **Local Review (approved)** → push / open PR → **triage-reviews**.
Branch is shippable; the 3 suggestions are optional and may be applied pre-push or tracked as follow-ups.
