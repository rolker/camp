# Plan: Running tasks on chart + selection sync (P2)

## Issue

https://github.com/rolker/camp/issues/129

## Context

P1 (#127 / PR #128) delivered a read-only `RunningTasksView` tree widget that
subscribes to `marine/status/mission_tasks` and renders `TaskFeedback` as a
structured tree.  The widget emits `taskSelected(QString id)` on row activation
and exposes `setSelectedTask(id)` for external selection.  Neither signal is yet
wired to anything outside the widget.

P2 adds three things on top of that foundation:

1. A per-task geometry overlay on CAMP's `QGraphicsScene` — point for single-pose
   tasks (goto), polyline for multi-pose tasks (survey_line), using the existing
   `GeoGraphicsItem` machinery but visually distinct from editable plan items.
2. Two-way selection sync between `RunningTasksView` and the overlay.
3. The CAMP-only glue class that owns the overlay items and wires the signals.

A hasTask guard (carry-over from P1) is included: `taskSelected` must not fire
for group rows that have no poses, so map glue isn't triggered by clicks on
pure-structural parent rows.

## Approach

1. **Add `hasTaskPoses` to `RunningTasksModel`** — predicate that returns true when
   the node at a `QModelIndex` has `task->message().poses` non-empty.  Guard
   `RunningTasksView::onCurrentRowChanged`: only emit `taskSelected` when
   `model_->hasTaskPoses(current)` is true.

2. **Add `tasksUpdated` signal to `RunningTasksView`** — emit from the end of
   `applyPendingTasks()` with the current-task id and the raw `TaskInformation`
   vector.  This lets the CAMP glue layer refresh overlay geometry without a
   second ROS subscription.

3. **Add `TaskOverlayItem`** (`GeoGraphicsItem` subclass, lives in
   `src/camp/running_tasks/`) — renders one task's poses on the scene.
   - Single pose → filled circle (goto).
   - Two or more poses → polyline (survey_line, transit, etc.).
   - Visual states: current (emphasized, brighter), pending (muted), done (gray).
   - Emits `clicked(QString id)` for map→tree selection sync.
   - New type constant `RunningTaskType` added to `GeoGraphicsItem`'s enum.

4. **Add `RunningTasksOverlay`** (`camp_ros::ROSWidget` subclass, lives in
   `src/camp/running_tasks/`) — the CAMP-only glue.
   - Initialized with the `QGraphicsScene*` and a `RunningTasksView*`.
   - Holds `QMap<QString, TaskOverlayItem*>` keyed by task id.
   - `tasksUpdated` → rebuild overlay: call `getGeoCoordinate` (from ROSWidget) for
     each pose, create/replace `TaskOverlayItem`s on the scene.
   - `taskSelected` → set `is_selected_` on the matching item, clear others,
     call `item->update()`.
   - `TaskOverlayItem::clicked(id)` → `runningTasksView_->setSelectedTask(id)`.

5. **Wire in `Platform`** — after `runningTasksView->setNode(node_)`, construct
   `RunningTasksOverlay`, pass it the scene from the `AutonomousVehicleProject`
   (via `Platform`'s parent chain or a passed-in pointer) and connect it to
   `m_ui->runningTasksView`.  `RunningTasksOverlay` inherits `nodeStarted` via
   `ROSWidget`, so call `overlay_->nodeStarted(node_, transform_buffer_)` in
   `Platform::onNodeUpdated`.

6. **CMakeLists** — add `running_tasks/task_overlay_item.cpp` and
   `running_tasks/running_tasks_overlay.cpp` to the `SOURCES` list.

## Files to Change

| File | Change |
|------|--------|
| `src/camp/running_tasks/running_tasks_model.h` | Add `hasTaskPoses(QModelIndex)` declaration |
| `src/camp/running_tasks/running_tasks_model.cpp` | Implement `hasTaskPoses` |
| `src/camp/running_tasks/running_tasks_view.h` | Add `tasksUpdated` signal |
| `src/camp/running_tasks/running_tasks_view.cpp` | Emit `tasksUpdated` from `applyPendingTasks`; guard `onCurrentRowChanged` with `hasTaskPoses` |
| `src/camp/running_tasks/task_overlay_item.h` | New file — `TaskOverlayItem` (`GeoGraphicsItem` subclass) |
| `src/camp/running_tasks/task_overlay_item.cpp` | New file — paint + click + coordinate update |
| `src/camp/running_tasks/running_tasks_overlay.h` | New file — `RunningTasksOverlay` (`camp_ros::ROSWidget`) |
| `src/camp/running_tasks/running_tasks_overlay.cpp` | New file — glue: rebuild items, sync selection |
| `src/camp/geographicsitem.h` | Add `RunningTaskType` to the type enum |
| `src/camp/platform_manager/platform.h` | Add `RunningTasksOverlay*` member |
| `src/camp/platform_manager/platform.cpp` | Construct overlay, call `nodeStarted`, connect signals |
| `CMakeLists.txt` | Add two new `.cpp` files to `SOURCES` |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | Two new classes; no third subscription or duplicate data path |
| Workspace vs. project separation | All new code in `camp/` project repo — no workspace infra touched |
| A change includes its consequences | `CMakeLists.txt` updated; `geographicsitem.h` enum extended |
| Improve incrementally | Single PR building on P1 foundation; P3 editing deferred |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0002 (Web-Mercator scene) | Yes | `TaskOverlayItem` uses `geoToPixel` for scene coordinates; no chart dependency |
| ADR-0008 (ROS 2 conventions) | Yes | Node-injection idiom for `RunningTasksView` unchanged; ROSWidget pattern for overlay |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `GeoGraphicsItem` type enum | Any switch/cast on item types | Yes — add enum value; no existing switch covers it |
| `RunningTasksView` signals | rqt plugin (future) | No — `tasksUpdated` is additive; rqt path unaffected |
| `Platform` construction | Any future subclass of Platform | No follow-up needed — single concrete class |

## Open Questions

- Which frame does the boat populate `poses[].header.frame_id` with in practice?
  `getGeoCoordinate` will TF-lookup to earth regardless, but knowing the source
  frame helps set timeout expectations (earth→map is quasi-static; a more dynamic
  frame would need a tighter retry window).
- Should the overlay recenter the view on the selected task's first pose (tree→map)?
  The issue says "and optionally recenters"; leaving optional — implement if trivial
  once selection sync is working.

## Estimated Scope

Single PR.
