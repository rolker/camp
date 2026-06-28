# Plan: CAMP running-task view P1 — read-only structured task tree

## Issue

https://github.com/rolker/camp/issues/127  (Part of #123)

## Context

CAMP's per-robot panel (`Platform`, `platform.ui`) shows `helmManager` +
`missionManager` in a vertical splitter. The `MissionManager` placeholder just
dumps a flattened `Heartbeat` into a text box. The boat now publishes structured
`marine_nav_interfaces/TaskFeedback` (current task + `TaskInformation[]`) on
`marine/status/mission_tasks` (unh_marine_autonomy#236). This adds a read-only
structured tree view sourced from that topic.

## Approach

1. **`RunningTasksModel`** (`QAbstractItemModel`) — builds a tree from the flat
   `TaskInformation[]` using slash-delimited `id`s (`survey_a/line_1` → child of
   `survey_a`; synthesize intermediate group rows when a path segment has no
   explicit task). Columns: name / type / priority / status / done. Holds the
   `current_navigation_task` id; `data()` returns a bold font + highlight
   background for that row. `setTasks(current, rows)` rebuilds via
   begin/endResetModel (small list, low rate — reset is fine for P1).
2. **`RunningTasksView`** (`QWidget`, node-injection — the `helm_manager` idiom,
   NOT `camp_ros::ROSWidget`): `setNode()` + `updateRobotNamespace()`; owns a
   `QTreeView` + `RunningTasksModel` built in the constructor (no `.ui` needed).
   Exposes `taskSelected(QString id)` signal + `setSelectedTask(QString id)` slot
   (wired to selection now; map glue consumes them in P2).
3. **Subscription** — `TaskFeedback` on `<ns>/marine/status/mission_tasks`,
   created with the guarded Realtime callback group:
   `if (auto ctx = camp_ros::RosContext::instance()) opts.callback_group =
   ctx->group(camp_ros::RosContext::Group::Realtime);`. The executor-thread
   callback converts the message to a GUI-friendly `QVector<TaskRow>` under a
   mutex, then `QMetaObject::invokeMethod(this, "applyPendingTasks",
   Qt::QueuedConnection)` marshals to the GUI thread (same pattern as
   `MissionManager`). Never touch Qt from the ROS thread.
4. **Integration** — promote `RunningTasksView` in `platform.ui` (third widget in
   the splitter); in `platform.cpp` call `runningTasksView->setNode(node_)`
   (alongside the helm `setNode`) and `updateRobotNamespace(...)` (alongside the
   others).
5. **Build wiring** — add the two `.cpp` to `SOURCES` (CMakeLists);
   `find_package(marine_nav_interfaces REQUIRED)` + add to the executable's
   `ament_target_dependencies`; `<depend>marine_nav_interfaces</depend>` in
   `package.xml`.

## Files to Change

| File | Change |
|------|--------|
| `src/camp/running_tasks/running_tasks_model.{h,cpp}` | New tree model |
| `src/camp/running_tasks/running_tasks_view.{h,cpp}` | New node-injection widget |
| `src/camp/platform_manager/platform.ui` | Promote `RunningTasksView` in splitter |
| `src/camp/platform_manager/platform.cpp` | `setNode` + `updateRobotNamespace` |
| `CMakeLists.txt` | SOURCES + `marine_nav_interfaces` dep |
| `package.xml` | `<depend>marine_nav_interfaces</depend>` |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Thread safety | ROS callback never touches Qt; marshals via queued invoke + mutex, mirroring `MissionManager`. |
| Dual-use (no compromise) | Node-injection core + guarded Realtime group → CAMP gets the curated group, rqt (later) falls back to default. No `ROSWidget` coupling. |
| Additive | Placeholder `MissionManager` stays; this is a sibling view. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| (camp repo ADRs, if any) | TBD | Scan during implementation; this is additive UI, no message/contract changes. |

## Consequences

| If we change... | Also update... | In plan? |
|---|---|---|
| Add `marine_nav_interfaces` use | `package.xml` + CMake find_package + deps | Yes |
| New per-robot widget | `platform.ui` + `platform.cpp` node/namespace wiring | Yes |

## Open Questions

- Status column rendering: P1 shows the raw `status` YAML string (truncated). Full
  parse/formatting deferred to P4 (mission progress).

## Estimated Scope

Single PR (new widget + model + wiring). Builds on the camp executable; rqt
wrapper and map linkage are separate later phases under #123.
