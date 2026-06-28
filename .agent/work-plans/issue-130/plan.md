# Plan: Running-task view polish (#130)

## Issue

https://github.com/rolker/camp/issues/130  (Part of #123)

## Context

First sim test of the P1 running-task view (#127) surfaced three polish items, all
in `camp`: order tasks by run order with done at the bottom; a HelmManager-style
green/yellow/red staleness background; and retiring the old Heartbeat text dump in
`MissionManager` (keeping its buttons).

## Approach

### 0. Adopt the shared task model (`marine_nav_tasks::TaskList` / `Task`)
Rather than hand-roll id-hierarchy parsing, ordering, done and YAML handling,
`RunningTasksModel` now wraps a `marine_nav_tasks::TaskList` (the same model the
boat uses). On each snapshot, `setTasks(current, tasks, clock)` calls
`task_list_.update(tasks, clock)` (which builds the tree from the flat list,
preserves message/run order, and creates/removes/reorders tasks), then snapshots
the `Task` tree into display `Node`s (each `Node` keeps the `TaskPtr` so P2 map
glue can read poses/markers). Adds `<depend>marine_nav_tasks</depend>` (ui→core).
Note: `TaskList` does **not** synthesize missing intermediate parents, so there
are no synthetic group rows — we render exactly the domain tree.

### 1. Done-to-bottom ordering (`running_tasks_model.cpp`)
Run order comes from `TaskList` (message order). On top of the snapshot, add a
stable partition **per tree level**: not-done nodes first (in run order), done
nodes last. `sortChildrenRecursive` sorts each node's `children` keyed on
`task->message().done` and re-assigns `rowInParent` so index/parent stay consistent.

### 2. Staleness background (`running_tasks_view.{h,cpp}`)
Mirror `HelmManager::watchdogUpdate`:
- `QTimer` at 500 ms → `watchdogUpdate()`.
- Track `last_message_time_` (rclcpp::Time), set on each applied `TaskFeedback`
  (GUI thread, in `applyPendingTasks`) from `node_->get_clock()->now()`.
- In `watchdogUpdate`, if a message has been received: `diff = now - last_message_time_`;
  `QPalette::Window` = green (<2 s) / yellow (<5 s) / red; `setAutoFillBackground(true)`.
- Thresholds `max_green_duration_` = 2 s, `max_yellow_duration_` = 5 s (members).
- Add a few px contents margin so the colored window frames the tree.
- Timer needs the node clock; guard until `node_` is set (start timer in `setNode`,
  like HelmManager).

### 3. Retire Heartbeat display in `MissionManager` (`mission_manager.{h,cpp,ui}`)
- `mission_manager.ui`: remove `missionStatusTextBrowser` (and its custom context
  menu policy). Keep the button row: Cancel Override, Clear, Restart.
- `mission_manager.{h,cpp}`: remove the `Heartbeat` subscription
  (`mission_status_subscription_`), `missionStatusCallback`, `updateMissionStatus`,
  and the `on_missionStatusTextBrowser_customContextMenuRequested` handler + the
  `marine_interfaces/Heartbeat` include. Keep `send_command_publisher_`, the button
  handlers, and `sendNextItem/sendHover/sendGoto/sendIdle/...` (used by projectview /
  autonomousvehicleproject).

## Files to Change

| File | Change |
|------|--------|
| `CMakeLists.txt`, `package.xml` | Add `marine_nav_tasks` dependency |
| `src/camp/running_tasks/running_tasks_model.{h,cpp}` | Wrap `marine_nav_tasks::TaskList`; Node holds `TaskPtr`; sort done-to-bottom; reindex `rowInParent` |
| `src/camp/running_tasks/running_tasks_view.{h,cpp}` | Staleness QTimer + palette coloring |
| `src/camp/mission_manager/mission_manager.ui` | Drop text browser, keep buttons |
| `src/camp/mission_manager/mission_manager.{h,cpp}` | Drop Heartbeat sub + callbacks |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| No regression | Removing the Heartbeat display is safe — the structured view replaces it; buttons + command paths used elsewhere are kept. |
| Consistency | Staleness reuses HelmManager's exact mechanism/colors/thresholds. |
| Model correctness | Re-sort must re-assign `rowInParent` so index()/parent()/indexForId stay consistent (verify with a tree walk). |

## Consequences

| If we change... | Also update... | In plan? |
|---|---|---|
| Remove MissionManager Heartbeat sub | Confirm no other consumer of `updateMissionStatus`/textBrowser | Yes (grep) |
| Sort model children | `rowInParent` reindex + idForIndex/indexForId | Yes |

## Open Questions

- "Order by how they'll be run" is taken as message array order (= boat
  `task_order_ids`, the run order), not a priority re-sort. Flag in review if a
  priority sort was intended instead.

## Estimated Scope

Single PR in `camp`.
