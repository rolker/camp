---
issue: 86
---

# Issue #86 — CAMP: crashes when deleting mission items

## Problem

CAMP crashed repeatably when deleting mission items (waypoint / line / task)
during 2026-06-09 student operations. The prior delete-crash fix #65 (`50ad308`,
2026-06-07) hardened the `AutonomousVehicleProject` selection/group bookkeeping
and the multi-select topmost-resolution, but a residual use-after-free survives.

## Root cause

The five **detail panels** in the right-hand pane each store a **raw pointer**
to the mission item they display, and that pointer is **never cleared when the
item is deleted**:

| Panel | Pointer | Guards on deref slots? |
|-------|---------|------------------------|
| `WaypointDetails` | `m_waypoint` | `onLocationChanged()` **unguarded** |
| `TrackLineDetails` | `m_trackLine` | `onTrackLineUpdated()` **unguarded**; pointer also uninitialized in ctor |
| `SurveyPatternDetails` | `m_surveyPattern` | `onSurveyPatternUpdated()` / `updateSurveyPattern()` **unguarded**; pointer also uninitialized in ctor |
| `BehaviorDetails` | `m_behavior` | `if(m_behavior)` — **useless**: dangling-but-non-null after delete |
| `OrbitDetails` | `orbit_` | `if(orbit_)` — **useless**: dangling-but-non-null after delete |

`DetailsView::onCurrentItemChanged()` hides the panel when the selection goes
invalid but does **not** clear these pointers (detailsview.cpp:138-142). When a
displayed item is deleted, the panel keeps a dangling pointer; the next
interaction with the (briefly still-focused) panel — a `editingFinished` /
`textChanged` / `stateChanged` slot, or a queued `…Updated` signal — dereferences
freed memory. The `if(ptr)` guards on Orbit/Behavior do not help because the raw
pointer is never set to null, so the guard tests a stale non-null value. This
matches the operator report of *several* crashes (interaction-timing dependent,
not every delete).

A secondary hazard: `AutonomousVehicleProject::deleteItem()` does a **synchronous
`delete item`** in the middle of the `beginRemoveRows … endRemoveRows` model
cascade (autonomousvehicleproject.cpp:862-865), rather than the safer
`deleteLater()` idiom the camp2 layer-delete path already uses (layer.cpp:54-64).

## Fix

1. **Detail-panel pointer lifetime (root-cause fix).** Change each panel's stored
   raw pointer to `QPointer<T>` (auto-nulls on `QObject` destruction —
   `MissionItem : QObject`). This makes the existing `if(ptr)` guards correct and
   the uninitialized-pointer cases safe (QPointer default-constructs to null).
   Add `if(!ptr) return;` guards to the unguarded deref slots
   (`onLocationChanged`, `onTrackLineUpdated`, `onSurveyPatternUpdated`,
   `updateSurveyPattern`).

2. **Deferred deletion (defense-in-depth).** In `deleteItem()`, complete the
   `beginRemoveRows / removeChildMissionItem / endRemoveRows` model removal with
   the item still alive, then `item->deleteLater()` — mirroring the camp2 layer
   pattern so connected slots unwind before the object dies.

## Verification

- Build `camp`.
- Sim repro: load/build a mission, select then delete each item type (waypoint,
  trackline with child waypoints, survey pattern, survey area, behavior, orbit,
  and a whole-mission multi-select) while its detail panel is shown. No crash.

## Testability gap (follow-up)

A CI `QAbstractItemModelTester` harness for the delete path (mirroring
`test/test_map_model.cpp` for the camp2 map model) is **blocked**: the mission
model (`AutonomousVehicleProject` + mission items) is compiled directly into the
`CCOMAutonomousMissionPlanner` executable and pulls in ROS deps via
`mission_manager.h` / `platform.h` — there is no library seam like `camp_map` to
link against. Extracting a ROS-free mission-model library is a separate,
larger refactor. File a follow-up issue.
