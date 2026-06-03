# CAMP User Manual

A guide for **student operators** running the operator station during Summer Hydro:
how to start CAMP, connect to the boat, read the display, plan a survey, and
send/control missions.

> **What CAMP is.** CAMP is the **CCOM Autonomous Mission Planner** (the deployed
> `CCOMAutonomousMissionPlanner` binary). It is the **map and mission-planning
> interface** at the operator station — you lay out survey lines on a chart, send
> them to the boat, watch the boat's position and status, and issue overrides.
>
> **What CAMP is not.** CAMP is not a camera/sonar viewer and not the autonomy
> stack itself. Live imagery (camera / segmentation) is shown in **separate windows
> (rqt)**, not in CAMP. The diagnostic **annunciators** also run as separate
> processes at the operator station — they are not CAMP panels. For *how the
> autonomy stack works*, see the **marine-autonomy framework guide**
> (`unh_marine_autonomy`, `docs/how_the_stack_works.md`); for the **boat side** of
> operations, see the **BizzyBoat operator manual**
> (`unh_echoboats_project11`, `docs/bizzyboat_operator_manual.md`).

---

## 1. Starting & connecting

Start CAMP from the operator station:

```
ros2 launch camp camp_launch.py
```

- CAMP **auto-loads chart 13283** as the background (the `background_chart` launch
  argument defaults to it). To use a different chart, pass
  `background_chart:=<path-to-.KAP>`.
- In normal Summer Hydro operation CAMP comes up as part of the operator launcher
  (see the BizzyBoat operator manual, *Starting the stack*) — you don't usually run
  it by hand.

### Connecting to the boat ("why isn't the boat showing up?")

The boat **advertises itself** to the operator station. When CAMP hears a platform
on the `/marine/platforms` topic, it **automatically creates a tab for that boat**
(and draws it on the map). You don't connect manually.

So if **BizzyBoat doesn't appear**, nothing is reaching CAMP on `/marine/platforms`.
Check, in order:

1. The **comms link** to the boat is up (see the operator manual, *Comms & range*).
2. The **boat's stack is actually running** on gabby (it publishes the platform).

When the link and boat are healthy, the BizzyBoat tab appears on its own.

---

## 2. Reading the display

CAMP is a **chart (map) view** with overlays, plus a **per-boat tab** on the side
for that boat's status and controls.

### The map

- The center/right is the **chart view** — the nautical chart (13283) with the boat
  drawn on it. The boat shows as a ship outline / track with its **speed over ground
  (SOG)**, and its currently followed path is drawn as it runs a mission.

### The boat's tab (status)

Each boat gets a tab containing:

- **Heartbeat panel** — the boat sends a **heartbeat**; this panel shows
  **"Last HB"** and **latency**, and is colored **green / yellow / red** by how long
  since the last heartbeat. Green = healthy link and the command is being applied.
  (Thresholds are adjustable via the panel's config button.)
- **Mission status** — key/value fields from the boat's mission manager (what task
  it's on, etc.).
- The **Standby / Autonomous** control buttons (see [§5](#5-sending--controlling-missions)).

### Map overlays (optional, via the toolbar managers)

- **Grids / costmaps** — the *Grid Manager* auto-discovers occupancy-grid / grid-map
  topics and draws them (e.g. the navigation costmap). Note: a costmap may read empty
  until a second subscriber attaches — a known lazy-publish quirk, not a CAMP fault.
- **Collision Monitor** — draws the boat's reflex/collision-zone polygons on the map.
- **Markers** and **AIS** — generic marker overlays and AIS contacts.

> **Not in CAMP:** camera, sonar, and segmentation imagery (use the separate rqt
> image windows), and the boat/operator **annunciators** (separate processes). Don't
> look for these as CAMP panels.

---

## 3. Planning a survey

Add mission items to the map using the **Add tools** (toolbar / menu actions), then
adjust their parameters. You can also **right-click the map** or **right-click the
mission tree** to add items.

Available item types include **Waypoint**, **Trackline**, **Survey Pattern**,
**Survey Area**, **Search Pattern**, and **Orbit**.

### Laying out survey lines (Survey Pattern)

Choose **Survey Pattern**, then click on the map to place it. A Survey Pattern has
these parameters (in its detail panel):

- **Line length**
- **Total Width**
- **Line spacing**
- **First line heading**
- **Alignment** — start / center / finish
- **Start Point** / **Opposite Point**

This generates the parallel survey lines (tracklines) for the area.

### Per-item parameters

Select any item to edit, in the details panel / top fields:

- **Speed (Knots)** and **Throttle (%)**
- **Priority**
- Task data

> The full mission JSON format (item types and their fields — SurveyPattern,
> TrackLine, Waypoint, etc.) is documented in the repository
> [README](../README.md#json-base-format-for-sending-missions-to-a-robot-and-saving-a-project-to-file).

---

## 4. Sending a mission

1. Select the mission item (or group) you want to run.
2. Click **Execute** (in the item's detail panel).

**Execute sends the plan as a `replace_task`** — this is the working path for
sending *and* for modifying the plan mid-run (it replaces the current task with what
you executed).

Other mission actions (right-click the mission tree → **Mission** submenu, or the
control buttons):

| Action | What it does |
|--------|--------------|
| **Execute** | Send the selected plan (replaces the running task). The normal "go" button. |
| **Append** / **Prepend** | Add the plan after / before the current task. |
| **Clear** | Clear all tasks. |
| **Restart** | Restart the current mission. |
| **Next** | Advance to the next mission item. |
| **Cancel Override** | Cancel an active Hover/Goto/Idle override and resume the mission. |

> **Heads-up — the "Update" menu item is a no-op right now.** CAMP has a Mission →
> **Update** action that sends `update_task`, but the boat does not currently handle
> it end-to-end (deferred to the behavior-tree redesign). **To change a running
> plan, use Execute** (which sends `replace_task`). Don't rely on Update.

> **Mid-line re-send caveat:** re-sending a *different* line while the boat is
> partway along one may not take effect immediately (the boat latches the path it's
> following). If a switch doesn't take, **Clear** and re-send. This is boat-side
> behavior, documented in the framework guide.

---

## 5. Controlling the boat (overrides)

### Standby / Autonomous (the boat's tab)

- **Standby** — request standby. This hands the boat to **manual (RC)** control
  **immediately** (the instant-handoff safety path). Expect the boat to drift after
  Standby — that's normal, it's no longer holding position.
- **Autonomous** — put the boat back under autonomous control.

The active mode's button highlights (green for the active autonomous/standby mode,
blue when the boat is in manual).

### Hover / Goto / Idle (right-click the map)

Right-click a point on the chart for the override menu:

- **Hover Here** — go to that point and hold position (autonomous).
- **Goto Here** — drive to that point (autonomous).
- **Idle in place** — stop and idle where it is.

Use **Cancel Override** (above) to drop the override and resume the planned mission.

> A re-commanded hover currently reuses the *original* hover spot in some cases
> (a known boat-side latch); if you need a new hover point, cancel and re-issue.

---

## See also

- **BizzyBoat operator manual** (`unh_echoboats_project11`,
  `docs/bizzyboat_operator_manual.md`) — the boat side: bring-up, readiness,
  driving, shutdown.
- **Marine-autonomy framework guide** (`unh_marine_autonomy`,
  `docs/how_the_stack_works.md`) — how the autonomy stack works; the place to build
  understanding when behavior is surprising.
- [CAMP software architecture](architecture_design.md).
