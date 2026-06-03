# Work Plan — Issue #61: CAMP user manual (operator-facing)

Issue: https://github.com/rolker/camp/issues/61

## Context & forcing function

Part of the Summer Hydro 2026 documentation push (driver: **June 4 dev-freeze /
class start**; audience = **rotating student-operator cohorts**). Companion to the
BizzyBoat operator manual (`rolker/unh_echoboats_project11#202`) and the framework
guide (`rolker/unh_marine_autonomy#132`, drafted as PR #133). Decisions agreed
with Roland 2026-06-01.

## Audience & scope decisions

- **Audience**: student operators at the operator station.
- **CAMP = the deployed `CCOMAutonomousMissionPlanner` binary**, NOT the unused
  `camp2` rewrite. Develop/document against `src/camp/` (RosContext / TopicBridge /
  GeoGraphicsItem arch; Markers is the overlay template).
- **Troubleshooting = link to the framework guide**, not a symptom→cure cookbook.
- No mission-specific (site/customer) detail.

## Outline

1. **Overview** — CAMP (`CCOMAutonomousMissionPlanner`) at the operator station;
   what it is and isn't.
2. **Starting & connecting** — launch; chart auto-load (13283 via `camp_launch.py`);
   confirming the boat link. A boat **advertises itself** on `/marine/platforms`
   and CAMP's `PlatformManager` auto-creates a tab + map drawing per platform —
   so "boat not showing up" → check `/marine/platforms` reaching the station and
   the boat's `platform_publisher` lifecycle node being activated (framework
   guide §3).
3. **Reading the display** — chart; boat pose; camera/sonar/segmentation views;
   **annunciators** (naming distinguishes diagnostic *source* — boat vs operator —
   not where the display lives; both run at the operator station); **heartbeat**.
4. **Planning a survey** — lay out lines; parameters.
5. **Sending & controlling missions** — send; **override** (Standby / Hover /
   Goto); **update-vs-replace** task semantics (mid-run plan modify = `replace_task`,
   which works; in-place `update_task` is unhandled / deferred to the #50 BT
   redesign — document current behavior, don't promise update_task).

## Source-of-truth references (verify against, don't assume)

- Platform self-advertisement: `ui_ws/src/camp/src/camp/platform_manager/`
  (`platform_manager.cpp` subscribes `/marine/platforms`) ↔ boat-side
  `unh_marine_autonomy/marine_autonomy/scripts/platform_send.py`.
- Command/override + task semantics: framework guide `docs/how_the_stack_works.md`
  in `unh_marine_autonomy` (PR #133) and the `marine/mission_manager/command`
  routing (command_bridge / camp_interface). Topic prefix is **`marine/`** (the
  `project11/` prefix is retired; PR #133 cleaned the autonomy docs).
- Bag-replay verification recipe (for testing CAMP behavior locally without a
  boat): `camp_launch.py` auto-loads chart 13283; `ros2 bag play --start-offset`
  is safe (earth→bizzy/map is dynamic `/tf`); no zenoh needed locally.
- Known display caveats to mention or work around:
  - Hover markers can be missing until CAMP restart (lazy viz publisher / FastDDS
    late-join; fixed by #48/PR#49 — note current state).
  - Windowed/operator costmap + imagery can read empty until a 2nd subscriber
    attaches (subscriber-gated lazy publish) — not a CAMP bug per se.

## Cross-links

- Framework guide: `rolker/unh_marine_autonomy#132` / PR #133 (link for the
  "how it works" / troubleshooting-by-understanding material).
- BizzyBoat operator manual: `rolker/unh_echoboats_project11#202`.

## Open items / notes

- ~~Decide doc location/format and link from the camp README~~ **DONE** —
  `docs/camp_user_manual.md`; added a new **Documentation** section to README
  (first README→docs link in this repo) listing the manual + architecture doc.
- ~~Verify the override/send UI flow against the actual CAMP widgets before
  writing §5~~ **DONE** — verified against `src/camp/`: Execute →
  `sendMissionPlan` → `replace_task` (`mission_manager.cpp:99`); map right-click
  overrides **"Hover Here"/"Goto Here"/"Idle in place"** (`projectview.cpp:347+`);
  helm **"Standby"/"Autonomous"** buttons (`helm_manager.cpp`).

### Implementation note — outline §3 corrected against source
The agreed outline listed "camera/sonar/segmentation views" under *Reading the
display*. **Source check shows these are NOT in CAMP**: no image/video widget is
built (grep for `Image`/`CompressedImage`/`setPixmap` in `src/camp` = zero); the
sonar panel is dead ROS1 code excluded from the build; annunciators run as
separate processes, not CAMP panels. The manual now states this explicitly and
redirects imagery to the separate rqt windows. Also confirmed: Execute sends
`replace_task` (works); the Mission→**Update** menu sends `update_task`, which is
live in CAMP but a no-op downstream (#50) — documented as "use Execute."

Annunciator + imagery coverage **moved out** of this manual to the BizzyBoat
operator manual (Roland's scope call — CAMP is not where they're viewed); §2 keeps
only a brief pointer. Added a CAMP-window example screenshot
(`docs/images/camp_window_2026-06-01.png`, cropped from the 2026-06-01 operator
capture; terminals excluded; Roland approved the content for this PUBLIC repo).

**Genericized 2026-06-02 (Roland):** removed Summer-Hydro / student framing so the
manual doesn't go stale; CAMP is platform-neutral — refs are "the boat" / "the
boat's operator manual" with BizzyBoat only as the explicit *See also* example.
Swapped the screenshot to a **June-2 mid-survey frame** (`camp_window_2026-06-02.png`,
UTC 18:18 / 14:18 EDT) showing the **working windowed costmap with detected
obstacles/targets** + survey-pattern tracklines (costmap confirmed at CAMP from
13:09 EDT per the salmon deployment log; #56/PR#61 costmap-over-bridge fix).

---
**Authored-By**: `Claude Code Agent`
**Model**: `Claude Opus 4.8 (1M context)`
