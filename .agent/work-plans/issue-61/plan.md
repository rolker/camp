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

- Decide doc location/format (e.g., `docs/camp_user_manual.md`) and link from the
  camp README.
- Verify the override/send UI flow against the actual CAMP widgets before writing
  §5 (read the relevant `src/camp/` UI code; don't document from memory).

---
**Authored-By**: `Claude Code Agent`
**Model**: `Claude Opus 4.8 (1M context)`
