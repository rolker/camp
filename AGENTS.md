# AGENTS.md — camp

Instructions for AI agents working in this repository — including **GitHub
Copilot code review**, which reads this file when reviewing PRs. Coding
agents: the deep guide (packages, layout, pitfalls) is
[`.agents/README.md`](.agents/README.md); read it before making changes.

## Workspace Rules

This repo is developed inside a [ROS 2 Agent Workspace](https://github.com/rolker/ros2_agent_workspace).
The workspace root `AGENTS.md` carries the full shared rules (worktree
isolation, issue-first policy, commit conventions, AI signatures). This file
**references** those rules and adds repo-specific context only — it must
never restate or fork them.

## Quality Standard

<!-- Standalone excerpt of the workspace AGENTS.md § Quality Standard. It is
     intentionally condensed for repos reviewed without workspace context; when
     the workspace § Quality Standard changes materially, re-sync this excerpt
     (the drift ADR-0017 acknowledges). -->

This is software for autonomous robot boats operating on open water.
Robustness is not optional.

- Fix bugs completely: add the test, handle the edge case, check the
  lifecycle transition.
- Concerns about error handling, silent failures, stale data, or missing
  validation are not nits — flag them unless the failure mode genuinely
  cannot occur. "Config is under our control" and "pathological input" are
  not blanket dismissals; field configs change under pressure.
- A change includes its consequences: tests, documentation, and dependent
  references update in the same PR.

## Reviewing PRs

- If the PR carries a work plan (`.agent/work-plans/issue-<N>/plan.md` or a
  plan in the PR body), the plan is kept **in sync with the implementation
  as it evolves** — an implementation that matches the current plan text is
  not "plan drift", even if the plan changed after the PR opened.
- Verify claims against source: parameters, topics, services, and message
  types in docs must match the code.

## Review Context — camp

- CAMP is the operator-station GUI (Qt5 + rclcpp) for planning and
  monitoring autonomous marine-vehicle missions. It is the operator's
  window on a live boat: silently stale displays or dropped mission
  commands are safety problems, not cosmetic ones.
- One package, three targets: the deployed `CCOMAutonomousMissionPlanner`
  executable (`src/camp/`) plus the `camp_map` / `camp_map_ros` shared
  libraries (`src/camp_map/`). The old `camp2` executable is retired.
- Long-running GUI process — resource lifecycle is a known risk area.
  Watch cleanup paths for GDAL datasets, OpenGL objects, timers, and
  background threads; destructors and disconnect paths deserve review.
- Tests are gtest and force `QT_QPA_PLATFORM=offscreen` themselves;
  new GUI-touching tests must stay headless-safe. Default branch: `jazzy`.
