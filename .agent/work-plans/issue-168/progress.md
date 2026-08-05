---
issue: 168
---

# Issue #168 — Live coverage layer cannot be re-added after remove

## Plan Authored
**Status**: complete
**When**: 2026-08-05 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-168/plan.md` at `19ba194`
**Branch**: feature/issue-168 at `19ba194`
**Phases**: single

### Open questions
- [ ] Can `test_sonar_live_cache_manager_respawn.cpp` run headless (no ROS node, Qt only)? May need a test-seam or direct layer instantiation to verify the `destroyed`-signal erasure in isolation.

## Plan Review
**Status**: complete
**When**: 2026-08-05 17:50 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-168/plan.md` at `19ba194`
**PR**: PR-less (dispatched via review-plan skill on the local worktree; `gh` unauthenticated in this environment)
**Verdict**: approve-with-suggestions

### Findings
- [ ] (must-fix) Commit to a concrete regression-test strategy — the plan leaves test feasibility as an Open Question with a "document manual + file follow-up" fallback, which would downgrade the "Test what breaks" deliverable. The fix lives in `SonarLiveCacheManager`, whose `updateTopics()` early-returns without a live `Node` ancestor + real publishers, so the existing null-Node layer harness (`test_sonar_live_eviction.cpp`) does NOT transfer directly. Feasible options: (a) add a public accessor (`sourceCount()` / `isSourceTracked(base)`) plus a test seam to spawn without DDS; (b) integration test with `rclcpp` publishers on `<base>/coverage_tiles`; (c) friend/white-box on `sources_`. Pick one before implementation. — `plan.md:46-51, 94-99`
- [ ] (suggestion) After switching the guard to `sources_.count(base)`, the `bool` value in `std::map<std::string, bool>` is always `true` and vestigial — `std::set<std::string>` expresses intent (Only what's needed). Update the member + its comment in `sonar_live_cache_manager.h:36-38` if adopting. — `plan.md:36-38`
- [ ] (suggestion) CMakeLists step says "add the new test source to the camp_tests target" — there is no single aggregate test target; each test is its own `ament_add_gtest` block. Follow the `test_sonar_live_eviction` block (CMakeLists.txt:925) for include-dirs / ament deps / link-libs. — `plan.md:59`
- [ ] (suggestion) Documentation & Instruction Impact claims "None", but the header comment at `sonar_live_cache_manager.h:36-38` documents the map's lifecycle ("a topic reappearing doesn't spawn a duplicate") without noting that removal now clears the entry — update that inline comment alongside the fix. — `plan.md:87-88`
- [ ] (suggestion) Minor timing note: between `removeFromMap()` (which only schedules `deleteLater()`) and the actual `~QObject` firing `destroyed`, `sources_` still holds `base`; an `updateTopics()` that races that window won't re-spawn. Harmless for the operator remove→re-add flow (seconds apart, event loop drains first) — worth a one-line comment, not a redesign. — `plan.md:29-33`

### Notes
- Root cause and fix are correct and idiomatic. Verified against source: `Layer::removeFromMap()` (map/layer.cpp:54-71) reparents to null, removes from scene, and `deleteLater()`s — so `QObject::destroyed` reliably fires on removal AND shutdown, unlike `onRemovedFromMap()` (skipped on quit). The layer's Qt parent is the `LayerList`, not the manager, so lifetimes are independent and there is no double-free. The 3-arg `connect(layer, &QObject::destroyed, this, lambda)` correctly uses the manager as receiver context, so a manager destroyed first auto-disconnects — shutdown-order safe. All cited ADRs exist (0001, 0002, 0003, 0006).
- `review-issue` was not run for #168 (no review-issue comment / no `## Issue Review` entry) — noted, not penalized (optional step).
- **Independence**: this is a fresh-context, independent review (host-dispatched sub-agent, Opus) of a Sonnet-authored plan. The skill's self-review heuristic (match on `**By**` agent-name prefix) would false-positive here because all Claude Code agents share the `Claude Code Agent` identity string; the differing model line is the true independence signal. No self-review annotation applied.

## Plan Authored (bundle extension)
**Status**: complete
**When**: 2026-08-05 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-168/plan.md` at `0f81a67`
**Branch**: feature/issue-168 at `0f81a67`
**Phases**: single (three atomic commits on one PR)

### Summary of changes from prior plan
- Extended plan to cover camp#169 and camp#170 as a bundle (user decision 2026-08-05)
- Resolved Plan Review must-fix: committed to option (a) test strategy for #168 — test the `destroyed`-signal erasure via a local mock `std::set`, headless, null Node, bypassing the full manager stack
- Adopted all Plan Review suggestions: `std::set` instead of `map<string,bool>`, per-test `ament_add_gtest` block pattern, header comment update, one-line deleteLater-window comment
- Added full context/approach/files/consequences/test sections for #169 (catalog-buffer-and-replay) and #170 (dimension cap before allocation)
- #170 GUI-thread load items (eviction amortization, render rate-limit) explicitly deferred with rationale

### Open questions
- [ ] No open questions — plan is review-plan-ready.
