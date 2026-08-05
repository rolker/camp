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

## Plan Review
**Status**: complete
**When**: 2026-08-05 18:19 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-168/plan.md` at `0f81a67`
**PR**: PR-less (dispatched via review-plan skill on the local worktree; `gh` unauthenticated in this environment)
**Verdict**: approve-with-suggestions

### Findings
- [ ] (suggestion) #168 test validates the Qt `destroyed`-signal pattern via a *local* mock `std::set` + its own lambda — it does NOT exercise `SonarLiveCacheManager::sources_` nor the manager's real `connect(...)` wiring. Deleting the manager's `connect(layer, &QObject::destroyed, …)` line would not fail this test (~zero regression protection for the actual fix). Prior review's must-fix option (a) was "public accessor + test seam"; extension chose the weaker mock. Consider a thin white-box accessor (`bool isSourceTracked(base) const`) so the test asserts against the manager's set after a real remove. Defensible tradeoff — `updateTopics()` requires a live `Node` ancestor + real publishers for a true end-to-end respawn test. — `plan.md:49-58`
- [ ] (suggestion) #170 cap: `msg.width`/`msg.height` are `uint16` (confirmed in `SonarVisualizationTile.msg`), so `msg.width <= 0` is effectively `== 0` (harmless, promotes to signed int). Residual: even at the 4096 cap, `4096×4096×64 bands×4B ≈ 4 GB` remains a large single allocation. Cap correctly eliminates the unbounded uint16-max crash path; a combined `width*height*bands` ceiling would harden further if cheap. Not blocking. — `plan.md:146-157`

### Notes
- Root causes and fixes verified correct and idiomatic against source. #168: `<map>` include (manager.h:6) and `sources_[base]` guard (manager.cpp:64,70) confirmed; `new SonarLiveCacheLayer(...)` return currently discarded — impl must capture the pointer for the `connect`. #169: catalog subscription is created once in the ctor and never torn down, so re-enable does NOT re-deliver the transient-local latched catalog — the buffered replay is exactly the needed reconcile trigger; `enableLiveCoverage` order (warmLoad→subscribeTiles→writeSettings, cpp:235-238) means `request_pub_` is live when the replay's `publishRequest` fires. #170: `kMaxImageEdge=4096` (layer.h:221) and the pre-construction validation site in `handleTile` (cpp:358-359) confirmed; node-boundary cap is the correct location.
- Scope: 3 issues / 1 PR / 3 atomic commits per explicit user decision (2026-08-05). Upper-bound but cohesive — all three are the same `live_coverage` field-incident subsystem. ~10 lines production across `sonar_live_cache_manager.{h,cpp}` + `sonar_live_cache_layer.{h,cpp}` plus 3 new headless test files + CMakeLists blocks.
- ADRs referenced (0001, 0006, 0010) all exist under `docs/decisions/`. Headless test harness (offscreen QApplication + Map + null Node) confirmed in `test_sonar_live_eviction.cpp` — supports the #169/#170 layer-level test plans; `residentTileCount()` is public (layer.h:85).
- `review-issue` was not run for #168/#169/#170 (no `## Issue Review` entry / no comment) — noted, not penalized (optional step).
- **Independence**: fresh-context, independent review (host-dispatched Opus sub-agent) of a Sonnet-authored plan. The skill's self-review heuristic (match on `**By**` agent-name prefix) false-positives here because all Claude Code agents share the `Claude Code Agent` identity string; the differing model line (Opus vs Sonnet) is the true independence signal. No self-review annotation applied.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-05 19:01 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-168 at `ee21fd0`
**Mode**: pre-push
**Depth**: Deep (reason: field-incident crash-path fix + Qt object-lifecycle/concurrency in cross-cutting live_coverage subsystem)
**Must-fix**: 0 | **Suggestions**: 4
**Round**: 1 | **Ship**: recommended — no must-fix; two independent adversarial passes confirmed all three fixes correct

### Findings
- [ ] (suggestion) #170 ingest cap reuses render-clamp constant kMaxImageEdge (4096) as tile-ingest ceiling; edge>4096 producer would hit a silent reject/re-request loop — consider kMaxIngestEdge or a documenting comment — `src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp:363`
- [ ] (suggestion) Catalog-resume test comment misstates prune mechanism: tiles seeded at reconciler version 0 (markHave(index,0)), not stamp.sec=100; the 100-vs-200 margin doesn't exist — `test/test_sonar_live_catalog_resume.cpp:137`
- [ ] (suggestion) enableLiveCoverage replay re-requests the full held set every enable (v0 seeding); rapid disable/enable emits a full-catalog TileRequest burst per cycle (by-design ADR-0006 D3) — worth a one-line note — `src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp:243`
- [ ] (suggestion) #170 byte ceiling multiplies by msg.bands.size() but applyPatch allocates per unique band name; duplicate-named bands over-counted (conservative, safe direction) — `src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp:361`
- [ ] (note) C++ test suite not built/run locally (offline; heavy ROS/Qt build) — CI colcon+gtest is the gate. Static analysis: cppcheck aborts on Qt slots macro; cmake-lint/yamllint enforced by pre-commit. Local/Copilot adversarial unavailable (no Ollama, off by default).

## Integrated Review
**Status**: complete
**When**: 2026-08-05 15:55 -04:00
**By**: Claude Code Agent (Claude Fable 5)

**PR**: #185 at `16e6e53`
**Sources**: 3 (Copilot R1 APPROVED @ `16e6e53`, Local Review (Pre-Push) R1, CI rollup)
**Cross-source confirmations**: 0
**CI**: all-pass

### Findings
- [ ] (trivial, Copilot ×2 same root) `makeManager()` helper EXPECTs non-null `topLevelLayers()` then dereferences anyway (non-void helper can't ASSERT); null would segfault instead of failing cleanly. Guard-return nullptr in the helper + `ASSERT_NE(manager, nullptr)` at both call sites — `test/test_sonar_live_cache_manager_respawn.cpp:44,57,78`

### False positives
- (none — Copilot approved; the pre-push review's 4 suggestions were already applied at this head)
