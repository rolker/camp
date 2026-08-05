# Plan: SonarLiveCacheManager never clears spawned-sources map on layer removal

## Issue

https://github.com/rolker/camp/issues/168

## Context

`SonarLiveCacheManager::updateTopics()` guards against duplicate-spawn using
`std::map<std::string, bool> sources_` (sonar_live_cache_manager.h:38). On each
DDS graph update it checks `if(sources_[base]) continue;` before spawning a
`SonarLiveCacheLayer` (sonar_live_cache_manager.cpp:64-65, 69-70). When the
operator removes the layer, `Layer::removeFromMap()` calls `deleteLater()` but
`sources_` is never updated — no `onRemovedFromMap()` override, no signal
connection, nothing. After removal every subsequent `updateTopics()` skips the
source permanently, making it impossible to re-add the live coverage layer within
a session without a full restart.

The correct fix is to clear `sources_[base]` when the spawned layer is
destroyed. The `QObject::destroyed` signal (fired at the start of the `QObject`
destructor, before any of the Qt child-management or signal/slot bookkeeping) is
the right hook: it fires whether the layer is removed by the user, by a
programmatic owner, or on app shutdown — unlike `onRemovedFromMap()`, which the
base-class `Layer::removeFromMap()` calls but which is skipped on normal app exit
(layer.cpp:59; camp#90).

## Approach

1. **Wire `destroyed` signal at spawn time** — In `updateTopics()`, after spawning
   the layer, connect its `QObject::destroyed` signal to a lambda (captured by the
   manager, with `Qt::ConnectionType::DirectConnection` — acceptable because the
   lambda only calls `std::map::erase`, which is fast and safe on the GUI thread)
   that erases the base entry from `sources_`.

2. **Switch guard to `count()` / `find()`** — The current `if(sources_[base])`
   uses `operator[]`, which inserts a default `false` entry for every unknown base
   encountered. Replace with `sources_.count(base)` (no side-effect) so the map
   stays clean (only live spawned bases present, not also every scanned base).

3. **Add a regression test** — New test `test_sonar_live_cache_manager_respawn.cpp`
   using the Qt + Map harness (QApplication, `camp::map::Map`, `camp::map::LayerList`)
   to exercise spawn → remove → spawn on the same source. The test creates a minimal
   `SonarLiveCacheManager` sub-object, calls `updateTopics()` via a testable seam,
   removes the spawned layer, and verifies a second `updateTopics()` call re-spawns.
   If the manager setup cost is too high without a ROS node, scope the test to verify
   only the `destroyed`-signal erasure in isolation (mock layer, check `sources_` via
   a test-friend or white-box inspection).

   **Open question**: whether a lightweight integration test (no ROS node, Qt only)
   is feasible without significant refactor of `SonarLiveCacheManager`'s constructor.
   If not, document the manual verification path and file a follow-up (#169-compat test).

## Files to Change

| File | Change |
|------|--------|
| `src/camp_map/ros/live_coverage/sonar_live_cache_manager.cpp` | In `updateTopics()`: (a) switch `if(sources_[base])` → `if(sources_.count(base))`, and (b) after `sources_[base] = true`, add `connect(layer, &QObject::destroyed, this, [this, base]() { sources_.erase(base); });` |
| `test/test_sonar_live_cache_manager_respawn.cpp` | New test: spawn a `SonarLiveCacheLayer` for a base, call `removeFromMap()`, verify `sources_` no longer contains the base (or verify a second spawn attempt succeeds) |
| `CMakeLists.txt` | Add the new test source to the camp_tests target (same pattern as other `test_sonar_live_*.cpp` entries) |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | The fix is 2-3 lines in one `.cpp`; no header change, no new abstraction |
| Improve incrementally | Single atomic fix; does not touch #169 (catalog latch) or #170 (tile dimension cap), which are separate issues |
| Test what breaks | The spawned-source guard is exactly the kind of in-session state machine that regression tests should cover |
| A change includes its consequences | The `CMakeLists.txt` update keeps CI green; the test catches future regressions |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0006 D5 | Yes | Fix preserves the opt-in gate — re-spawned layer still starts inactive and requires operator enable |
| ADR-0002 | Tangential | Layer destruction still goes through the standard `deleteLater()` path; the new connection fires during that destruction |
| ADR-0001 | Checked | The `destroyed` signal fires on the GUI thread (via `Qt::DirectConnection`; `deleteLater()` dequeues on the event loop); the lambda touches only `std::map::erase` — no ROS/thread concern |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `updateTopics()` spawn guard | `onRemovedFromMap()` in `SonarLiveCacheLayer` (not needed — destroyed signal is broader and already correct) | N/A — not required |
| spawn→remove→spawn path | Manual verification in the field (BizzyBoat / gabby) | No — integration test covers it in CI |

## Documentation & Instruction Impact

- **Stale docs**: None — the class-level Doxygen comment in sonar_live_cache_manager.h
  already says "deduped by base namespace"; no doc implies the dedup is permanent.
- **Agent-instruction candidates**: The `QObject::destroyed` pattern (connect at spawn,
  erase at destruction) is a clean solution for manager-tracks-children problems. Could be
  added as a note to `.agent/knowledge/` if similar managers emerge — but premature now
  with only one instance.

## Open Questions

- Can `test_sonar_live_cache_manager_respawn.cpp` run headless (no ROS node, Qt only)?
  The test needs to drive `updateTopics()` without a live `TopicsManager`; this may
  require either a test-seam method on `SonarLiveCacheManager` or direct construction
  of a `SonarLiveCacheLayer` to verify the `destroyed`-signal erasure in isolation.

## Estimated Scope

Single PR. The fix itself is 2-3 lines; the test is the main effort. Issues #169 and
#170 share the same field incident but are separate structural bugs addressed in
separate commits on the same PR branch (per user decision 2026-08-05).
