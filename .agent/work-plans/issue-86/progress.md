---
issue: 86
---

# Issue #86 — CAMP: crashes when deleting mission items

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-10 11:35 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: changes-requested → addressed

**Branch**: feature/issue-86 at `6acabc5`
**Mode**: pre-push
**Depth**: Deep (reason: use-after-free / lifecycle fix in a deployed safety-relevant GUI)
**Static analysis**: skipped (pre-commit/ament; clang LSP noise only — no Qt include paths)
**Copilot Adversarial**: run (2 cross-model passes)
**Must-fix**: 2 | **Suggestions**: 0

### Findings
- [x] (must-fix, Copilot) `MainWindow::setCurrent` dereferenced `itemFromIndex(index)` with no null check; `currentChanged` fires with an invalid index when the selected item is deleted → synchronous null-deref crash. Likely the primary #86 crash. Fixed: null-guard + clear fields — `src/camp/mainwindow.cpp:177`
- [x] (must-fix, Copilot) `deleteItem` used `pi`/`rownum` unguarded; `pi` null for orphaned parent and `rownum == -1` for already-detached item → `beginRemoveRows(p,-1,-1)` asserts (window widened by the new `deleteLater`). Fixed: `rownum < 0` guard + free without touching model rows — `src/camp/autonomousvehicleproject.cpp:860`

### Confirmed correct (Claude adversarial, independent)
- QPointer valid for all five panel types (`MissionItem : QObject`; QObject is first base in the multiple-inheritance chain).
- `delete`→`deleteLater()` introduces no new model-protocol inconsistency: `removeChildMissionItem` is synchronous, so `rowCount`/child indexing stay consistent; only the C++ free is deferred. Strictly safer for the `deleteItems()` topmost-ancestor walk.
- No remaining unguarded QPointer derefs in the five detail panels after the fix.

### Out-of-scope observations (pre-existing on jazzy, not fixed here)
- `SurveyPatternDetails::setSurveyPattern` / `TrackLineDetails::setTrackLine` reconnect their update signal without disconnecting the prior connection. Benign (connections die with the item); untouched.

### Testability gap (follow-up)
- No CI `QAbstractItemModelTester` harness for the delete path: the mission model is welded into the `CCOMAutonomousMissionPlanner` executable (ROS deps via `mission_manager.h`/`platform.h`), with no `camp_map`-style library seam. Extracting a ROS-free mission-model library is a separate refactor — file a follow-up.
