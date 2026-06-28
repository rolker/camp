---
issue: 127
---

# Issue #127 — CAMP running-task view P1: read-only structured task tree

## Plan Authored
**Status**: complete
**When**: 2026-06-28 09:26 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-127/plan.md` at `2a1f7ce`
**Branch**: feature/issue-127 at `2a1f7ce`
**Phases**: single (P1 of camp#123)

### Open questions
- [ ] Status column shows raw status YAML first line (truncated) in P1; full parse deferred to P4.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-28 09:33 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: approved (must-fix addressed before push)

**Branch**: feature/issue-127 at `e96eb34` (impl) — review fixes committed on top
**Mode**: pre-push
**Depth**: Deep (reason: custom QAbstractItemModel + ROS→GUI thread marshaling)
**Must-fix**: 1 (fixed) | **Suggestions**: 3 (2 fixed, 1 deferred)
**Round**: 1 | **Ship**: recommended — must-fix fixed + verified by rebuild

### Findings
- [x] (must-fix) ~RunningTasksView=default → UAF: pending_mutex_/rows_ destroyed before subscription_; callback in flight at teardown derefs freed members. FIXED: explicit dtor resets subscription_ first — `running_tasks_view.cpp`
- [x] (suggestion) Selection-restore re-emitted taskSelected() every republish → P2 feedback-loop risk. FIXED: QSignalBlocker around the restore — `running_tasks_view.cpp`
- [x] (suggestion) current_task_ stored raw vs normalized node fullId → highlight could miss on stray-slash id. FIXED: normalize current_task_ in setTasks — `running_tasks_model.cpp`
- [ ] (suggestion, deferred to P2) Synthetic group rows emit taskSelected(groupId) for a non-task; guard when map glue lands — `running_tasks_view.cpp`

### Model correctness (Lens A, verified)
- index/parent/rowCount/columnCount mutually consistent; rowInParent stable (append-only per rebuild); reset ordering correct (view restores by string id, not stale QModelIndex); null-safe. Walked rows [survey_a/line_2, survey_a, survey_a/line_1] — round-trips.
