---
issue: 130
---

# Issue #130 — Running-task view polish: ordering + done-to-bottom, staleness background, retire Heartbeat text

## Plan Authored
**Status**: complete
**When**: 2026-06-28 10:17 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-130/plan.md` at `1dedeee`
**Branch**: feature/issue-130 at `1dedeee`
**Phases**: single

### Open questions
- [ ] "Order by how they'll run" taken as message array order (boat task_order_ids), not a priority re-sort — flag in review if priority sort intended.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-28 10:45 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: approved (suggestions addressed)

**Branch**: feature/issue-130 at `56b74be` (+ review-fix commit)
**Mode**: pre-push
**Depth**: Standard (reason: model data-source refactor onto marine_nav_tasks::TaskList + cross-package integration)
**Must-fix**: 0 | **Suggestions**: 3 (2 fixed, 1 flagged)
**Round**: 1 | **Ship**: recommended — no must-fix; model correctness + threading + lifetime walked clean

### Findings
- [x] (suggestion) Current-task highlight compared raw fullId vs normalized current_task_ → could miss on stray-slash id. FIXED: normalize both sides in data() — `running_tasks_model.cpp`
- [x] (suggestion) Dtor comment referenced removed pending_rows_ member. FIXED → pending_tasks_ — `running_tasks_view.cpp`
- [ ] (suggestion, flagged to operator — pre-existing, out of scope) MissionManager has orphan auto-connect slots (on_gotoLine/startLine/nextMissionItemPushButton_clicked) for widgets absent from the .ui → Qt "No matching signal" runtime warnings. Also: "Next Mission Item" was context-menu-only and is now unreachable from the UI after removing the text browser — offer to add a Next button.

### Verified clean (walked)
- buildNodes + sortChildrenRecursive: rowInParent consistent with final order (re-indexed over sorted vector); 2-level done-task example round-trips index/parent/indexForId.
- TaskPtr lifetime: Nodes hold shared_ptr copies; old tree freed on root_ reassign; no dangling across cycles. beginResetModel/endResetModel brackets whole rebuild.
- Threading: callback only copies msg under mutex; TaskList touched GUI-thread-only. Clock fallback unreachable+harmless. Dtor resets subscription_ first (ordering holds post-refactor).
- MissionManager Heartbeat removal complete; no residual refs; kept buttons match slots.
