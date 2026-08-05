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
