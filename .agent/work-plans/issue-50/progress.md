---
issue: 50
---

# Issue #50 — Field import: camp (2026-05-01)

## External Review
**Status**: complete
**When**: 2026-05-18 19:10
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #51 — 0 formal reviews, 1 conversation comment (self-review, multi-specialist).
2 valid suggestions + 1 deferred + 0 false positives.
**CI**: no checks configured on camp.

### Actions
- [x] **Code:** Switch all 7 NavSource subscriptions from
  `.reliability_best_available()` to `.reliable()` to sidestep the
  same rmw_zenoh 0.2.9 keyexpr bug `udp_bridge#16` worked around on
  the publisher side (commit `3c18b09`). UX choice ratified by user
  ("Switch all 7 sites to `.reliable()` now").
- [x] **PR body:** Replace the inverted-QoS-rule "Why" paragraph
  with the commit-message framing (defensive: subscriber matches
  workspace publisher's RELIABLE default; both ends avoid the
  BEST_AVAILABLE keyexpr bug). Updated via `gh api PATCH` —
  `gh pr edit --body-file` was the silent classic-Projects no-op
  (see `reference_gh_pr_edit_workaround.md`).
- [ ] **Deferred** (file as follow-up issue if it turns out load-bearing):
  basic NavSource subscription-binding test. Author already
  acknowledged tests aren't load-bearing for this mechanical
  defensive change.
- [ ] **Field check** (post-merge, on boat / operator bench):
  `ros2 topic info <bridged_nav_topic> --verbose` should now show
  CAMP as a subscriber. If not, the keyexpr-bug hypothesis was wrong
  and the QoS choice needs to be re-evaluated.
