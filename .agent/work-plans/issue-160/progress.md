---
issue: 160
---

# Issue #160 — Consumer-side overview pyramid + bounded eviction for live sonar coverage

## Issue Review
**Status**: complete
**When**: 2026-07-01 00:50 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #160
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Actions
- [ ] Record eviction policy + pyramid design decisions via camp ADR-0006 addendum (ADR-0012 carve-out) or new camp ADR — the eviction trigger metric (distance-from-vessel + byte budget vs. tile-count), fold timing (fold-on-evict vs. fold-on-arrival), and overview chain depth are open sub-decisions that ADR-0001 requires to be captured before or during implementation.
- [ ] Clarify reconciler interaction on LRU-evict: does the evicted tile stay in the reconciler's `tiles_` map (as disk-backed) or get `drop()`d? This affects whether camp re-requests tiles from the boat after LRU eviction — plan-task should settle this and record it.
- [ ] Plan multi-repo worktree: changes span `camp` (eviction + pyramid + LOD) and `unh_marine_autonomy` (GGGS `parent(GridIndex)` helper); use `--packages camp,unh_marine_autonomy` or sequence a GGGS PR first and a camp PR consuming it — plan-task should decide.
- [ ] Threading: verify eviction + overview fold happen on the GUI thread (ADR-0006 D4 invariant: reconciler and tile map are not thread-safe).

### Orchestrator note (checkpoint 1 resolved by operator, 2026-07-01)
- **GGGS `parent()` helper: ADD IT NOW (cross-repo).** Operator chose the dedicated
  `parent(GridIndex)`/`children()` primitive in `unh_marine_autonomy/gggs` (handling the
  polar 1/3/9 column scaling) over a camp-local geographic round-trip, so the boat can reuse
  it later for producer-side overviews. **plan-task**: plan the multi-repo coordination —
  recommended sequencing is a small standalone GGGS-helper PR (own issue in
  `unh_marine_autonomy`, `Part of rolker/camp#160`) merged + underlay rebuilt first, then the
  camp PR consuming it. Settle the remaining open sub-decisions (eviction trigger metric,
  fold timing, overview chain depth, reconciler-on-evict semantics) with the issue's stated
  leanings (distance-from-vessel + byte budget; full overview chain) and record them in the
  ADR-0006 addendum / new camp ADR.
