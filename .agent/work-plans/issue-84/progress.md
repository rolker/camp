---
issue: 84
---

# Issue #84 — Layers tab: drag-drop layer reorder collapses to the first spot

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-10 06:46 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: approved

**Branch**: feature/issue-84 at `62e2316`
**Mode**: pre-push
**Depth**: Light (reason: small, low-risk — one flag removal + tests)
**Must-fix**: 0 | **Suggestions**: 3 (all addressed before push)

### Findings
- [x] (suggestion) `LayerListAcceptsDrops` could false-pass on an invalid index — added `ASSERT_TRUE(...isValid())` — `test/test_map_model.cpp`
- [x] (suggestion) `DropReordersToTargetRow` implicitly depended on Map's 3 default tile layers — added an initial-order assertion — `test/test_map_model.cpp`
- [x] (suggestion) `layer.cpp` comment implied nested-child layers don't exist yet (they do: markers/grids) — reworded — `src/camp2/map/layer.cpp`
- [ ] (note) No test drives the view-side OnItem→Above/Below conversion that is the literal bug (Qt QTreeView behavior; not unit-testable) — manual sim confirm requested in PR #87

Two independent adversarial reviewers (fresh-context Claude + Copilot CLI) confirmed the fix is sound with no must-fix issues.

## Integrated Review
**Status**: complete
**When**: 2026-06-10 07:20 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**PR**: #87 at `2e47f65`
**Sources**: 2 (Copilot R1 @ `62e2316`, Local Review (Pre-Push) @ `62e2316`)
**Cross-source confirmations**: 0
**CI**: none configured (camp has no CI)

### Findings
- [ ] (low, Copilot) `WarningTrap` is constructed after `QAbstractItemModelTester`, so the tester's constructor-time validation warnings go untrapped — swap so `WarningTrap` precedes the tester (applies to all four pairs: lines 77–78, 94–95, 121–122, 251–252) — `test/test_map_model.cpp`

### False positives
- (none)
