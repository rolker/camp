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
