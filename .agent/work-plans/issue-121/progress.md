---
issue: 121
---

# Issue #121 — Unified raster render abstraction + CAMP live tile cache (anti-entropy)

## Issue Review
**Status**: complete
**When**: 2026-06-28 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #121
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: needs-splitting

### Actions
- [ ] Split into two PRs: Part A (RasterFieldSource interface + adapters) and Part B (live tile cache) — Part B depends on #230 (anti-entropy transport) and #44/#68 (discover/opt-in); verify those are merged before starting Part B.
- [ ] Verify that the `RasterFieldSource` interface doesn't merely rename existing `GggsTileLayer` internals — the abstraction should emerge from needing a second implementation (live cache), not be introduced speculatively ahead of it. Consider whether the interface belongs in the same PR as Part A or only in Part B's PR.
- [ ] Write a camp project ADR for Part B's persistence contract: in-memory render + write-through to disk + anti-entropy reconcile + "display-grade preview vs. data of record" distinction. This is a significant design decision for the project.
- [ ] Clarify how the live cache layer presents to the operator: does it appear in the Layers tree? How is its "live/preview" status visually distinguished? Align with ADR-0005's browse/compose split.
- [ ] For Part B acceptance: add a test or documented scenario for the simulated-downtime gap (request + prune) that can run without a live boat connection.
