---
issue: 152
---

# Issue #152 — Fix GDAL/OGR handle leaks in Georeferenced / VectorDataset chart loading

## Issue Review
**Status**: complete
**When**: 2026-06-30 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #152
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Summary

Confirmed GDAL/OGR memory leaks in `camp` project repo, specifically:
1. `Georeferenced` — missing destructor; two `OGRCoordinateTransformation*` members allocated in `extractGeoreference()` (lines 48–49 of `georeferenced.cpp`) are never freed. Every `DepthRaster` and `VectorDataset` instance leaks ~180 KB of OGR/PROJ state.
2. `VectorDataset::open` — `GDALDataset*` from `GDALOpenEx` is never `GDALClose`d; per-layer `OGRCoordinateTransformation` is never destroyed; `OGRPointIterator*` leaked at all three geometry branches (linestring line 72, polygon exterior ring line 100, polygon interior ring line 119).

The leaks are well-evidenced (valgrind confirmed). The fix is clearly scoped to two files (`georeferenced.{h,cpp}` and `vector/vectordataset.cpp`) and is completable in a single PR.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Issue is well-documented with valgrind stacks; proposed fix is unambiguous |
| Enforcement over documentation | Watch | No automated memory-leak CI check exists; valgrind re-run is manual. Acceptable for this codebase, but worth noting |
| Capture decisions, not just implementations | Watch | Adding a `virtual` destructor to `Georeferenced` is a minor design decision (vtable impact); the rationale should appear in the PR description |
| A change includes its consequences | Action needed | Re-run valgrind after the fix and include output in the PR to prove the leak is closed; explicitly confirm `DepthRaster` has no direct GDAL handle members beyond those `Georeferenced` holds |
| Only what's needed | OK | Fix is minimal and targeted; no scope creep |
| Improve incrementally | OK | Small, reviewable single-PR change |
| Test what breaks | Watch | No automated regression for this leak; valgrind harness is manual. Acceptable given the project's test maturity |
| Workspace vs. project separation | OK | Correctly scoped to the `camp` project repo |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0002 — Worktree isolation | Yes | Worktree already exists (`issue-camp-152`) |
| ADR-0008 — ROS 2 conventions | Marginal | C++ source only; no package interface changes; follow existing RAII patterns in this package |
| ADR-0013 — progress.md vocabulary | Yes | This entry |

### Consequences

- `DepthRaster` inherits `Georeferenced` and will gain the virtual destructor. Confirmed it holds no direct GDAL handle members beyond those `Georeferenced` owns — no additional fix needed there.
- The `gdal_closer` lambda pattern (unique_ptr + custom deleter) is already established in `camp_map/raster/raster_layer.cpp:176–178`. The implementer should reuse or reference that pattern for `VectorDataset::open`'s `GDALDataset*` to keep the codebase consistent.

### Actions
- [ ] Re-run valgrind after the fix and include "definitely lost" output in the PR (proves the leak is closed from `Georeferenced::extractGeoreference` frames)
- [ ] Add virtual destructor to `Georeferenced`; include "why virtual" rationale in PR description (base class with polymorphic subclasses — deleting through base pointer would be UB without it)
- [ ] In `VectorDataset::open`: use the `gdal_closer` unique_ptr pattern (already in `raster_layer.cpp:176`) for the `GDALDataset*`; destroy the per-layer `OGRCoordinateTransformation`; call `OGRPointIterator::destroy(pi)` at all three iterator sites
- [ ] Confirm `DepthRaster` has no direct GDAL handle members that also need cleanup (quick grep shows it does not, but verify during implementation)

## Plan Authored
**Status**: complete
**When**: 2026-06-30 14:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-152/plan.md` at `40d4749`
**Branch**: feature/issue-152 at `40d4749`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.
