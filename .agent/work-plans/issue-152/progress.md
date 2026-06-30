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

## Plan Review
**Status**: complete
**When**: 2026-06-30 15:15 +00:00
**By**: Claude Code Agent (Claude Opus)
<!-- Independent: the plan was authored by a separate fresh-context Sonnet dispatch;
this review is a separate Opus dispatch. The shared workspace identity "Claude Code Agent"
matches by name for every agent, so the mechanical self-review check does not apply here. -->

**Plan**: `.agent/work-plans/issue-152/plan.md` at `40d4749`
**PR**: PR-less (`gh` unauthenticated in this environment; reviewed local plan file)
**Verdict**: approve-with-suggestions

### Findings
- [ ] (must-fix) `test_vector_dataset_cleanup` has no library to link — `vectordataset.cpp`/`group.cpp`/`autonomousvehicleproject.cpp` compile only into the `CCOMAutonomousMissionPlanner` executable, and `open()` drags the full QGraphics item graph + `AutonomousVehicleProject` (mission_manager, platform_manager, map/raster). Settle the build strategy (bounded source list vs. valgrind-only descope) before implementing — `plan.md:26,36,37`
- [ ] (must-fix) In `VectorDataset::open`, destroy the per-layer `unprojectTransformation` at end-of-layer (else all but the last layer leak), and destroy the interior-ring `OGRPointIterator` inside the ring loop (`:119`, reassigned per ring) — `plan.md:34`
- [ ] (suggestion) Automated tests cover only the GDAL-dataset leak; `GetOpenDatasets()` does not see the `OGRCoordinateTransformation` handles that are the primary ~180 KB `Georeferenced` leak (valgrind-only). The `DepthRaster` test guards the pre-existing `GDALClose`, not this PR's dtor. State this in test comments so the names don't mislead — `plan.md:25,26,45`
- [ ] (suggestion) Rule of three: `= delete` copy ctor/assignment on `Georeferenced` when adding the owning dtor (latent double-free if ever copied; currently heap-only so no live bug) — `plan.md:23,32`

### Notes
- Source fixes verified against actual code: all five leak sites confirmed real and correctly located (`georeferenced.cpp:48-49`; `vectordataset.cpp:22,35,72,100,119`). Approach correctly reuses the `gdal_closer` RAII pattern (`raster_layer.cpp:176`) and the `GetOpenDatasets()` baseline-delta test pattern (`test_raster_layer_gdal_cleanup.cpp`).
- `virtual ~Georeferenced()` is not strictly required (no call site deletes through `Georeferenced*`) but is a sound defensive choice; review-issue already tracks the PR rationale.
