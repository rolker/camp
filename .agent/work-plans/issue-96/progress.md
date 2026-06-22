---
issue: 96
---

# Issue #96 — GDAL dataset leak in `loadAndReprojectFile`

## Issue Review
**Status**: complete
**When**: 2026-06-22 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #96
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Scope Assessment

The issue identifies a specific resource leak in `loadAndReprojectFile` in
`src/camp2/raster/raster_layer.cpp`: two GDAL dataset handles (`GDALOpen` result
and `GDALAutoCreateWarpedVRT` result) are opened but never closed, on every return
path including early-exit paths. The sibling `initExtent()` shows the correct
pattern and already uses `GDALClose`. The suggested fix — RAII `unique_ptr` with a
custom deleter — is idiomatic C++ and covers all return paths automatically. The
HOST NOTE confirms the leak is still present at HEAD (post-#102/#104 single-band
colormap work). Scope is minimal: fix the function, add a test.

The issue correctly distinguishes this from the zoom/pan OOM (#98) — these are
separate phenomena with separate causes.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| A change includes its consequences | Watch | Issue explicitly calls for a test. Implementation must include one — the pattern of GDAL resource mis-management could silently recur without a regression guard. |
| Only what's needed | OK | Fix is narrowly scoped: RAII wrappers in one function. No interface changes, no `.msg`/`.srv` changes. |
| Improve incrementally | OK | Small, self-contained bug fix. Does not require a companion redesign. |
| Test what breaks | Watch | GDAL dataset lifecycle is hard to assert at the unit-test level (no mock). A functional/integration test (load a raster N times, verify memory does not grow unboundedly, or verify `GDALOpen` call count with a GDAL callback) may be more practical than a pure unit test. Implementation should choose a feasible test strategy and document it if a simpler option is unavailable. |
| Human control and transparency | OK | No behavioral change to the user-visible layer — same pixels, same colormap. Fix is transparent to users. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0002 — Worktree isolation | Yes | Worktree `issue-camp-96` (branch `feature/issue-96`) already exists — OK. |
| ADR-0008 — ROS 2 conventions | Yes | C++ change in a ROS 2 package. RAII is consistent with ROS 2 / modern C++ style. No convention concerns. |
| ADR-0013 — progress.md vocabulary | Yes | This entry uses `## Issue Review` — correct. |

### Consequences

- Only `raster_layer.cpp` changes; no package interface changes, no `.msg`/`.srv` files, no callers affected.
- The early-return path `if(!reprojected_dataset) return result;` also leaks `dataset` — the RAII approach covers this automatically; the implementation must not miss it.
- Callers (`RasterLayer` ctor, `setColormap`, `readSettings`) do not need changes — the fix is internal to `loadAndReprojectFile`.

### Actions
- [ ] Implement RAII fix for both GDAL handles in `loadAndReprojectFile`, covering all return paths (including the `if(!reprojected_dataset)` early return that also leaks `dataset`).
- [ ] Add a regression test (automated or documented manual) for the resource leak.
