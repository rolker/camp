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

## Plan Authored
**Status**: complete
**When**: 2026-06-22 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-96/plan.md` at `c327fe2`
**Branch**: feature/issue-96 at `c327fe2`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-06-22 02:45 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-96/plan.md` at `c327fe2`
**PR**: PR-less (reviewed via worktree; `gh` unauthenticated in this environment)
**Verdict**: approve-with-suggestions

Independence note: the `## Plan Authored` entry shares the workspace agent name
("Claude Code Agent") but was a separate Sonnet sub-agent invocation; this review
is a fresh-context Opus sub-agent, so it is genuinely independent — the
`(in-context — author self-review)` annotation is omitted as it would be inaccurate.

### Findings
- [ ] (suggestion) Assert load success, not just zero open handles — if the synthetic raster fails to load/reproject, every early-return path leaks nothing, so a `GetOpenDatasets()==0` assertion passes vacuously even if `loadAndReprojectFile` never reached the warp path. Add an assertion that the load produced a non-empty image/mipmaps (e.g. via `imageReady`/`boundingRect`) so the test is a meaningful regression guard. — `plan.md:34`
- [ ] (suggestion) Prefer a baseline-delta over `EXPECT_EQ(n, 0)` — `GDALDataset::GetOpenDatasets()` counts *all* process-wide open handles, not just the layer's. Capture the count before constructing `RasterLayer` and assert equality after destruction, so the test won't flake if `Map`/GDAL internals hold an unrelated handle. — `plan.md:39`
- [ ] (suggestion) Preserve declaration order to keep close order = reprojected-then-source — `initExtent()` deliberately `GDALClose(reprojected)` before `GDALClose(dataset)` (the warped VRT references the source). Local `unique_ptr`s destruct in reverse declaration order, so keep `dataset` declared before `reprojected_dataset` (as the current code already does); a note in the implementation avoids an accidental reorder. — `plan.md:29`
