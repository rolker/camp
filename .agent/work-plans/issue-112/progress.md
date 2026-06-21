---
issue: 112
---

# Issue #112 — GGGS tile glob matches `_time`/`_source` companion files

## Issue Review
**Status**: complete
**When**: 2026-06-21 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #112
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Summary

Companion tiles (`<grid>_time.tif`, `<grid>_source.tif`) produced by the bathy and MBES backscatter stores (unh_marine_autonomy#178 / #194) land in the same directory as the value tile (`<grid>.tif`). The GGGS tile-layer and store-source glob `*.tif`/`*.tiff` with no companion-suffix filter, so CAMP treats the companions as renderable value tiles — producing garbage renders.

**Current scan sites (verified against HEAD, post–camp#104):**
- `src/camp2/raster/gggs_tile_layer.cpp:122` — `loadDirectory()` entryList glob
- `src/camp2/raster/gggs_tile_layer.cpp:170` — `rescan()` entryList glob
- `src/camp2/raster/gggs_store_source.cpp:23–24` — `dirHasTifs()` entryList glob

The two stale sites from the original issue body (`gggs_store_layer.cpp:21` / `:27`) no longer exist — that file was retired by camp#104. Only the three sites above remain.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| A change includes its consequences | Action needed | The fix requires a companion test (companion files in a store dir; assert only base tiles listed/rendered). Extensive GGGS test coverage already exists (`test_gggs_rescan.cpp`, `test_gggs_flat_layer_spawn.cpp`, `test_gggs_render.cpp`, etc.) — the new test follows the same pattern. |
| Test what breaks | Action needed | No test currently exercises `_time`/`_source` companions; one must be added. |
| Only what's needed | OK | Targeted 3-site fix + shared helper; no scope creep. |
| Improve incrementally | OK | Narrow fix, single PR, no cross-cutting redesign. |
| Capture decisions, not just implementations | Watch | The positive-pattern (`\d+_\d+_\d+\.tif`) vs. denylist (`_time`/`_source` suffix exclusion) choice should be noted inline so future companion additions know which approach was taken and why. |
| Workspace vs. project separation | OK | All changes are in the `camp` project repo; no workspace infra touched. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0001 (Adopt ADRs) | No | Design choice (positive pattern vs. denylist) is implementor's call; a code comment suffices — no new ADR needed. |
| ADR-0002 (Worktree isolation) | Yes | Worktree already in place for issue-112. |
| ADR-0005 (Stores browser / flat display layers) | Yes (context) | This fix is consistent with ADR-0005: `dirHasTifs()` and `buildNode()` in `GggsStoreSource` are the successors to `GggsStoreLayer`'s subtree scan; the companion filter applies here too. |
| ADR-0008 (ROS 2 conventions) | Yes | C++ change in a ROS 2 package — follow standard ROS 2/ament_cmake C++ conventions. |
| ADR-0013 (progress.md vocabulary) | Yes | This entry. |

### Consequences

- A shared `isValueTile(QString)` helper (or positive-pattern lambda) must be placed in a single location and called at all three scan sites — the issue explicitly flags this so future companion bands don't require multi-site edits.
- One new test file (or additions to `test_gggs_flat_layer_spawn.cpp` / `test_gggs_rescan.cpp`) must exercise the companion-filter path.
- No interface changes, no `.msg`/`.srv` changes, no doc updates needed beyond inline comments.

### Actions
- [ ] Implement `isValueTile()` helper (or positive-pattern filter) at a single definition site.
- [ ] Apply the filter at all three current scan sites: `loadDirectory()`, `rescan()`, `dirHasTifs()`.
- [ ] Add a test: construct a store dir with base + `_time` + `_source` companions; assert only the base tile is listed/loaded/rendered.
- [ ] Document the positive-vs.-denylist choice in an inline comment so future companion bands know which approach to extend.

## Plan Authored
**Status**: complete
**When**: 2026-06-21 12:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-112/plan.md` at `1251ab7`
**Branch**: feature/issue-112 at `1251ab7`
**Phases**: single

### Open questions
- [ ] Header placement: new `gggs_tile_util.h` (shared, one definition site) vs. static lambda duplicated in each `.cpp` — recommendation is shared header per the issue's "one place" requirement.
- [ ] New test file (`test_gggs_companion_filter.cpp`) vs. adding cases to existing `test_gggs_rescan.cpp` and `test_catalog_source.cpp` — recommendation is add to existing tests where fixture already covers the relevant class.
