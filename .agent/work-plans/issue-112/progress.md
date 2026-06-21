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

## Plan Review
**Status**: complete
**When**: 2026-06-21 16:04 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-112/plan.md` at `1251ab7`
**PR**: PR-less (`--issue` mode; `gh` unauthenticated — issue substance cross-checked against the `## Issue Review` entry above)
**Verdict**: approve-with-suggestions

Independent review (fresh-context Opus dispatch; the `## Plan Authored` entry is a
separate Sonnet dispatch under the shared `Claude Code Agent` name — not author
self-review). Core approach validated against HEAD: three scan sites confirmed
(`gggs_tile_layer.cpp:122`, `:170`; `gggs_store_source.cpp:23`); positive pattern
`\d+_\d+_\d+\.tiff?` matches every legitimate value tile (all tiles/tests use
`<level>_<row>_<col>.tif`) and excludes `_time`/`_source` companions without
dropping real tiles. Consequences map and test obligation are complete. All
findings below are suggestions; none block implementation.

### Findings
- [ ] (suggestion) ADR-0005 mislabeled "Layered enforcement" (workspace title); camp ADR-0005 is "Catalog browser + flat selectable display layers" — fix title, substance is correct — `plan.md:85`
- [ ] (suggestion) Spurious "ADR-0006-D7" citation — camp has no ADR-0006 (only 0001–0005); drop or correct, the "companion set may grow" rationale stands alone — `plan.md:28`
- [ ] (suggestion) Specify anchored/full regex match (`anchoredPattern`/`^…$`) for `isValueTile()` so partial matches can't admit malformed names — `plan.md:27`
- [ ] (suggestion) Resolve test-structure inconsistency: "Files to Change" adds new `test_gggs_companion_filter.cpp` while Approach/Open-Questions recommend extending existing `test_catalog_source.cpp`/`test_gggs_rescan.cpp` (which already have reusable fixtures) — pick one; extending is recommended — `plan.md:56,66,104`

## Implementation
**Status**: complete
**When**: 2026-06-21 17:30 +00:00
**By**: Claude Opus

**Branch**: feature/issue-112 at `4e46fb1`
**Commits**:
- `6f4070d` fix(gggs): filter `_time`/`_source` companion tiles from value-tile scans — helper + 3 sites + plan sync
- `4e46fb1` test(gggs): cover companion-tile filtering — extend existing targets

### What was built

**Shared helper** — new `src/camp2/raster/gggs_tile_util.h` with an inline
`camp::raster::isValueTile(const QString& filename)`. It matches the filename
(never a full path) against an anchored, full-match positive pattern
`^\d+_\d+_\d+\.tiff?$` via `QRegularExpression::anchoredPattern(...)` with a
function-local `static const QRegularExpression` (compiled once). Exactly three
underscore-separated digit groups then `.tif`/`.tiff` → the base value tile;
companions (a 4th `_time`/`_source` component) and malformed names fail to
match. A header comment records the positive-pattern decision and that future
companion suffixes are excluded for free.

**Three patched scan sites** (each now skips non-value tiles), both `.cpp`s
include `gggs_tile_util.h`:
- `gggs_tile_layer.cpp` `loadDirectory()` — `if(!isValueTile(name)) continue;`
  before constructing the `GggsTile`.
- `gggs_tile_layer.cpp` `rescan()` — same skip before the `known.contains()`
  check, so a rescan never adds a companion as a new tile.
- `gggs_store_source.cpp` `dirHasTifs()` — iterate the `*.tif` entryList and
  return true only if at least one entry `isValueTile()`, so a directory of
  companions alone is not counted as a tile-set.

**Tests** (extended existing targets — no new file, no CMakeLists change):
- `test_catalog_source.cpp` → `CompanionTilesDoNotBreakDiscovery`: a tile-set
  dir with `0_0_0.tif` + `0_0_0_time.tif` + `0_0_0_source.tif` discovers as a
  leaf keyed to the dir; a sibling dir holding ONLY companions is pruned (not a
  tile-set).
- `test_gggs_rescan.cpp` → `CompanionTilesAreNotLoaded`: a `GggsTileLayer` over
  base + companions (companions written at a disjoint *eastward* extent, so they
  would widen `sceneBounds()` if loaded) has `valid()==true` and `sceneBounds()`
  equal to a base-only layer's — exactly one tile loaded.
- `test_gggs_rescan.cpp` → `RescanIgnoresCompanionTiles`: companions landing
  after the initial load make `rescan()` return `false` with the extent
  untouched.
- No tile-count accessor was added; the existing public `sceneBounds()`/`valid()`
  surface (with disjoint-extent companions) distinguishes one-tile from
  three-tile loads, per the plan.

### Doc fixes (Plan Review findings, applied to `plan.md`)
- ADR-0005 retitled "Catalog browser + flat selectable display layers".
- Spurious "ADR-0006-D7" citation removed (camp has no ADR-0006).
- `isValueTile()` spec updated to anchored/full-match + filename-only.
- Both Open Questions resolved (shared header; extend existing tests); Files-to-
  Change / Consequences tables updated to drop the new-test-file + CMakeLists rows.

### Build / test
Camp deps were absent (empty `core_ws/install`): built `marine_ais_msgs`,
`marine_interfaces`, `marine_autonomy` via `colcon build --packages-up-to` in
`core_ws` first (warnings only). Then:
- Build: `./ui_ws/build.sh camp` → 1 package finished (pre-existing warnings only).
- Test: `./ui_ws/test.sh camp` → **99 tests, 0 errors, 0 failures, 2 skipped**.
The three new cases were confirmed run and passing (direct binary runs).

Not pushed.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-21 16:29 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-112 at `37981cb`
**Mode**: pre-push
**Depth**: Standard (reason: 5 reviewable code files / ~149 lines; raster-render path)
**Must-fix**: 0 | **Suggestions**: 2
**Round**: 1 | **Ship**: recommended — no must-fix; only optional case-sensitivity/test-coverage suggestions

### Findings
- [ ] (suggestion) Case mismatch: `*.tif`/`*.tiff` glob may admit `.TIF`/`.TIFF` but regex `\.tiff?` is case-sensitive → uppercase-ext value tile silently dropped; theoretical (producer emits lowercase), optional `CaseInsensitiveOption` — `src/camp2/raster/gggs_tile_util.h:24`
- [ ] (suggestion) No test for a 4-group name (`13_0_0_0.tif`); correctly rejected by the anchored regex but unexercised — `src/camp2/raster/gggs_tile_util.h:24`

### Notes
- Two fresh-context Claude adversarial passes (Lens A logic / Lens B systemic) both verdict the fix correct: regex accepts all legitimate `<level>_<row>_<col>.tif` value tiles and rejects all companions; the three patched sites (`loadDirectory`, `rescan`, `dirHasTifs`) are the complete `*.tif` scan set (no missed site); `GggsTile` is constructed only at the two filtered layer sites; the async worker/ctor need no guard (companions never enter `tiles_`). The case-sensitivity suggestion was cross-pass confirmed.
- Static analysis: no actionable findings. `ament_cpplint` copyright/header-guard/C-cast hits dropped — camp does not enforce ament_cpplint (no CI/pre-commit/CMake hook), cast hits are on untouched lines, and the new header matches every sibling's guard style and no-copyright convention.
- Plan adherence: exact — files and approach match `plan.md` as written; no drift/scope creep.
- Doc-citation nit (not code): earlier lifecycle entries cite ADR-0008/ADR-0013, which are workspace ADRs; camp's own ADR set is 0001–0005.
