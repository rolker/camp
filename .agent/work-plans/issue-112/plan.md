# Plan: GGGS tile glob matches `_time`/`_source` companion files

## Issue

https://github.com/rolker/camp/issues/112

## Context

The bathy and MBES backscatter stores produce three files per tile in the same
directory: `<level>_<row>_<col>.tif` (value), `<level>_<row>_<col>_time.tif`
(Int64 ns timestamp), and `<level>_<row>_<col>_source.tif` (uint16 source index).
CAMP's `*.tif`/`*.tiff` globs in `loadDirectory()`, `rescan()`, and `dirHasTifs()`
match all three. CAMP tries to render the `_time` and `_source` companions as
depth/intensity tiles, producing garbage renders.

**Current scan sites (verified against HEAD, post-camp#104):**

- `src/camp2/raster/gggs_tile_layer.cpp:122` — `loadDirectory()` `entryList` glob
- `src/camp2/raster/gggs_tile_layer.cpp:170` — `rescan()` `entryList` glob
- `src/camp2/raster/gggs_store_source.cpp:23` — `dirHasTifs()` `entryList` glob

## Approach

1. **Add `isValueTile(const QString&)` free function** (anonymous namespace) in
   `gggs_tile_layer.cpp` — returns `true` iff the basename matches the positive
   pattern `\d+_\d+_\d+\.tiff?` (digits-underscore-digits-underscore-digits, then
   `.tif` or `.tiff`). Positive-match is preferred over a denylist because the
   companion-suffix set may grow (ADR-0006-D7 mentions future sidescan time
   tiles); the pattern is the canonical definition shared by the stores' own tile
   naming convention. Add a one-line comment explaining the pattern so future
   companion additions know where to look.

2. **Apply `isValueTile()` in `loadDirectory()`** — filter the entryList result
   before constructing `GggsTile` objects (lines 122–147).

3. **Apply `isValueTile()` in `rescan()`** — filter the entryList result before
   the `known.contains()` check (lines 170–185).

4. **Move `isValueTile()` to a shared location visible to both compilation units**
   OR duplicate the lambda in `gggs_store_source.cpp`. Because the function is
   tiny (one QRegularExpression match) and both files are in the same directory,
   add it to `gggs_tile_layer.h` as an inline free function in the anonymous
   implementation detail namespace — OR declare it in a new `gggs_tile_util.h`
   if reviewers prefer. The simplest option is a static helper in a new
   `gggs_tile_util.h` (one-line inline) included by both `.cpp` files.

5. **Apply `isValueTile()` in `dirHasTifs()`** in `gggs_store_source.cpp` — filter
   after the `entryList` call so the helper sees only value tiles when deciding
   whether a directory holds a tile-set.

6. **Add a test** — extend `test_catalog_source.cpp` with a case that places
   `0_0_0.tif`, `0_0_0_time.tif`, and `0_0_0_source.tif` in a store directory and
   asserts `discover()` returns a leaf (the companions don't trip the tile-free
   pruning path) and the leaf key points to the directory. Add a parallel case to
   `test_gggs_rescan.cpp` (or a new `test_gggs_companion_filter.cpp`) that creates
   a tile directory with base + companion files, constructs a `GggsTileLayer`, and
   asserts `valid() == true` with exactly one tile (not three).

## Files to Change

| File | Change |
|------|--------|
| `src/camp2/raster/gggs_tile_util.h` | New header: `isValueTile(const QString&)` inline function |
| `src/camp2/raster/gggs_tile_layer.cpp` | Include `gggs_tile_util.h`; filter `entryList` result in `loadDirectory()` and `rescan()` |
| `src/camp2/raster/gggs_store_source.cpp` | Include `gggs_tile_util.h`; filter `entryList` result in `dirHasTifs()` |
| `test/test_gggs_companion_filter.cpp` | New test: store dir with base + `_time` + `_source` companions; assert only base tile is enumerated by `GggsTileLayer` and by `GggsStoreSource::discover()` |
| `CMakeLists.txt` | Register `test_gggs_companion_filter` with `ament_add_gtest` (same pattern as `test_catalog_source` / `test_gggs_rescan`) |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | New test file + CMakeLists registration included in this plan. |
| Test what breaks | Test explicitly covers the bug path: companions in directory → only base tile enumerated. |
| Only what's needed | Narrow 3-site fix + shared helper + one test; no scope beyond issue description. |
| Capture decisions, not just implementations | Positive-pattern vs. denylist choice documented via inline comment in `gggs_tile_util.h`. |
| Improve incrementally | Single PR, no architectural change. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0001 (Adopt ADRs) | No | Pattern choice (positive vs. denylist) is a code-comment decision, not ADR-worthy. |
| ADR-0002 (Worktree isolation) | Yes | Worktree `issue-camp-112` already in place; all changes land here. |
| ADR-0005 (Layered enforcement) | Context | `dirHasTifs()` in `gggs_store_source.cpp` is the post-#104 successor to `GggsStoreLayer`'s subtree scan; the filter applies identically. |
| ADR-0008 (ROS 2 conventions) | Yes | C++ changes in a ROS 2 ament_cmake package; no convention violations. |
| ADR-0013 (progress.md vocabulary) | Yes | `progress.md` has an `## Issue Review` entry; `## Plan Authored` will be appended. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `dirHasTifs()` in `gggs_store_source.cpp` | `test_catalog_source.cpp` (companion-in-dir case) | Yes — new test file covers both `GggsTileLayer` and `GggsStoreSource` |
| `loadDirectory()` / `rescan()` glob filter | `test_gggs_rescan.cpp` or new test | Yes — `test_gggs_companion_filter.cpp` covers this |
| Add `gggs_tile_util.h` | `CMakeLists.txt` (install headers if needed) | No install target needed — internal header, not exported |

## Open Questions

- **Header placement**: `gggs_tile_util.h` (new shared header) vs. a static lambda
  duplicated in each `.cpp` (avoids an extra file). The shared header is cleaner for
  future companion types; the duplicate lambda is simpler if no other callers are
  expected. Recommendation: shared header, given the issue's explicit callout that
  the excluded-suffix list must live in one place.
- **New test file vs. extending existing tests**: `test_gggs_companion_filter.cpp`
  (dedicated) vs. adding cases to `test_gggs_rescan.cpp` and `test_catalog_source.cpp`.
  Recommendation: add to existing tests where the fixture already covers the relevant
  class, rather than creating a new file — unless the test logic is orthogonal enough
  to warrant its own setup.

## Estimated Scope

Single PR.
