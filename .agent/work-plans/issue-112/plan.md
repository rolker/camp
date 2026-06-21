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

1. **Add `isValueTile(const QString&)` inline free function** in a new shared
   header `src/camp2/raster/gggs_tile_util.h` (namespace `camp::raster`) —
   returns `true` iff the **filename** (what `QDir::entryList` returns, never a
   full path) matches the **anchored, full-match** positive pattern
   `^\d+_\d+_\d+\.tiff?$` via `QRegularExpression::anchoredPattern(...)` with a
   function-local `static const QRegularExpression` (compiled once). Exactly three
   underscore-separated digit groups then `.tif`/`.tiff`. Positive-match is
   preferred over a denylist because the companion-suffix set may grow; the
   pattern is the canonical value-tile name shared with the stores' own tile-
   naming convention, and a 4th `_time`/`_source` component (or any future
   companion suffix) is excluded for free by not matching. A one-line comment in
   the header explains this so future companion additions know where to look.

2. **Apply `isValueTile()` in `loadDirectory()`** — filter the entryList result
   before constructing `GggsTile` objects (lines 122–147).

3. **Apply `isValueTile()` in `rescan()`** — filter the entryList result before
   the `known.contains()` check (lines 170–185).

4. **Share `isValueTile()` via the new `gggs_tile_util.h`** (resolved: shared
   header, not a duplicated lambda) — included by both `gggs_tile_layer.cpp` and
   `gggs_store_source.cpp` so the value-tile definition lives in exactly one
   place, per the issue's "one place" requirement.

5. **Apply `isValueTile()` in `dirHasTifs()`** in `gggs_store_source.cpp` — filter
   after the `entryList` call so the helper sees only value tiles when deciding
   whether a directory holds a tile-set.

6. **Extend the existing tests** (resolved: extend, no new test file). In
   `test_catalog_source.cpp` add a case that places `0_0_0.tif`, `0_0_0_time.tif`,
   and `0_0_0_source.tif` in a store directory and asserts `discover()` returns a
   leaf keyed to the directory, plus a negative case that a directory holding ONLY
   companions is not a tile-set (pruned). In `test_gggs_rescan.cpp` add a case
   that constructs a `GggsTileLayer` over a directory with base + companions
   (companions written at a disjoint eastward extent) and asserts `valid()==true`
   with `sceneBounds()` equal to a base-only layer's bounds (companions excluded —
   only one tile loaded), plus a case that companions landing after load make
   `rescan()` no-op (`false`, extent untouched).

## Files to Change

| File | Change |
|------|--------|
| `src/camp2/raster/gggs_tile_util.h` | New header: `isValueTile(const QString&)` inline function |
| `src/camp2/raster/gggs_tile_layer.cpp` | Include `gggs_tile_util.h`; filter `entryList` result in `loadDirectory()` and `rescan()` |
| `src/camp2/raster/gggs_store_source.cpp` | Include `gggs_tile_util.h`; filter `entryList` result in `dirHasTifs()` |
| `test/test_catalog_source.cpp` | Add: tile-set dir with base + `_time` + `_source` companions discovers as a leaf; companions-only dir is not a tile-set |
| `test/test_gggs_rescan.cpp` | Add: `GggsTileLayer` over base + companions loads only the base tile; companions landing after load make `rescan()` no-op |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | Companion-filter cases added to the existing `test_catalog_source.cpp` and `test_gggs_rescan.cpp` targets (no new target/CMakeLists change). |
| Test what breaks | Tests explicitly cover the bug path: companions in directory → only base tile enumerated. |
| Only what's needed | Narrow 3-site fix + shared helper + extended tests; no scope beyond issue description. |
| Capture decisions, not just implementations | Positive-pattern vs. denylist choice documented via inline comment in `gggs_tile_util.h`. |
| Improve incrementally | Single PR, no architectural change. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0001 (Adopt ADRs) | No | Pattern choice (positive vs. denylist) is a code-comment decision, not ADR-worthy. |
| ADR-0002 (Worktree isolation) | Yes | Worktree `issue-camp-112` already in place; all changes land here. |
| ADR-0005 (Catalog browser + flat selectable display layers) | Context | `dirHasTifs()` in `gggs_store_source.cpp` is the post-#104 successor to `GggsStoreLayer`'s subtree scan; the filter applies identically. |
| ADR-0008 (ROS 2 conventions) | Yes | C++ changes in a ROS 2 ament_cmake package; no convention violations. |
| ADR-0013 (progress.md vocabulary) | Yes | `progress.md` has an `## Issue Review` entry; `## Plan Authored` will be appended. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `dirHasTifs()` in `gggs_store_source.cpp` | `test_catalog_source.cpp` (companion-in-dir case) | Yes — companion + companions-only cases added |
| `loadDirectory()` / `rescan()` glob filter | `test_gggs_rescan.cpp` | Yes — load + rescan companion cases added |
| Add `gggs_tile_util.h` | `CMakeLists.txt` (install headers if needed) | No install target needed — internal header, not exported |

## Open Questions (resolved)

- **Header placement** — RESOLVED: shared header `src/camp2/raster/gggs_tile_util.h`
  with an inline `isValueTile()`, included by both `gggs_tile_layer.cpp` and
  `gggs_store_source.cpp`. The excluded-suffix logic lives in exactly one place,
  per the issue's explicit callout.
- **New test file vs. extending existing tests** — RESOLVED: extend the existing
  `test_catalog_source.cpp` and `test_gggs_rescan.cpp` targets (their `touchTif`/
  `writeTile` fixtures already cover the relevant classes); no new test file or
  CMakeLists change.

## Estimated Scope

Single PR.
