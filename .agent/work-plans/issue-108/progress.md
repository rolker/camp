---
issue: 108
---

# Issue #108 — GeoTIFF band picker for store display layers

## Issue Review
**Status**: complete
**When**: 2026-06-27 23:56 +0000
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #108
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Actions
- [ ] Confirm #104's flat-selectable-layer work is merged/rebased before starting — the flat display layer is the anchoring UI surface for the band picker.
- [ ] Per-band NoData: re-query `GetNoDataValue()` for the selected band (not band 1's cached value) when the operator switches bands (`gggs_tile.cpp:48`).
- [ ] Reset auto-range (`data_min_`/`data_max_`) and reload pixels on band switch — range is band-specific, reusing band 1's range for another band would give wrong colormap scaling.
- [ ] Single-band graceful handling: disable or hide the band-picker context menu item when `GetRasterCount() == 1` — nothing to pick.
- [ ] Update doc comments that will be stale after the change: `gggs_tile.h:36` ("Read band 1"), `gggs_tile_layer.h:41-42` ("Slice 1: single-band…  Band-select + colormap are [follow-up]").
- [ ] Persist the selected band index via QSettings alongside colormap, consistent with how colormap is persisted in `GggsTileLayer`.

## Plan Authored
**Status**: complete
**When**: 2026-06-28 00:10 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-108/plan.md` at `4d40d16`
**Branch**: feature/issue-108 at `4d40d16`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-06-28 00:06 +00:00
**By**: Claude Code Agent (Claude Opus) (in-context — author self-review)

**Plan**: `.agent/work-plans/issue-108/plan.md` at `4d40d16`
**PR**: PR-less (--issue mode)
**Verdict**: changes-requested

### Findings
- [ ] (must-fix) New `test/test_gggs_band_select.cpp` not registered in `CMakeLists.txt` (`ament_add_gtest` + include/link, mirror `test_gggs_render` at `CMakeLists.txt:562`); without it the test never builds — `plan.md:57-65`
- [ ] (must-fix) Per-band NoData not re-queried — `has_nodata_`/`nodata_` are read once for band 1 (`gggs_tile.cpp:47-49`, used in range filter at `:93`); `setBand` must refresh them so a switched band ranges correctly (review-issue finding) — `plan.md:21-24`
- [ ] (suggestion) Stale doc comments at `gggs_tile.h:36` ("Read band 1") and `gggs_tile_layer.h:41-43` ("Slice 1: single-band… Band-select [follow-up]") not flagged for update (review-issue finding) — `plan.md:60-63`
- [ ] (suggestion) Fragment shader discards `v <= 0.0` and auto-ranges (`gggs_tile_layer.cpp:71-73`) — a second band with 0/negative valid samples renders blank/mis-ranged; acknowledge in Open Questions — `plan.md:92-95`
- [ ] (suggestion) Layer `setBand` must guard null `gl_context_` (not-yet-painted) before makeCurrent/texture release, as `releaseGL()` already does (`gggs_tile_layer.cpp:571`) — `plan.md:27-33`

**Note**: independence annotation applied per the skill's name-based rule (the `## Plan Authored` entry shares the `Claude Code Agent` agent-name); this review was nonetheless a fresh-context dispatch on a different model (Sonnet authored, Opus reviewed).

## Implementation
**Status**: complete
**When**: 2026-06-28 00:28 +0000
**By**: Claude Opus

**Summary**: Implemented per-layer band selection for `GggsTileLayer` per the
committed plan, folding in all 5 plan-review findings.

- `GggsTile` (`gggs_tile.{h,cpp}`): store `band_count_ = GetRasterCount()` in the
  ctor (kept the `< 1` guard); added `int band_ = 1`, `bandCount()`, `band()`, and
  `setBand(int)` (clears `data_`, resets range to the crossed sentinel, sets
  `pixels_loaded_ = false`, validates `[1, band_count_]`, does NOT touch the GL
  texture). `loadPixels()` now reads `GetRasterBand(band_)` and **re-queries
  `GetNoDataValue()` for that band** (must-fix #2) so a switched band ranges
  correctly — done in `loadPixels()` (which already has the dataset/band open)
  rather than `setBand()` (which does not open the dataset).
- `GggsTileLayer` (`gggs_tile_layer.{h,cpp}`): added `int band_ = 1`, `band()`,
  `bandCount()` (delegates to the first valid tile), and `setBand(int)` —
  validates, aborts+joins the in-flight load, **guards a null `gl_context_`**
  (suggestion #5) before `makeCurrent` + per-tile texture release (textures only,
  not shader/FBO/LUT), re-points each tile, resets the layer range to crossed,
  invalidates `cached_image_`, persists, re-kicks `loadTiles()` only if a load had
  started, and repaints.
- Context menu: "Band" submenu added **only when `bandCount() > 1`**; one checkable
  1-indexed action per band, checked at the current selection, wired to `setBand`.
- Persistence: `writeSettings()` writes `"band"` and `readSettings()` reads it
  (default 1) under the existing `MapItem/itemID()` group, applying via `setBand()`
  only when it differs.
- Tests: extended `test_gggs_tile.cpp` (`BandSelectSwitchesRange` — 2-band tile,
  `bandCount()==2`, switch band, distinct range; passes) and added
  `test/test_gggs_band_select.cpp` (`SetBandShiftsLayerRange` — 2-band tile-set,
  `GggsTileLayer`, `waitForLoad()`, render differs after `setBand(2)`; **SKIPs**
  not FAILs without offscreen GL via the same `offscreenGLAvailable()` guard).
- Doc comments updated (suggestion #3): `gggs_tile.h` "Read band 1" → selected
  band; `gggs_tile_layer.h` "Slice 1: single-band… [follow-up]" → implemented
  band picker.
- Shader `v <= 0.0` limitation (suggestion #4): NOT reworked; documented in a code
  comment beside the discard and in the plan's Open Questions.

**Build/test result** (verified in-container):
- Build: `./ui_ws/build.sh camp` → `1 package finished` (camp), warnings only
  (all pre-existing), no errors. Required building the shared `underlay_ws` (22
  pkgs) and `core_ws` (35 pkgs) symlink layers first; their installs were empty.
- Test: `./ui_ws/test.sh camp` → **114 tests, 0 errors, 0 failures, 3 skipped**.
  `test_gggs_tile` (10 tests incl. `BandSelectSwitchesRange`) all passed;
  `test_gggs_band_select` SKIPPED (no offscreen GL in container — by design).

**Findings checklist** (all 5 addressed):
- [x] (must-fix) New test registered in `CMakeLists.txt` — `ament_add_gtest` +
  include/link for `test_gggs_band_select.cpp`, mirroring `test_gggs_render`;
  verified it builds (`build/camp/test_gggs_band_select` present, ran).
- [x] (must-fix) Per-band NoData re-queried in `loadPixels()` for the selected band.
- [x] (suggestion) Stale doc comments updated in `gggs_tile.h` + `gggs_tile_layer.h`.
- [x] (suggestion) Shader `v<=0` discard limitation acknowledged in Open Questions
  + a code comment; no shader rework.
- [x] (suggestion) Null `gl_context_` guarded in layer `setBand` before
  `makeCurrent`/texture release.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-28 02:12 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-108 at `946d9c2`
**Mode**: pre-push
**Depth**: Deep (reason: 200+ changed lines + concurrency/GL-lifecycle promotion trigger)
**Must-fix**: 0 | **Suggestions**: 4
**Round**: 1 | **Ship**: recommended — no must-fix; both adversarial lenses independently verified the abort/join, GL-texture release ordering, null-context guard, and crossed-range re-fold as correct.

Static analysis: cppcheck clean; no lines >100 cols / no trailing whitespace
(cpplint binary unavailable in this env — line length checked manually). Claude
Adversarial: 2 passes (Lens A logic + Lens B systemic, Deep horizon). Copilot:
off (default). All 5 plan-review findings confirmed folded in. Plan adherence:
exact, no scope creep. Governance: ADR-0005 (#108 is its scoped band-select
follow-up), ADR-0002/0003 — compliant.

### Findings
- [ ] (suggestion) `readSettings()`→`setBand()`→`writeSettings()` couples the read path to a settings write (safe now, but fragile); consider an internal `applyBand` that skips the persist, like colormap's inline apply — `gggs_tile_layer.cpp:731`
- [ ] (suggestion) Non-uniform band counts across tiles silently mishandled — a tile with fewer bands than `tiles_.front()` is dropped from the render with no status signal; guard if mixed dirs are possible — `gggs_tile_layer.cpp:646`
- [ ] (suggestion) Constructor band-1 NoData read is now dead (re-queried in `loadPixels`); minor cleanup — `gggs_tile.cpp:49`
- [ ] (suggestion) File a follow-up: shader `v <= 0.0` discard mis-ranges signed/uncertainty bands the picker now makes selectable (documented/deferred) — `gggs_tile_layer.cpp:78`

## Implementation
**Status**: complete
**When**: 2026-06-28 02:37 +0000
**By**: Claude Opus

**Summary**: Addressed local-review (pre-push) suggestions 1-3 in code; suggestion 4
deferred to the camp#122 follow-up (shader not reworked). Commit `9adfd7d` on
`feature/issue-108`.

1. **applyBand decoupling** (suggestion 1) — added a private, non-persisting
   `GggsTileLayer::applyBand(int)` carrying the band-switch body (validate →
   abort+join load → null-guarded GL texture release → re-point tiles → reset
   range/cache → re-kick load → repaint). Public `setBand()` now guards, calls
   `applyBand()`, then `writeSettings()`; `readSettings()` calls `applyBand()`
   directly, so the read path no longer triggers a settings write. Mirrors the
   inline colormap apply. The "only apply if differs" guard is retained in both.
2. **Non-uniform per-tile band-count guard** (suggestion 2) — `applyBand()` now
   skips any tile whose own `bandCount()` is below the requested band (leaving it
   on its prior band rather than letting `GggsTile::setBand()` silently no-op) and
   emits a one-shot `qWarning` per switch reporting the dropped count, matching the
   file's existing `qWarning("GggsTileLayer: ...")` idiom.
3. **Dead band-1 NoData read cleanup** (suggestion 3) — removed the constructor
   `GetNoDataValue()` read and the now-unused `GetRasterBand(band_)` handle in
   `gggs_tile.cpp`; `loadPixels()` already re-queries NoData for the selected band.
   `has_nodata_`/`nodata_` members and the `band_count_`/`GetRasterCount()` storage
   are retained (members just stay at defaults until the first `loadPixels()`; no
   path reads them earlier — `texture()`/range are gated on `pixelsLoaded()`).
   Stale doc comments in `gggs_tile.{h}` updated; the `NoDataExcludedFromRange`
   test now asserts `hasNoData()`/`noData()` after `loadPixels()`.
4. **Shader `v<=0` discard** (suggestion 4) — NOT reworked, per instruction. The
   code comment beside the discard now references the **camp#122** follow-up.

Also synced `plan.md` (persistence step + Files-to-Change) to note the `applyBand`
refactor and the band-count guard.

**Build/test result** (verified in-container):
- Build: `./ui_ws/build.sh camp` → `1 package finished` (camp), warnings only
  (all pre-existing, e.g. Qt `HighQualityAntialiasing` deprecation), no errors.
  Required building the shared `underlay_ws` (22 pkgs) and `core_ws` (35 pkgs)
  symlink layers first; their installs were empty.
- Test: `./ui_ws/test.sh camp` → **114 tests, 0 errors, 0 failures, 3 skipped**.
  The 3 skips are the offscreen-GL tests (`test_gggs_band_select` etc.) that SKIP
  in-container by design — expected, not a failure.

**Note**: `feature/issue-108` not pushed and no PR opened, per the handoff contract
(the host performs pushes).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-28 02:54 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-108 at `2c8c873`
**Mode**: pre-push
**Depth**: Deep (reason: GL-texture lifecycle + async-worker concurrency on a 200+ line change)
**Must-fix**: 0 | **Suggestions**: 2
**Round**: 2 | **Ship**: recommended — no must-fix; the prior round's suggestions 1-3 are folded in correctly, and both adversarial lenses re-verified the abort/join, texture-release ordering, per-band NoData re-query, and crossed-range re-fold. The one new finding is a defensive-path gap the store-uniformity invariant prevents from ever firing.

Static analysis: cppcheck clean apart from pre-existing `shadowFunction` style notes
(locals `width`/`height`/`band` shadow accessor methods — pattern predates this
change); line-length (<=100) and trailing-whitespace clean (cpplint binary
unavailable — checked manually). Claude Adversarial: 2 passes (Lens A logic +
Lens B systemic/concurrency, Deep horizon); Lens B traced the worker abort/join,
GL context lifecycle, and release/acquire memory ordering and found no race/leak/
use-after-free. Copilot: off (default). Plan adherence: exact, no scope creep.
Governance: camp ADR-0005 (this is its scoped band-select follow-up), ADR-0002/0003
— compliant.

### Findings
- [ ] (suggestion) Non-uniform tile-set: `applyBand()` releases every tile's texture but only reloads tiles that have the requested band; a dropped tile (data_ freed to GPU, pixels_loaded_ still true) renders BLANK rather than "left on their prior band" as the comment claims, and its stale prior-band range still pollutes the tilesReady() fold. Gated behind the store-uniformity invariant (never fires on a uniform store). Fix: skip releaseGL() for dropped tiles, and/or correct the comment — `src/camp2/raster/gggs_tile_layer.cpp:653`
- [ ] (suggestion) Shader `v <= 0.0` discard mis-ranges signed/uncertainty bands the picker now makes selectable; already documented in-code and deferred to camp#122 — tracking only — `src/camp2/raster/gggs_tile_layer.cpp:62`

## Implementation
**Status**: complete
**When**: 2026-06-28 10:01 +0000
**By**: Claude Opus

**Summary**: Addressed the Round-2 local-review must-fix candidate (the non-uniform
tile guard); the shader `v<=0` item stays deferred to camp#122 (untouched). Commit
`0b62c24` on `feature/issue-108`.

**The guard fix** (`gggs_tile_layer.cpp`) — `applyBand()` previously released
*every* tile's GL texture but only `setBand()`-reloaded tiles carrying the requested
band, so a tile lacking that band had its texture freed (`data_` already gone to the
GPU) while `pixels_loaded_` stayed true → it rendered BLANK, and its stale prior-band
range still folded into the layer auto-range. That contradicted the guard's own
"left on their prior band" comment. Now:
- `applyBand()` skips both `releaseGL()` and `setBand()` for any tile whose own
  `bandCount()` is below the requested band, so it keeps its already-loaded
  prior-band texture/pixels and renders normally. Merged the former two-pass
  (release-all, then re-point) into a single loop under one `makeCurrent`, with a
  `have_context` flag carrying the **unchanged** null `gl_context_`/`gl_surface_`
  guard. Only tiles that DO carry the band are released + re-pointed.
- `tilesReady()` now folds only tiles whose `band() == band_`, excluding a kept
  prior-band tile so its stale min/max can't pollute the new band's auto-range.
- The one-shot `qWarning` per switch and the in-flight abort+join are unchanged.
- The guard comment now accurately states the behavior; `plan.md` step 4 synced.

Invariant-gated: a real GGGS store has uniform band counts, so this path never fires
in practice — but the defensive behavior now matches the comment, as Roland chose.

**No forced test**: the dropped-tile behavior (range-fold exclusion + prior-band
retention) has no non-offscreen-GL observable seam — the layer exposes neither
`data_min_`/`data_max_` nor its individual tiles, so verifying it without GL would
need test-only API outside this finding's scope. The existing GL-gated
`test_gggs_band_select` (which SKIPs in-container) already exercises `applyBand`'s
abort/reload/range-shift on the uniform path. Per the task's "otherwise don't force
it", no test was added.

**Build/test result** (verified in-container):
- Build: `./ui_ws/build.sh camp` → `1 package finished` (camp), warnings only
  (all pre-existing, e.g. `-Wunused-parameter` in mainwindow.cpp), no errors.
  Required building the shared `underlay_ws` (22 pkgs) and `core_ws` (35 pkgs)
  symlink layers first; their installs were empty.
- Test: `./ui_ws/test.sh camp` → **114 tests, 0 errors, 0 failures, 3 skipped**.
  The 3 skips are the offscreen-GL tests (`test_gggs_band_select` etc.) that SKIP
  in-container by design — expected, not a failure.

**Note**: `feature/issue-108` not pushed and no PR opened, per the handoff contract
(the host performs pushes).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-28 10:17 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-108 at `169733e`
**Mode**: pre-push
**Depth**: Deep (reason: GL-texture lifecycle + async-worker concurrency on a 200+ line change)
**Must-fix**: 1 | **Suggestions**: 3
**Round**: 3 | **Ship**: continue — a new correctness must-fix surfaced this round (rescan does not inherit the selected band), cross-confirmed by Lens A and the lead read; it fires on the normal switch-then-rescan flow (not invariant-gated), so it warrants a fix + the regression test that would have caught it before shipping.

Static analysis: cppcheck clean apart from pre-existing `shadowFunction` notes (locals
`width`/`height`/`band` shadow accessors — predates this change); line-length (<=100)
and trailing-whitespace clean (cpplint binary unavailable — checked manually). Claude
Adversarial: 2 passes (Lens A logic + Lens B systemic/GL, Deep horizon). Lens B traced
the worker abort/join, GL context lifecycle, atomic `pixels_loaded_` release/acquire
ordering, and destruction-during-switch and found no race/leak/use-after-free; Lens A
traced the selected band across rescan/readSettings/tilesReady/renderImage and found the
must-fix below. Copilot: off (default). Plan adherence: exact, no scope creep (the
rescan interaction is a plan gap, not a deviation). Governance: ADR-0005 (this is its
scoped band-select follow-up), ADR-0002/0003 — compliant.

### Findings
- [ ] (must-fix) `rescan()` does not propagate the layer's `band_` to newly added tiles — after a switch to band N>1, a rescanned tile keeps default band 1: renders the wrong band through band N's range and is permanently excluded from the auto-range fold with no recovery. Fix: `tile->setBand(band_)` on each new tile in the rescan add loop — `src/camp2/raster/gggs_tile_layer.cpp:223`
- [ ] (suggestion) `applyBand()` texture-release asymmetry: when `gl_context_` exists but `makeCurrent` fails, `releaseGL()` is skipped yet `setBand()` still clears pixels; `texture()` never refreshes an existing texture, so a stale old-band texture can render (narrow window) — `src/camp2/raster/gggs_tile_layer.cpp:670`
- [ ] (suggestion) `test_gggs_band_select` covers only a clean uniform `setBand`; add rescan-after-switch coverage (the must-fix above passes the existing test) — `test/test_gggs_band_select.cpp:1`
- [ ] (suggestion) Shader `v <= 0.0` discard mis-ranges signed/uncertainty bands the picker now makes selectable; already documented in-code and deferred to camp#122 — tracking only — `src/camp2/raster/gggs_tile_layer.cpp:62`
