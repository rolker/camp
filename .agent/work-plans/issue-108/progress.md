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

## Implementation
**Status**: complete
**When**: 2026-06-28 10:38 +0000
**By**: Claude Opus

**Summary**: Addressed the Round-3 local-review must-fix (rescan band propagation)
plus the two non-shader suggestions. The shader `v<=0` item stays deferred to
camp#122 (untouched). Commit `17158cb` on `feature/issue-108`.

1. **(must-fix) rescan() propagates the selected band** (`gggs_tile_layer.cpp`) —
   the rescan add loop now calls `tile->setBand(band_)` on each newly discovered
   tile before it is folded into `tiles_`. A freshly-constructed `GggsTile` defaults
   to band 1, so after a switch to band N>1 a rescanned tile rendered the wrong band
   scaled through band N's range and was permanently excluded from the
   `tilesReady()` auto-range fold (which folds only `band() == band_`), with no
   recovery. `setBand(1)` on the band-1 default is a no-op (`GggsTile::setBand`
   returns early on `band == band_`), so single-band / band-1 stores are unaffected.

2. **(suggestion) applyBand() texture-release asymmetry** (`gggs_tile_layer.cpp`) —
   when `gl_context_` exists but `makeCurrent` fails, the old path skipped
   `releaseGL()` yet `setBand()` still cleared each tile's pixels, and `texture()`
   never refreshes an EXISTING texture, so a stale old-band texture could render
   against the new band's range once pixels reloaded (narrow window). `applyBand()`
   now distinguishes the three context states: a null/not-yet-created context stays
   the expected no-op (no textures exist yet); a successful `makeCurrent` releases
   as before; a `makeCurrent` FAILURE now sets `gl_failed_` (and WARNs), exactly
   matching `renderImage()`'s response to the same failure — `ensureGL()`
   short-circuits so the layer stops rendering rather than drawing a stale-band
   frame. Minimal, idiom-matching; no broader rework needed.

3. **(suggestion) rescan-after-switch test coverage** (`test_gggs_band_select.cpp`)
   — added `RescanInheritsSelectedBand`: switch a 2-band layer to band 2, `rescan()`
   in a newly-landed tile, then assert via a new narrow GL-free `tileBands()` test
   seam that EVERY tile (original + rescanned) reads band 2. The per-tile band
   assertion needs no GL, so this test **RUNS (not SKIPs) in-container** and would
   have caught the must-fix; the render confirmation stays
   `offscreenGLAvailable()`-guarded. **Verified it is a genuine regression test**:
   with `tile->setBand(band_)` reverted, the test FAILS (`band` reads 1, expected
   2); with the fix in place it PASSES. `tileBands()` is a narrowly-scoped test-only
   accessor (returns each tile's `band()` in order) documented as not part of the
   public layer surface.

4. **Shader `v<=0` discard** (suggestion 4) — NOT touched; the in-code comment still
   points at the camp#122 follow-up, as instructed.

`plan.md` synced: step 4 notes the rescan interaction (round-3 fix), the Tests
section lists the new regression test, and the Files-to-Change table reflects the
`rescan()`/`applyBand()`/`tileBands()` edits.

**Build/test result** (verified in-container):
- Build: `./ui_ws/build.sh camp` → `1 package finished` (camp), warnings only
  (all pre-existing, e.g. `-Wunused-parameter`, Qt `matrix()` deprecation), no
  errors. Required first building the shared `underlay_ws` (22 pkgs) and `core_ws`
  (35 pkgs) symlink layers (their installs were empty): `Summary: 22 packages
  finished` and `Summary: 35 packages finished`, both exit 0.
- Test: `./ui_ws/test.sh camp` → **115 tests, 0 errors, 0 failures, 3 skipped**
  (was 114; +1 from the new test). The 3 skips are the offscreen-GL tests that SKIP
  in-container by design. The new `RescanInheritsSelectedBand` RAN and passed
  (`[ OK ] ... (51 ms)`); `SetBandShiftsLayerRange` SKIPPED (no offscreen GL).

**Note**: `feature/issue-108` not pushed and no PR opened, per the handoff contract
(the host performs pushes).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-28 10:51 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-108 at `5f3d6a3`
**Mode**: pre-push
**Depth**: Deep (reason: GL-texture lifecycle + async-worker concurrency on a 200+ line change)
**Must-fix**: 0 | **Suggestions**: 3
**Round**: 4 | **Ship**: recommended — no must-fix; the Round-3 must-fix (rescan band propagation) is folded in correctly at `gggs_tile_layer.cpp:233` with a genuine regression test (`RescanInheritsSelectedBand`) that RUNS in-container and fails without the fix ({2,1}). Both adversarial lenses independently cleared the concurrency/GL-lifecycle and logic surfaces.

Static analysis: cppcheck clean apart from pre-existing `shadowFunction` notes (locals
`width`/`height`/`band` shadow accessors — predates this change); line-length (<=100)
and trailing-whitespace clean (cpplint binary unavailable — checked manually). Claude
Adversarial: 2 passes (Lens A logic + Lens B systemic/GL/concurrency, Deep horizon).
Lens B traced the worker abort/join, GL context lifecycle, `pixels_loaded_`
release/acquire ordering, readSettings-before-GL, and destruction-during-switch and
found no race/leak/use-after-free; Lens A verified the rescan fix and that the new
regression test is genuine. Copilot: off (default). Plan adherence: exact, no scope
creep. Governance: ADR-0005 (this is its scoped band-select follow-up), ADR-0002/0003
— compliant.

### Findings
- [ ] (suggestion) Band QSettings round-trip (`readSettings`→`applyBand`, `writeSettings` `setValue("band")`) is untested — `test_gggs_persistence.cpp` covers visibility/colormap but never band; a "persisted band reverts to 1" regression would be silent — `src/camp2/raster/gggs_tile_layer.cpp:808`
- [ ] (suggestion) Non-uniform tile-set: `setBand`/`applyBand` validate against the front tile's `bandCount()` while the per-tile loop uses each tile's own count — the validation/application authorities diverge; invariant-gated (uniform stores never hit it), acknowledged in comments — tracking for the deferred non-uniform work — `src/camp2/raster/gggs_tile_layer.cpp:661`
- [ ] (suggestion) Shader `v <= 0.0` discard mis-ranges signed/uncertainty bands the picker now makes selectable; documented in-code and deferred to camp#122 — tracking only — `src/camp2/raster/gggs_tile_layer.cpp:62`

## Implementation
**Status**: complete
**When**: 2026-06-28 11:52 +0000
**By**: Claude Opus
**Round**: 4 address-findings (the single cheap actionable suggestion)

Addressed the round-4 pre-push suggestion: the band QSettings round-trip was
untested. Added two tests to `test/test_gggs_persistence.cpp` (TEST-ONLY; no
production code changed; the two deferred items — non-uniform validation
divergence and shader `v<=0` → camp#122 — left untouched):

- `BandRoundTrips` — over a 2-band synthetic GeoTIFF tile-set, switch to band 2
  via the public `setBand(2)`, persist with `writeSettings()`, then a fresh layer
  over the SAME directory (same itemID) `readSettings()` restores band 2. This is
  the regression a silent "band reverts to 1 on reload" would have slipped past.
- `BandDefaultRoundTrips` — the default round-trips with no spurious change: a
  layer that never sets a band writes/reads back band 1, and reading a cleared
  settings group also defaults to 1.

Mirrors the visibility round-trip pattern (a `TestableGggsTileLayer` subclass
exposes the protected `readSettings()`/`writeSettings()` hooks, asserted
synchronously without the deferred `itemConstructed()` timer) and reuses the
2-band GeoTIFF helper from `test_gggs_band_select.cpp`. The band-integer
round-trip is **GL-free** (`applyBand()` sets `band_` before any texture work and
skips GL when no context exists), so both tests **RUN — not SKIP — in-container**
(GDAL only, which is available in-container).

**RUN vs SKIP in-container**: both new tests RUN (verified via the gtest XML:
`status="run"`, `skipped="0"` for the `GggsPersistence` suite).

**Build**: `./ui_ws/build.sh camp` → `Summary: 1 package finished [39.9s]` —
clean (camp had only pre-existing `-Wunused-parameter`/deprecation warnings; the
lower layers `underlay_ws` (22 pkgs) and `core_ws` (35 pkgs) had to be built
first as their install spaces were empty in this worktree — both exit 0).

**Test**: `./ui_ws/test.sh camp` →
`Summary: 117 tests, 0 errors, 0 failures, 3 skipped` (the 3 skips are the
offscreen-GL tests, SKIP-not-FAIL in-container by design). `test_gggs_persistence`
suite: 7 tests, 0 failures, 0 skipped — including the 2 new band tests.

**Commit**: `1d4251f` test(camp#108): cover band QSettings round-trip (hooks ran,
no `--no-verify`). Not pushed; no PR opened (handoff contract — the host pushes).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-28 12:03 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-108 at `042f749`
**Mode**: pre-push
**Depth**: Deep (reason: GL-texture lifecycle + async-worker concurrency on a 200+ line change)
**Must-fix**: 0 | **Suggestions**: 2
**Round**: 5 | **Ship**: recommended — no must-fix in diff scope; production code is unchanged since the Round-4 approval (already cleared by two Deep adversarial passes), and the only new surface — the band QSettings round-trip tests — was independently verified as a genuine, non-vacuous regression test by Lens A.

Static analysis: cppcheck clean (the lone `unknownMacro` is the Qt `slots` parse limitation in a transitively-included header, not a finding in the changed code); line-length (<=100) and trailing-whitespace clean (cpplint binary unavailable — checked manually). Claude Adversarial: 2 passes (Lens A logic + Lens B systemic/GL/concurrency, Deep horizon). Lens A traced itemID coupling / readSettings→applyBand→writeSettings and confirmed `BandRoundTrips`/`BandDefaultRoundTrips` fail under a read- or write-side regression (non-vacuous); Lens B re-cleared the worker abort/join, atomic release/acquire pairing, and the readSettings→applyBand null-context path, and confirmed the band-switch makeCurrent-failure path correctly prevents stale-band rendering. Copilot: off (default). Plan adherence: exact, no scope creep (Round-4 finding closed; plan.md Tests synced). Governance: camp ADR-0005 (this is its scoped band-select follow-up), ADR-0002/0003 — compliant.

### Findings
- [ ] (suggestion) Pre-existing (byte-identical on `origin/jazzy`, out of this PR's diff): `releaseGL()` makeCurrent-failure branch skips the reset body then deletes the context, so the FBO/program/LUT/tile-textures destruct with no current context → leak + warnings; only fires on GL loss at process teardown (OS reclaims handles). Possible follow-up — `src/camp2/raster/gggs_tile_layer.cpp:594`
- [ ] (suggestion) `applyBand()` makeCurrent-failure path clears tile pixels without releasing the old-band texture; stale-render is correctly prevented (sets `gl_failed_`), so this is a texture leak on the rare GL-failure path only — ties to the pre-existing teardown behavior above — `src/camp2/raster/gggs_tile_layer.cpp:701`
- [ ] (tracking, deferred) Shader `v <= 0.0` discard mis-ranges signed/uncertainty bands → camp#122; non-uniform-tile validation authority divergence (front-tile vs per-tile count) — invariant-gated, never fires on a uniform store — `src/camp2/raster/gggs_tile_layer.cpp:62`

## Implementation
**Status**: complete
**When**: 2026-06-28
**By**: Claude Code Agent (Claude Opus)
**Scope**: camp#126 only (parent/leaf store-layer naming). camp2/directory rename
(#125) deliberately untouched — that is the separate later step.

Implements **camp#126**: a flat store layer's tree-view name is now its last two
path components ("parent/leaf", e.g. `sidescan/processed`) instead of just the leaf
folder, so two stores that share a leaf (`.../sidescan/processed` vs
`.../bathymetry/processed`) stay distinguishable in the tree.

- Added a file-local helper `displayName(const QString&)` in the anonymous
  namespace of `src/camp2/raster/gggs_tile_layer.cpp`: `QDir::dirName()` for the
  leaf (which trims a trailing slash), `QFileInfo(dir.path()).dir().dirName()` for
  the parent, returning `parent + '/' + leaf` — or just the leaf when the parent
  component is empty (a root-level dir like `/processed`) or `.`.
- The `GggsTileLayer` ctor now passes `displayName(directory)` to `map::Layer`
  instead of `QFileInfo(directory).fileName()`.
- **DISPLAY-name change only**: `directory_`, persistence (`GggsTileLayers/dirs`),
  and dedup all remain keyed by the full directory — unchanged.

**Test added** (`test/test_gggs_layer_name.cpp`, wired in CMakeLists mirroring the
`test_gggs_persistence` gtest block): constructs a `GggsTileLayer` over an empty
nested store path and asserts `objectName() == "sidescan/processed"`
(`ParentSlashLeaf`), plus the root-level fallback `objectName() == "processed"`
over `/processed` (`BareLeafFallback`). Both are **GL/GDAL-free** — an empty/absent
tile-set dir yields no tiles, so the ctor takes the `(no tiles)` branch and never
touches the offscreen GL path — so both **RUN — not SKIP — in-container** (verified
by running the binary directly: `2 tests from GggsLayerName ... PASSED`).

**Build**: lower-layer install spaces were empty in this worktree; only camp's
core deps were needed (`marine_sensor_msgs`/`grid_map_ros`/`nav2_msgs` are apt
packages in `/opt/ros/jazzy`), so `core_ws` was built `--packages-up-to
marine_autonomy marine_interfaces marine_ais_msgs` → `Summary: 3 packages finished`
(exit 0). Then `./ui_ws/build.sh camp` → `Summary: 1 package finished [2min 54s]` —
clean (camp had only pre-existing `-Wsign-compare`/`-Wunused-parameter` warnings).

**Test**: `./ui_ws/test.sh camp` →
`Summary: 120 tests, 0 errors, 0 failures, 3 skipped` (includes the 2 new
`GggsLayerName` tests, both RUN; the 3 skips are the offscreen-GL tests,
SKIP-not-FAIL in-container by design).

**Commit**: `c64f352` feat(camp#126): name store layers parent/leaf, not just the
leaf folder (hooks ran, no `--no-verify`). Not pushed (handoff contract — the host
pushes).

## Implementation
**Status**: complete
**When**: 2026-06-28
**By**: Claude Code Agent (Claude Opus)
**Scope**: camp#125 only — retire the camp2 executable + mechanically rename
`src/camp2/` → `src/camp_map/`. No behavior change to the shared library code; the
build + full test suite passing is the correctness gate.

Implements **camp#125**, completing the #59 migration: the `camp2` dev-sandbox
executable is retired now that `CCOMAutonomousMissionPlanner` runs entirely on the
shared map framework.

**Deleted**:
- `src/camp2/main/` entirely (`main.cpp`, `main.qrc`, `main_window.{cpp,h,ui}`) —
  the camp2 executable's app shell (`git rm -r`).
- The `add_executable(camp2 …)` target block in `CMakeLists.txt` plus its
  `target_include_directories` / `target_link_libraries(... Qt5::Test)` /
  `ament_target_dependencies` / `install(TARGETS camp2 …)`. `Qt5::Test` stays in
  the `find_package(Qt5 … Test …)` COMPONENTS list — still used by tests
  (`test_topic_bridge`, `test_color_map`, `test_map_model`, `test_gggs_flat_layer_spawn`).

**Renamed** (`git mv`, history preserved — 103 files renamed at 100% similarity):
`src/camp2/` → `src/camp_map/`. `#include` lines are rooted at the include dir
(e.g. `"map/map.h"`), so they were rename-safe; no source `#include` edits needed.

**Updated `CMakeLists.txt`**: every `src/camp2` path string → `src/camp_map`
(`CAMP_MAP_SOURCES`, `CAMP_MAP_ROS_SOURCES`, the `target_include_directories`
PUBLIC include dir, and all 15 per-test `target_include_directories`), plus the
narrative comments ("camp2's self-contained map core" → shared map core, "The
camp2 ROS layer set" → "The camp_map ROS layer set", the camp2-sandbox
QAbstractItemModelTester note). `grep -n camp2 CMakeLists.txt` → clean.

**Doc/comment sweep**:
- `.agents/README.md`: rewrote the Package Inventory (now **one** executable +
  two shared libs — camp2 row removed), the adoption-strategy paragraph (states
  the #59 migration is complete and the camp2 executable is **retired**), the
  Repository Layout (`src/camp_map/`), the catalog/map path refs, and the
  QSettings-org pitfall (dropped the live "camp2 sandbox" reference).
- Source comments in `src/camp/` and `src/camp_map/` that referred to the app or
  the old dir reworded `camp2` → `camp_map` (or "retired camp2 sandbox" where the
  historical fact matters: `roslink.cpp`, `test/test_map_model.cpp`). The broken
  `src/camp2/ros/node.h` path ref in `mainwindow.cpp` was fixed to `src/camp_map`.
  Namespaces (`camp::map::` etc.) were **not** touched — they never contained
  `camp2`.
- `Doxyfile`: `INPUT = src/camp2` → `src/camp_map` (was a now-broken path).
- `launch/camp_launch.py` confirmed to run `CCOMAutonomousMissionPlanner` with no
  camp2 reference.

**Final `grep -rn camp2` state**: the only live-tree hits remaining are deliberate
historical references to the **retired** camp2 sandbox (`test/test_map_model.cpp`
line 12 "(now-retired) camp2 sandbox app", `src/camp/roslink.cpp` "The retired
camp2 sandbox wired the same"). Left ALONE as past records, per the task:
`.agent/work-plans/issue-*/plan.md` historical entries, and `docs/decisions/`
ADRs + `docs/parity/markers.md` (immutable architecture/parity records that
describe camp2 as it was at decision time).

**Build**: lower layers were unbuilt in this worktree (shared symlinks to
`layers/main/{underlay,core}_ws`), so `underlay_ws` (`colcon build` → 22 packages
finished) and `core_ws` (35 packages finished) were built first. Then
`./ui_ws/build.sh camp` → `Summary: 1 package finished` — clean (only pre-existing
`-Wsign-compare`/`-Wunused-parameter` warnings). Verified with a **from-scratch**
rebuild (`rm -rf ui_ws/build/camp ui_ws/install/camp` then rebuild): produces
`CCOMAutonomousMissionPlanner` and **zero** `camp2` artifacts;
`grep -n 'add_executable(camp2' CMakeLists.txt` → none.

**Test**: `./ui_ws/test.sh camp` →
`Summary: 120 tests, 0 errors, 0 failures, 3 skipped` — identical to the pre-step
baseline (this is a rename, not a feature; the 3 skips are the offscreen-GL tests
that SKIP-not-FAIL in-container by design).

**Commit**: `675c6f1` refactor(camp#125): retire camp2 executable; rename
src/camp2 -> src/camp_map (single atomic commit — git mv + CMake/path/doc edits;
hooks ran, no `--no-verify`). Not pushed (handoff contract — the host pushes).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-28 14:12 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-108 at `568ca20`
**Mode**: pre-push
**Depth**: Deep (reason: GL-texture lifecycle + async-worker concurrency on a 200+ line change spanning the camp2->camp_map cross-layer rename)
**Must-fix**: 1 | **Suggestions**: 2
**Round**: 6 | **Ship**: continue — a new correctness/consequence must-fix surfaced from the camp#126 code (renaming the layer changes its QSettings itemID, silently resetting persisted visible/colormap/band on upgrade), cross-confirmed by both adversarial lenses and governance; it carries a migrate-vs-accept design choice so warrants resolution before push. Advisory — a single well-understood item Roland may also accept-and-ship with a comment fix; this review never blocks the ship.

Branch now bundles three issues: camp#108 (band picker, 5 prior clean rounds), camp#126 (parent/leaf layer naming), camp#125 (retire camp2 exe + rename src/camp2 -> src/camp_map, 103 files at 100% similarity). Static analysis: cppcheck clean apart from pre-existing `shadowFunction` notes (locals width/height/band shadow accessors — predates this change) and the cppcheck C-parser `namespace`/Qt-`slots` false errors; line-length (<=100)/trailing-whitespace clean. CMake rename verified: only the CCOMAutonomousMissionPlanner executable target remains, `grep camp2 CMakeLists.txt/Doxyfile` clean, all three new tests wired. Claude Adversarial: 2 passes (Lens A logic + Lens B systemic/GL/concurrency, Deep horizon) — both independently cleared the worker abort/join at every tiles_ mutation site, the atomic pixels_loaded_ release/acquire pairing, GL-context lifecycle (create/makeCurrent-fail/releaseGL/destroy-mid-load), and band/rescan/range-fold logic with no race/leak/UAF, and both converged on the persistence-key regression below. Copilot: off (default). Plan adherence: the #108 plan is implemented faithfully; camp#126/#125 are separately-tracked issues bundled on the branch, not #108 scope creep. Governance: ADR-0005 (#108 is its scoped band-select follow-up), ADR-0002 (camp#125 completes the #59 adoption migration), ADR-0003 (async load contract preserved) — compliant; the one MISSING consequence is camp#126's itemID/persistence-stability (the must-fix).

NOTE: did not run a fresh build/test this round (mature branch; relied on static analysis + two Deep adversarial reads). The last recorded build/test (camp#125 implementation entry) was `120 tests, 0 errors, 0 failures, 3 skipped` (the 3 are the offscreen-GL tests, SKIP-not-FAIL in-container by design).

### Findings
- [ ] (must-fix) camp#126 layer rename changes `objectName()` -> `itemID()` -> the per-layer QSettings key, so `visible`/`colormap`/`band` persisted on a prior (leaf-named) build are not found after upgrade and silently revert to defaults (layer comes back OFF/grayscale/band 1); the `GggsTileLayers/dirs` restore list is directory-keyed so the layer IS recreated, only its prefs are lost. Contradicts this branch's own main.cpp persistence-stability note. The test comment `test_gggs_layer_name.cpp:4-5` ("persistence stays keyed by the full directory") is wrong for the per-layer settings group. Fix: one-time migration of the old leaf-keyed group, OR consciously accept + document the reset and correct the comment — `src/camp_map/raster/gggs_tile_layer.cpp:419`
- [ ] (suggestion) `setBand()` `pixels_loaded_.store(false, std::memory_order_release)` — release ordering on the false store pairs with no acquire-of-false; real sync is the worker join. `relaxed` + a note, or keeping it for symmetry, reads clearer — `src/camp_map/raster/gggs_tile.cpp:144`
- [ ] (suggestion) Pre-existing idiom: locals `width`/`height`/`band` shadow accessor methods (`shadowFunction`); completeness only — `src/camp_map/raster/gggs_tile.cpp:34`
- [ ] (tracking, deferred) Shader `v <= 0.0` discard mis-ranges signed/uncertainty bands -> camp#122; non-uniform-tile validation authority divergence (front-tile vs per-tile count) — invariant-gated, never fires on a uniform store — `src/camp_map/raster/gggs_tile_layer.cpp:62`

## Implementation
**Status**: complete
**When**: 2026-06-28
**By**: Claude Code Agent (Claude Opus)
**Round**: 6 address-findings (camp#126 must-fix: directory-keyed per-layer persistence)

Addressed the Round-6 local-review **must-fix**: the camp#126 layer rename had
moved the per-layer QSettings group, which keyed on `itemID()` (parent +
`objectName()`). Two problems: (a) the leaf→parent/leaf rename reset persisted
`visible`/`colormap`/`band` on upgrade, and (b) the deeper clash Roland raised —
two stores whose directories resolve to the SAME parent/leaf display name (e.g.
`survey_a/bathymetry/processed` and `survey_b/bathymetry/processed`, both shown as
`bathymetry/processed`) collided on ONE settings group and clobbered each other's
prefs. **Decision (Roland): key per-layer persistence on the DIRECTORY (unique +
stable), NO migration** — a one-time reset of currently-saved prefs is accepted
(pre-deployment). Display stays `parent/leaf`. Commit `2fcfeb6`.

1. **`settingsKey()` seam** (`map/map_item.h`) — added
   `virtual QString settingsKey() const { return itemID(); }` on `MapItem`.
   Default-returns `itemID()`, so EVERY existing layer type is behavior-preserving.
2. **Routed the settings group through it** — `map::Layer::readSettings()`/
   `writeSettings()` (`map/layer.cpp`) and `GggsTileLayer::readSettings()`/
   `writeSettings()` (`raster/gggs_tile_layer.cpp`) now `beginGroup(settingsKey())`
   instead of `beginGroup(itemID())`, so the base (opacity/visible) and derived
   (colormap/band) groups share one identity-stable key.
3. **`GggsTileLayer::settingsKey()` override** (`raster/gggs_tile_layer.{h,cpp}`) —
   returns `"dir:" + QUrl::toPercentEncoding(QDir(directory_).absolutePath())`: a
   single FLAT key (every `/` → `%2F`, no deep nested group tree), readable and
   collision-free across full paths. Documented why identity is the directory, not
   the display name, and that no migration is done.
4. **Display unchanged** — `displayName()`/`objectName()` stay `parent/leaf`; the
   tree label is untouched. Only the persistence key moved off the name.

**Tests** (the consequence — covered):
- Fixed the now-wrong comment in `test/test_gggs_layer_name.cpp:1-9` (per-layer
  persistence is keyed on the DIRECTORY via `settingsKey()`, decoupled from the
  display name — not "keyed by the full directory" as a vague aside that conflated
  it with the `GggsTileLayers/dirs` restore list).
- Added `GggsPersistence.SameDisplayNameDistinctPersistence`
  (`test/test_gggs_persistence.cpp`): two `GggsTileLayer`s over DIFFERENT
  directories that resolve to the SAME `objectName()`/`itemID()` get DISTINCT
  `settingsKey()`; set `visible`/`colormap`/`band` on one, persist, and confirm the
  other's persisted `visible`/`band` are independent. **GL-free** (GDAL-only 2-band
  synthetic tiles; `band_` is set before any texture work) so it **RUNS — not SKIPs
  — in-container**. **Verified genuine**: with the routing reverted to `itemID()`,
  the test FAILS (`a` reads `b`'s clobbered `visible=false`/`band=1`); with the fix
  it PASSES, while `BandRoundTrips` (same dir → same key) still passes.
- Existing `test_gggs_persistence` band/colormap/visible round-trips still pass
  (same directory → same `settingsKey()` → round-trips correctly).

**Minor review suggestions**: added a clarifying comment beside
`gggs_tile.cpp` `setBand()`'s `pixels_loaded_.store(false, release)` (kept release
for symmetry; noted the real cross-thread sync is the worker abort+join the caller
performs, and that relaxed would be equally correct). The `shadowFunction` locals
and the shader `v<=0` / non-uniform-tile divergence stay deferred (camp#122 /
invariant-gated), per the task — untouched.

**Build/test result** (verified in-container):
- Build: `./ui_ws/build.sh camp` → `Summary: 1 package finished` — clean (only
  pre-existing `-Wdeprecated`/`-Wsign-compare`/`-Wunused-parameter` warnings, no
  errors). Lower-layer install spaces were empty in this worktree; only camp's core
  deps needed building: `core_ws --packages-up-to marine_autonomy marine_interfaces
  marine_ais_msgs` → `Summary: 3 packages finished` (exit 0).
- Test: `./ui_ws/test.sh camp` → **121 tests, 0 errors, 0 failures, 3 skipped**
  (was 120; +1 from the new clash test). The 3 skips are the offscreen-GL tests
  that SKIP-not-FAIL in-container by design. The new
  `SameDisplayNameDistinctPersistence` RAN and passed; the `GggsPersistence` suite
  is 8 tests, all OK.

**Commit**: `2fcfeb6` fix(camp#126): key store-layer persistence on directory, not
display name (hooks ran, no `--no-verify`). Not pushed; no PR opened (handoff
contract — the host pushes).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-28 14:47 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-108 at `6d99daf`
**Mode**: pre-push
**Depth**: Deep (reason: GL-texture lifecycle + async-worker concurrency on a 200+ line change spanning the camp2->camp_map rename)
**Must-fix**: 0 | **Suggestions**: 2
**Round**: 7 | **Ship**: recommended — no must-fix; the only new production code since the round-6 approval is the camp#126 settingsKey() directory-keying fix (`2fcfeb6`), and both Deep adversarial passes independently cleared the concurrency (abort/join, atomic release/acquire), GL lifecycle (null/makeCurrent-fail/success paths), and the persistence re-keying, while the `SameDisplayNameDistinctPersistence` regression test was verified non-vacuous. Remaining items are low/deferred.

Production code is UNCHANGED since `2fcfeb6` (HEAD `6d99daf` adds only progress.md), so no rebuild this round — the last recorded build/test (round-6 address-findings) stands: `121 tests, 0 errors, 0 failures, 3 skipped` (the 3 are offscreen-GL tests, SKIP-not-FAIL in-container by design). Static analysis: cppcheck clean apart from the known Qt `slots`/`unknownMacro` C-parser limitation; line-length (<=100) and trailing-whitespace clean on changed lines (the lone 101-col line and trailing-WS hits are pre-existing carried-over rename regions, not lines this change touches; cpplint binary unavailable — checked manually). Field-mode host: `git fetch origin/jazzy` failed (host-key); reviewed against the local `origin/jazzy` ref at `9840b89` (may be stale). Claude Adversarial: 2 passes (Lens A logic + Lens B systemic/GL/concurrency, Deep horizon) — both converged on 0 must-fix; Lens A and the lead independently flagged the incomplete settingsKey() seam adoption (cross-confirmed). Copilot: off (default). Plan adherence: exact; camp#126/#125 are separately-tracked issues bundled on the branch, not #108 scope creep. Governance: camp ADR-0005 (#108 is its scoped band-select follow-up), ADR-0002 (camp#125 completes the #59 adoption migration), ADR-0003 (async load contract preserved) — compliant; the camp#126 itemID/persistence-stability consequence (the round-6 must-fix) is now Done via settingsKey() + a genuine regression test.

### Findings
- [ ] (suggestion) camp#126 `settingsKey()` seam not applied to sibling persisting layers — `raster_layer.cpp` and `grid_map.cpp` still hand-roll `beginGroup(itemID())`; harmless today (`settingsKey()==itemID()` for them) but the re-keying is incomplete — `src/camp_map/raster/raster_layer.cpp:341` / `src/camp_map/ros/grids/grid_map.cpp:229`
- [ ] (suggestion) Directory identity mismatch: dedup/de-persist compare the raw `directory_` string while `settingsKey()` normalizes via `QDir::absolutePath()`; a trailing-slash/relative variant would dedup as distinct yet collide on one settings group (low likelihood — only source is `QFileDialog`, absolute + no trailing slash) — `src/camp_map/raster/gggs_tile_layer.cpp:873`
- [ ] (tracking, deferred) Shader `v <= 0.0` discard mis-ranges signed/uncertainty bands the picker now makes selectable → camp#122; non-uniform-tile validation-authority divergence (front-tile vs per-tile count) — invariant-gated, never fires on a uniform store — `src/camp_map/raster/gggs_tile_layer.cpp:62`

## Implementation
**Status**: complete
**When**: 2026-06-28 15:06 +00:00
**By**: Claude Code Agent (Claude Opus)
**Round**: 7 (address-findings)
**Commit**: `13ee218` fix(camp#126): canonicalize store directory so dedup and settingsKey agree

Closes the round-7 edge suggestion: `GggsStoreSource::instantiate()` dedup'd on the
raw `directory_` string while `GggsTileLayer::settingsKey()` normalized via
`QDir::absolutePath()`, so two string variants of one path (trailing slash / relative)
would dedup as DISTINCT layers yet collide on ONE settings group. Normalized the
directory to a single canonical absolute form at the boundary and keyed every
authority off it.

**Canonicalization applied at**:
- `src/camp_map/raster/gggs_store_source.cpp` `instantiate()`: compute
  `const QString canonical = QDir(key).absolutePath();` up front, then use `canonical`
  for (a) the dedup compare against `existing->directory()`, (b) `new GggsTileLayer(layers, canonical)`,
  and (c) the persisted `GggsTileLayers/dirs` restore list (`contains`/`append`/`setValue`).
- `src/camp_map/raster/gggs_tile_layer.cpp` ctor: `directory_(QDir(directory).absolutePath())`
  so any other caller (e.g. `createDefaultLayers()` restoring from the dirs list) is
  consistent. `loadDirectory()`/`rescan()` read `directory_` via `QDir(...)` which is
  path-agnostic, so the absolute path works unchanged.
- `settingsKey()`: dropped the now-redundant `QDir(directory_).absolutePath()` — keys
  `directory_` directly since the ctor already canonicalizes it (idempotent).
- `onRemovedFromMap()` de-persist (`dirs.removeAll(directory_)`) now matches because the
  dirs list and `directory_` are both canonical.

Scope: identity-normalization only. Band/colormap/persistence behavior otherwise
unchanged; deferred items (shader v<=0 / camp#122, non-uniform tiles) and the sibling
`settingsKey()` adoption (raster_layer/grid_map) left untouched per the task.

**Test added**: `GggsPersistence.NonCanonicalDirNormalizes` (test_gggs_persistence.cpp) —
a layer built from a trailing-slash variant yields the same `directory()`/`settingsKey()`
as the canonical form; `instantiate(variant)` then `instantiate(canonical)` dedups to ONE
layer; the persisted dirs list holds the canonical path. GL/GDAL-free (empty tile-set dir).
Verified non-vacuous: ran (status="run" result="completed", time 0.028s), no failure child.

**Build/test result** (verified in-container):
- Lower-layer deps were empty in this worktree; rebuilt first:
  `core_ws colcon build --packages-up-to marine_autonomy marine_interfaces marine_ais_msgs`
  → `Summary: 3 packages finished` (only pre-existing warnings).
- Build: `./ui_ws/build.sh camp` → `Summary: 1 package finished` — clean (only pre-existing
  `-Wunused-parameter`/`-Wdeprecated`/`-Wsign-compare` warnings, no errors).
- Test: `./ui_ws/test.sh camp` → **122 tests, 0 errors, 0 failures, 3 skipped** (was 121;
  +1 from the new canonicalization test). The 3 skips are the offscreen-GL tests that
  SKIP-not-FAIL in-container by design.

**Commit**: `13ee218` (hooks ran, no `--no-verify`). Not pushed; no PR opened (handoff
contract — the host pushes).
