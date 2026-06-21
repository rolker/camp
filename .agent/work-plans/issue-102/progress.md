---
issue: 102
---

# Issue #102 — GGGS store layer: lazy/async tile load + default-off tile-sets + persisted visibility

## Issue Review
**Status**: complete
**When**: 2026-06-20 21:20 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Issue**: #102
**Comment**: https://github.com/rolker/camp/issues/102#issuecomment-4760503965
**Scope verdict**: well-scoped

Follow-up to merged camp#90; part of unh_marine_autonomy#171/#175. Five cohesive
changes (split GggsTile extent/pixel; default-off tile-sets; async load-on-visible
mirroring RasterLayer's QtConcurrent path; persisted per-layer visibility;
QFileSystemWatcher) toward one goal — open a large store without freezing the GUI
thread. LOD/visible-region render, producer pyramids, and live ROS transport are
explicitly out of scope (separate issues). Right repo (rolker/camp, GitHub-origin,
default branch `jazzy`). Strong in-repo precedent: RasterLayer already does
sync-extent (ctor) + async-pixel (worker) per camp ADR-0003 §3; this generalizes
that to GggsTile.

### Actions
- [ ] Add gtests (camp has no CI/pre-commit — the suite is the only gate): (a) tile
      extent/boundingRect valid before pixels load; (b) default-off visibility
      persistence round-trip.
- [ ] OQ1 — Keep GDAL `RasterIO` on the QtConcurrent worker; keep the `QOpenGLTexture`
      upload (`gggs_tile.cpp:86-104`) on the render/paint path only (current-context
      requirement). Never call `texture()`/`allocateStorage` off the render thread.
- [ ] OQ2 — Define the "pixels not ready yet" path for the offscreen-FBO render
      (`GggsTileLayer::renderImage`): skip/partial-render an in-flight tile and
      repaint on completion (today `data_` is present at `texture()` time; with
      deferral it may be empty).
- [ ] OQ3 — Flip visibility default to off via a `readSettings()` override with a
      `false` fallback in the GGGS tile-set leaf layer — NOT a ctor `setVisible(false)`
      (overwritten: `MapItem::itemConstructed()` calls `readSettings()` via
      `QTimer::singleShot(0,...)` after the ctor; `Layer::readSettings` defaults
      `visible` to `true`, `layer.cpp:101-102`), and NOT a base-class change (would
      wrongly default charts/AIS off). Confirm it affects tile-set leaves, not
      grouping `GggsStoreLayer` nodes.
- [ ] OQ4 — Port RasterLayer's abort/join dtor contract verbatim (set `abort_flag_`
      under mutex + `waitForFinished()`) so a layer destroyed mid-load can't outlive
      `this`.
- [ ] OQ5 — Decide `QFileSystemWatcher` granularity (recursive/per-dir watch limits
      over a deep multi-epoch store) and reload semantics (incremental add vs rebuild;
      tolerate a half-written tile firing mid-write — atomic-write producer half is a
      separate issue).
- [ ] Update the stale TODO at `background_manager.cpp:70-73` and the `gggs_tile.h:15-20`
      doc comment (ctor no longer reads CPU sample data) when the lazy path lands.
- [ ] Record the GL-upload-vs-async-GDAL split decision (ADR addendum or PR/issue note)
      and note whether default-off-for-tile-sets departs from ADR-0003 §4's implied
      default-on visibility.

## Plan Authored
**Status**: complete
**When**: 2026-06-20 21:20 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-102/plan.md` at `5369945`
**Branch**: feature/issue-102 at `5369945`
**Phases**: single PR (~6 atomic commits)

### Open questions
- [ ] OQ-A — `QFileSystemWatcher` scale: per-dir non-recursive watches over a deep
      multi-epoch store could hit OS inotify limits; cap watch count now (root +
      active epochs) or defer until stores get big?
- [ ] OQ-B — Headless gtest for "skip in-flight tile, repaint on completion" may
      need GL/event-loop scaffolding the current harness lacks; is asserting the
      non-GL load state machine + range folding sufficient instead?

## Plan Review
**Status**: complete
**When**: 2026-06-20 21:44 -04:00
**By**: Claude Code Agent (Claude Opus 4.8)

**Plan**: `.agent/work-plans/issue-102/plan.md` at `5369945`
**PR**: PR-less
**Branch**: feature/issue-102
**Verdict**: approve-with-suggestions

### Findings
- [ ] (suggestion) Per-layer single-`QFutureWatcher` over a multi-tile `QtConcurrent::run`
      is a faithful structural port of `RasterLayer` (one watcher / one future /
      abort-between-units / dtor join + re-launch guard), but the unit of abort shifts
      from scanline (RasterLayer) to whole-tile. Spell out in the layer worker that the
      abort check is between tiles AND that a `tilesReady()` fold + a `paint()` lazy-kick
      cannot launch a second `loadTiles()` while one is in flight — port `loadFile`'s
      `isRunning() → abort+join` re-launch guard verbatim, not just the dtor join. The
      plan states this (Approach 2) but the Files-to-Change row for `gggs_tile_layer.cpp`
      omits the re-launch guard from its checklist — add it. — `plan.md:42-45,96`
- [ ] (suggestion) The worker mutates each shared `GggsTile`'s pixel buffer
      (`loadPixels()`) off-thread while `tilesReady()` reads `dataMin/Max` and
      `texture()` runs on the paint thread. The dtor/re-launch join is the only barrier
      — confirm in implementation that no tile touched by an in-flight worker is read on
      the paint path before `tilesReady()` fires (the "in-flight tile → `texture()`
      returns null → skip in `renderImage`" path, Approach 3, must gate on a per-tile
      `pixelsLoaded()` flag set only after the worker completes and the fold runs, never
      mid-write). — `plan.md:50-53`
- [ ] (suggestion) OQ-B is the right call: the non-GL load state machine
      (loading/loaded flags + incremental range fold) is headless-testable and is the
      highest-risk logic; the GL skip/repaint path is already covered by the existing
      self-skipping `test_gggs_render` harness. Recommend asserting the state machine in
      a gtest and documenting the render path as manually verified — don't block on
      GL/event-loop scaffolding. — `plan.md:101,136-140`
- [ ] (suggestion) OQ-A (watcher inotify scale): defer the watch-count ceiling. The
      incremental-add, per-discovered-dir design is sound for the Massabesic-scale store
      this lands against; a root+active-epoch cap is premature until stores grow. Note
      the deferral in the PR so it isn't lost. — `plan.md:132-135`

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-20 23:06 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Branch**: feature/issue-102 at `267e7ea`
**Mode**: pre-push
**Depth**: Deep (reason: async/threading refactor, ~804 lines)
**Round**: 1
**Verdict**: changes-requested
**Ship**: continue (must-fix data race in the target rescan/streaming path)
**Static analysis**: limited (cpplint not installed; clean compile) | **Claude Adversarial**: 2 passes (Lens A logic + Lens B concurrency) — both cross-confirmed the race | **Copilot**: off
**Must-fix**: 2 | **Suggestions**: 2

### Findings
- [ ] (must-fix) Worker-vs-paint **data race** on `GggsTile::pixels_loaded_` + `data_`: `loadPixels()` writes them on the QtConcurrent worker (gggs_tile.cpp:101-102) with no barrier; the GUI paint reads `pixelsLoaded()`/`texture()`/`data_` (gggs_tile_layer.cpp:431,452). Safe on first load only via the incidental crossed-range gate (line 493); **races on the `rescan()` re-kick** (range already valid → gate open) = the live-streaming case. Fix: make `pixels_loaded_` `std::atomic<bool>` with release(after data_ move)/acquire(paint read), OR make `tilesReady()` (post-join, GUI) the sole writer of a layer-side paintable set. — `gggs_tile.cpp:101-102` / `gggs_tile_layer.cpp:431,452,493`
- [ ] (must-fix) Comments assert a barrier that doesn't exist ("flag set ONLY in tilesReady() on the GUI thread after the worker finished") — it's set on the worker in `loadPixels()`. Correct the comments AND make the invariant real (pairs with #1). — `gggs_tile_layer.h:131-134`, `gggs_tile_layer.cpp:238-241,428-430`
- [ ] (suggestion) All-failed / all-NoData enabled tile-set → silent blank (range stays crossed, status cleared); show "(no data)"/error so the operator gets a signal. — `gggs_tile_layer.cpp:269`
- [ ] (suggestion) `rescan()`/`loadTiles()` `waitForFinished()` blocks the GUI thread for a full large-tile `RasterIO` (whole-tile abort granularity vs RasterLayer's scanline); fine at current scale, note/finer-abort for large tiles. — `gggs_tile_layer.cpp:162,223`

Lifecycle/teardown confirmed solid by both lenses (dtor abort+join, tiles_-mutation guarding, QFileSystemWatcher parented/serial, single finished connect).

## Implementation
**Status**: complete
**When**: 2026-06-20 23:45 -0400
**By**: Claude Code Agent (Claude Opus 4.8)

**Branch**: feature/issue-102
**Addresses**: the latest Local Review (Pre-Push) round 1 (2 must-fix + 2 suggestions)
**Build**: `./ui_ws/build.sh camp` — clean (only pre-existing Qt `setData` deprecation warnings)
**Test**: `./ui_ws/test.sh camp` — 72 tests, 0 failures, 1 skipped (GL render self-skip, no offscreen GL)

### Findings addressed
- [x] (must-fix #1) **Worker-vs-paint data race** on `GggsTile::pixels_loaded_`/`data_`.
      Made `pixels_loaded_` a `std::atomic<bool>` (added `<atomic>` to `gggs_tile.h`).
      `loadPixels()` now stores `true` with `memory_order_release` AFTER the `data_`
      move (and the load-guard read uses acquire); `pixelsLoaded()` loads with
      `memory_order_acquire`. This establishes happens-before (worker `data_`/range
      write → release store → acquire load → paint `data_`/`texture()` read), closing
      the race on both first load and the `rescan()` re-kick (where the crossed-range
      gate is already open). Confirmed the only paint-path reads of tile pixel state
      (`renderImage` `pixelsLoaded()`→`texture()`) go through the acquire check;
      `tilesReady()` reads `dataMin/Max` post-join (already safe). — `gggs_tile.h`,
      `gggs_tile.cpp`
- [x] (must-fix #2) Corrected the three comments that wrongly claimed the flag is set
      "ONLY in tilesReady() on the GUI thread" — it is set on the worker. They now
      describe the real atomic release/acquire publication invariant. —
      `gggs_tile_layer.h` (~131-138), `gggs_tile_layer.cpp` `loadTilesWorker` (~239-248),
      `renderImage` skip-gate (~431-437)
- [x] (suggestion #3) `tilesReady()` now calls `setStatus("(no data)")` when the
      folded range is still crossed (every loaded tile all-NoData/failed) instead of
      clearing the status, so an enabled-but-empty tile-set signals the operator. —
      `gggs_tile_layer.cpp` `tilesReady`
- [x] (suggestion #4) Added a comment at both `waitForFinished()` joins
      (`rescan()` + `loadTiles()`) noting the abort granularity is whole-tile (a large
      in-flight `RasterIO` blocks the GUI thread until that tile finishes), acceptable
      at current scale, finer sub-tile abort is a follow-up. — `gggs_tile_layer.cpp`

### Tests
Added two non-GL regression tests to `test/test_gggs_tile.cpp` pinning the race's
contract without GL/event-loop scaffolding:
- `PixelsLoadedFalseUntilLoadCompletes` — `pixelsLoaded()` stays false (and the range
  stays the crossed sentinel) until `loadPixels()` completes; after, flag true AND
  range consistent.
- `PixelsPublishedToObserverThread` — a worker thread `loadPixels()` while an observer
  thread spins on `pixelsLoaded()` exactly as the paint gate does; asserts the released
  range is fully visible/consistent the moment the acquire flag reads true (a
  non-atomic/no-barrier flag could let the observer see a torn/stale range).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-20 23:22 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Branch**: feature/issue-102 at `fbb2bb4`
**Mode**: pre-push
**Depth**: Deep (re-review of the round-1 race fix)
**Round**: 2
**Verdict**: approved
**Ship**: recommended
**Claude Adversarial**: 1 focused concurrency re-review (traced the atomic release/acquire publication end-to-end)
**Must-fix**: 0 | **Suggestions**: 2 (nits)

### Findings
- [x] (round-1 must-fix) Worker-vs-paint data race — CLOSED: `pixels_loaded_` is `std::atomic<bool>`, release-store after the `data_`/range writes, acquire-load gating every paint-path read of pixel state (`renderImage`→`texture()`, `tilesReady`→dataMin/Max); no bypass exists (grep-confirmed). First-load path + auto-range + default-off unchanged.
- [x] (round-1 must-fix) Misleading "set only on GUI thread" comments — corrected to the real acquire/release invariant.
- [x] (round-1 suggestions) "(no data)" status on all-crossed fold + whole-tile abort-granularity comments — applied.
- [ ] (nit) `PixelsLoadedFalseUntilLoadCompletes` is single-threaded — pins the API contract but would pass on a plain bool; comment overstates what it guards.
- [ ] (nit) `PixelsPublishedToObserverThread` is a probabilistic cross-thread test — real teeth only under TSan; document that.

Tests: 72, 0 failures, 1 skipped (GL self-skip). Production fix is correct + complete; safe to push.
