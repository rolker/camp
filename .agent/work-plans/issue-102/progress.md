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
