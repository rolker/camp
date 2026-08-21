---
issue: 195
---

# Issue #195 — GggsTileLayer: no viewport-scoped retention or residency budget — panning accumulates tiles without bound (camp#153 precedent)

## Plan Authored
**Status**: complete
**When**: 2026-08-21 15:02 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-195/plan.md` at `b70311f`
**Branch**: feature/issue-195 at `b70311f`
**Phases**: single (one PR, ~5 atomic commits)

### Design decisions taken (host constraints evaluated, not inherited)
- **Fold-to-parent rejected.** GGGS tiles are file-backed; `resetPixels()` +
  `releaseGL()` is already a complete release and the existing demand-driven
  loader re-reads on pan-back, so the reload path is free. `SonarLiveCacheLayer`
  folds only because its data arrives once over ROS. A CAMP-side pyramid would
  also manufacture a derived product `uma-ADR-0010` D9 deliberately withholds
  from `chart`/`reference`. Cost — an evicted area blanks rather than degrades —
  is bought back by never evicting the coarsest available level.
- **Count budget**, adaptive cap `clamp(3 × protected_count, 32, 64)` (MapTiles /
  camp#98 form), which makes "budget ≥ current-frame set" true by construction.
  Fixed 960×960 tiles cost ~7.03 MiB each (CPU buffer retained past GPU upload
  per camp#180), so 64 ≈ 450 MiB.
- **`views().first()` designed around, not inherited.** `paint()` already
  receives the painting viewport widget; `deriveViewportClip()` gains an optional
  `widget` param and recovers the painting view, falling back to `views().first()`.
  Protection accumulates per `paint()`; eviction is deferred + debounced so one
  pass sees the union of all views (camp#150-safe).
- **Structural protection** via a new `TileResidency` splice-partition type
  (two `std::list`s, O(1) per frame/touch) whose eviction API cannot reach the
  protected partition — D4's sentinel-node guarantee without an intrusive list.
- **Hysteresis**: evict to `0.75 × cap`, plus a per-kick loader volume cap.
- **Degrade lever** included: `pressure_bias_` steps the LOD selection coarser
  when the protected set alone exceeds the cap, with a cool-down and a visible
  status; a no-op with a visible status where no coarser level exists.
- **camp#194 hole-coverage rule preserved**: finer tiles covering a
  `loadFailed()` tile are protected from budget eviction too.

### Open questions
- [ ] Confirm cap defaults (`kMultiplier=3`, `kMinCap=32`, `kMaxCap=64` ≈ 450 MiB) and the `GggsTileLayers/max_resident_tiles` settings key.
- [ ] Confirm the degrade lever belongs in this PR rather than a follow-up — it is the part most likely to oscillate in the field.
- [ ] Confirm the `deriveViewportClip()` `widget` parameter (touching `RasterLayer` and `SonarLiveCacheLayer` call sites) is acceptable scope here versus a separate camp#150 PR.
