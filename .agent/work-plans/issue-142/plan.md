# Plan: Per-layer colormap range override (PR1 — backend + numeric UI)

## Issue

https://github.com/rolker/camp/issues/142

## Context

The `RasterGlRenderer` shader's `u_min`/`u_max` uniforms are fed from each
layer's accumulated `data_min_`/`data_max_` (auto-range over all data).  A
single outlier (e.g. backscatter band 2: max=925, mean=0.16) collapses the
useful color range to a single shade.

**Dependencies merged:**
- camp#134 — unified `RasterGlRenderer` with `u_min`/`u_max` uniforms; three
  layer callers: `GggsTileLayer`, `SonarLiveCacheLayer`, `RasterLayer`.
- camp#141 — migrated GPU LUT path to `marine_colormap`; `RasterGlRenderer::
  setColormap()` now takes `std::string`; internal `camp::map::ColorMap` deleted.
  `marine_colormap::RangeModel` (in `marine_colormap/transfer.hpp`) is available.

**This branch** (`feature/issue-142`) diverged from camp at camp#134; it does NOT
yet include camp#141 changes.  First implementation step is a rebase.

**camp#138** (live-cache auto-range refold) is NOT yet merged.  Both issues
touch `SonarLiveCacheLayer`'s range; they are orthogonal — #138 changes how
`data_min_`/`data_max_` accumulate; this PR adds an override on top that is
applied at render time, transparent to the accumulation logic.

## Approach

**This PR is PR1 of a staged plan.** PR2 (separate run) embeds
`marine_colormap_widgets::ColormapLegendWidget` (drag handles).

1. **Rebase** `feature/issue-142` onto camp `main` (which includes #141).
   Resolve any conflicts (the `ColorMap`-type-enum → string-name API change is
   the primary conflict surface).

2. **Add `marine_colormap::RangeModel range_model_`** to each of the three
   layer classes.  `marine_colormap` is already a linked dependency (camp#141).

3. **Wire `range_model_` into `renderImage()`** in each layer: replace the
   hardcoded `float(data_min_)` / `float(data_max_)` args to
   `renderer_.renderToImage(...)` with the model's resolved `lo()`/`hi()`.
   In Auto mode the model tracks `data_min_`/`data_max_` via `update_auto()`.

4. **Keep auto-range methods unchanged** (`resetAutoRange`/`foldAutoRange` in
   `SonarLiveCacheLayer`; `tilesReady` fold in `GggsTileLayer`).  After each
   fold call `range_model_.update_auto(data_min_, data_max_)` so the Auto
   resolved range stays current.

5. **Add public API** to each layer:
   - `setRangeOverride(float lo, float hi)` — calls `range_model_.set_manual()`,
     persists, re-renders.
   - `resetRangeToAuto()` — calls `range_model_.reset()`, persists, re-renders.
   - `rangeMode() const` → `marine_colormap::RangeMode`  (for tests + PR2 binding).

6. **Context-menu additions** in each layer's `contextMenu()`, following the
   existing colormap/band submenu pattern.  Add a "Colormap range" submenu with:
   - "Set range…" → two sequential `QInputDialog::getDouble()` calls (one for lo,
     one for hi, pre-filled with current `lo()`/`hi()`); calls `setRangeOverride`.
   - "Reset to auto" → calls `resetRangeToAuto()`.
   The submenu is only added for scalar layers (gate on the same condition as the
   colormap submenu; `RasterLayer` already gates on `is_scalar_`).

7. **Persist/restore** via the existing `settingsKey()` pattern in each layer's
   `readSettings()`/`writeSettings()`:
   - `"range_mode"` → `"auto"` | `"manual"`
   - `"range_min"` → float
   - `"range_max"` → float
   All under `settings.beginGroup(settingsKey())`.

8. **Uncertainty band default — deferred.**  Band semantics (#104) are not
   resolved; GGGS bands are 1-indexed with no type classification.
   `SonarLiveCacheLayer` bands are named, but no "uncertainty" naming convention
   is established.  PR1 delivers the **mechanism** (operator sets quality palette
   + manual range); an auto-default by band name is a follow-on after #104.

9. **Tests** in a new `test/test_range_persist.cpp`, modelled on
   `test_gggs_persistence.cpp`.  All tests must be GL-free (the range model is
   pure data; `renderImage()` is not called):
   - `RangeDefaultIsAuto`: fresh `TestableGggsTileLayer` → `rangeMode()` == Auto.
   - `SetOverrideSwitchesToManual`: `setRangeOverride(1, 5)` → mode Manual,
     `lo() == 1`, `hi() == 5`.
   - `RangeRoundTrips`: write Manual [1,5] → `writeSettings()` → fresh layer →
     `readSettings()` → mode Manual, lo/hi correct.
   - `AutoRoundTrips`: default auto → `writeSettings()` → fresh layer →
     `readSettings()` → mode Auto.
   - `ResetToAutoRestoresMode`: set manual, reset → mode Auto.

## Files to Change

| File | Change |
|------|--------|
| `src/camp_map/raster/gggs_tile_layer.h` | Add `range_model_` field; add `setRangeOverride`/`resetRangeToAuto`/`rangeMode` |
| `src/camp_map/raster/gggs_tile_layer.cpp` | `tilesReady`: call `range_model_.update_auto`; `renderImage`: use `range_model_` lo/hi; add contextMenu range submenu; add persist keys in `readSettings`/`writeSettings` |
| `src/camp_map/ros/live_coverage/sonar_live_cache_layer.h` | Same additions as GggsTileLayer |
| `src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp` | `foldAutoRange`: call `range_model_.update_auto`; `renderImage`: use `range_model_` lo/hi; add contextMenu range submenu; add persist keys |
| `src/camp_map/raster/raster_layer.h` | Same additions as GggsTileLayer; scalar-only gate |
| `src/camp_map/raster/raster_layer.cpp` | `imageReady`: call `range_model_.update_auto`; `renderImage`: use `range_model_` lo/hi; add contextMenu range submenu; add persist keys |
| `test/test_range_persist.cpp` | New test file for range persist/restore round-trip (GL-free) |
| `CMakeLists.txt` | Add `test_range_persist` gtest target; link `marine_colormap::marine_colormap` |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Improve incrementally | PR1 is backend + numeric UI only; interactive widget is PR2. Both are independently reviewable. |
| Only what's needed | No new class hierarchy; `RangeModel` is the existing type from `marine_colormap`. `QInputDialog` reuses the already-present Qt dialog facility. |
| A change includes its consequences | Tests for persist/restore ship with the PR; manual verification of context-menu is done at host (camp cannot build in-container). |
| Human control and transparency | Default stays Auto; override is explicit; reset-to-auto is always available; persisted state is visible via the "Set range…" pre-fill. |
| Capture decisions, not just implementations | Uncertainty-band deferral is recorded here; camp#138 sequencing is noted; no ADR is required for this PR (range override is a narrowly-scoped operator control, not a new UI topology). |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| camp ADR-0007 (RasterFieldSource unified renderer) | Yes | Override flows through `renderToImage(lo, hi)` — the single GPU-path insertion point; no per-layer shader duplication. |
| camp ADR-0008 (marine_colormap LUT bake) | Yes | Override goes to `u_min`/`u_max` only, not into the LUT. `RangeModel.lo()`/`.hi()` are passed as `renderToImage` args exactly as `data_min_`/`data_max_` were before. |
| camp ADR-0005 (stores browser / flat display layers) | Watch | Context-menu additions follow the existing colormap/band submenu pattern; no new menu topology. |
| workspace ADR-0013 (progress.md vocabulary) | Yes | `## Plan Authored` entry written to progress.md. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `renderImage()` args in all three layers | Tests must still pass without GL (range model is pure data) | Yes |
| `readSettings`/`writeSettings` in all three layers | The persist round-trip test pins the new keys | Yes |
| camp#138 lands after this PR | `foldAutoRange` may change internals; the `range_model_.update_auto` call site will need review at merge | No — sequencing note only |

## Open Questions

- **camp#138 sequencing**: If camp#138 is in-flight at implement time, check that
  its `foldAutoRange` changes don't conflict with the `range_model_.update_auto`
  call site.  The override is transparent to accumulation; no functional conflict
  is expected, but a diff review is warranted at merge.

## Estimated Scope

Two PRs total (#142): this is **PR1** ("Part of #142").  Single PR,
independently reviewable, does NOT close the issue.  PR2 (colorbar widget embed)
closes #142.
