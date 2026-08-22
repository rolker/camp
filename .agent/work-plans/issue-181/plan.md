# Plan: Depth layers: land/water-distinguishing topo-bathy colormap pivoted at chart datum

## Issue

https://github.com/rolker/camp/issues/181

## Scope decision (overrides Issue Review recommendation)

The Issue Review recommended scoping to an interim constant per-layer pivot,
decoupled from `unh_marine_autonomy#288`. **The operator explicitly chose full
scope instead: automatic per-region chart-datum pivot**, at the run-issue
checkpoint. This plan builds that. Where a piece of full scope cannot be
verified in time for the 2026-08-25 survey, that is called out in Open
Questions rather than silently narrowed.

`unh_marine_autonomy#288` (relocating grid storage to `world/`) is **not** a
blocker — the grids already live at their #288-decided location
(`~/data/world/datum/{geoid,vdatum}/`, confirmed present on this dev host)
regardless of whether #288's updater/materialization work has landed.

## Context (confirmed this session)

- Root cause confirmed by reading `raster_layer.cpp`: `RangeModel::update_auto`
  (called from `imageReady()` and `resetRangeToAuto()`) stretches the palette
  linearly over `[data_min_, data_max_]` with no anchor. `oleron`/`hypsometric`
  are already in the colormap picker (`raster_layer.cpp:535`,
  `marine_colormap::palette_names()`) but nothing in camp ever reads
  `Palette::domain()->shoreline_position`, so the baked shoreline color lands
  wherever the data span happens to put it.
- The anchoring primitive already exists and is already tested upstream:
  `marine_colormap::BreakpointMap(lo, hi, breaks)` (`lookup.hpp`) maps a data
  value onto normalized `[0,1]` through one or more anchored breaks, clamps
  degenerate input, and never throws. `marine_colormap/test/test_lookup.cpp`
  already covers its degenerate cases (break outside domain, at a boundary,
  zero-width domain) — camp's tests do not need to re-prove that math, only
  that camp wires it correctly.
- The shared render path (`RasterGlRenderer`, ADR-0007) is the single seam:
  `ensureLut()` bakes a 256×1 LUT from `marine_colormap::bake_lut(palette,
  TransferParams{}, 256)`; the fragment shader linearly normalizes
  `(v - u_min)/(u_max - u_min)` and samples the LUT. `RasterLayer`,
  `GggsTileLayer`, and `SonarLiveCacheLayer` all share one `RasterGlRenderer`
  instance per layer and this one `ensureLut()`.
- The full-form pivot *source* already exists too:
  `marine_vertical_datum::make_vdatum_query()` (ROS-free, PROJ-backed, built
  once and reused per point) plus `marine_vertical_datum::resolve_datum()`
  (`datum_config.hpp`), which layers VDatum → polygon overrides →
  `lake_datum` param → **`nullopt`** with the exact precedence chart_datum_node
  already uses in production. `resolve_datum()` returning `nullopt` for "no
  in-datum data" is not a gap to work around — it is this library's designed
  fallback signal, and it is what tonight's 3-of-80 ENC export failures hit.
  CAMP's job is to handle that `nullopt` honestly (see Fallback below), not to
  invent a fallback value.
- `camp` already depends on `marine_autonomy` (the `unh_marine_autonomy`
  metapackage), so `core_ws` is already an underlay for `ui_ws` — adding
  `marine_vertical_datum` as a new `<depend>` is not a new layering
  relationship.
- Grids on this dev host: `~/data/world/datum/geoid/us_noaa_g2018u0.tif` +
  `~/data/world/datum/vdatum/MENHMAgome23_8301_*.gtx`. No consumer in the
  workspace currently defaults to this path — `import_geotiff`/`s102_import`
  require explicit `--geoid`/`--vdatum-dir`. CAMP will be the first consumer
  to default to it (Decision 4 below).

## Approach

### 1. Pure pivot-bake helper (no GL, no I/O) — lands first, fully unit-testable

Add `src/camp_map/raster/pivoted_lut.{h,cpp}`:

```cpp
// Bake `palette` into an n-entry RGBA8 LUT whose entry i corresponds to the
// SAME linear position the shader's (v - lo)/(hi - lo) normalize would
// produce for i/(n-1), but re-expressed through `pivot` so the palette's
// shoreline_position lands at the pivot's data value instead of at whatever
// t the raw span put it at. `pivot` absent => plain palette_sample(i/(n-1))
// (today's behavior, unpivoted).
std::vector<marine_colormap::Rgba8> bake_pivoted_lut(
  const marine_colormap::Palette& palette,
  float lo, float hi, std::optional<float> pivot_value, std::size_t n);
```

Implementation: when `pivot_value` and `palette.domain()->shoreline_position`
are both present, build
`marine_colormap::BreakpointMap(lo, hi, {{*pivot_value, *shoreline_position}})`
and, for each `i`, take `data_value = lo + (hi - lo) * i/(n-1)`,
`t = breakpoint_map.normalize(data_value)`, `palette.sample(t)`. This is
exactly the "asymmetric effective lo/hi feeding the existing linear shader
normalize" the Issue Review calls for: the shader's own normalize is
untouched (still plain linear `(v-u_min)/(u_max-u_min)`); only which color
each LUT slot holds changes. `bake_lut`'s `TransferParams` stays out of this
path entirely — this helper calls `Palette::sample()` directly, so
ADR-0008 Consequence #1 (`TransferParams` must stay identity) is structurally
unbreakable here, not just honored by convention.

When `pivot_value` is absent, or the active palette has no
`shoreline_position` (grayscale/viridis/turbo/etc.), fall back to
`marine_colormap::bake_lut(palette, TransferParams{}, n)` — byte-identical
to today's `ensureLut()` output, so every non-topo-bathy palette (and every
topo-bathy palette before a pivot resolves) is provably unaffected.

Gate on `palette.domain()->shoreline_position` (not on a new "is this a depth
layer" flag): pivoting only ever engages for `oleron`/`hypsometric`, on any
of the three consumers. This also means a backscatter `GggsTileLayer` band
switched to `oleron` would get pivoted too — that is consistent, not a bug:
the palette declares the semantic, not the layer.

### 2. Wire the helper into `RasterGlRenderer`

- `RasterGlRenderer::setPivot(std::optional<float> value)` (mirrors
  `setColormap`'s dirty-flag pattern) plus `pivot()` getter.
- `ensureLut()` currently caches on `colormap_name_` alone
  (`lut_dirty_`/`setColormap`). It must now also depend on `u_min`/`u_max`
  (the pivot bake needs `lo`/`hi` to place breakpoints), so `lut_dirty_` also
  flips when the incoming `data_min`/`data_max` passed into `renderToImage()`
  differ from the last bake's. This is a real behavior change from today
  (LUT was previously range-independent) — call it out explicitly in the
  camp ADR's Consequences: LUT rebakes on every Auto-range update and every
  Manual range edit, not just on colormap switch. 256-entry CPU bake is cheap
  (confirm with a quick timing note in the PR, not a perf test) but this is
  the honest place to flag it rather than let it surface as a surprise
  regression in `review-code`.
- `renderToImage()` passes `pivot_` into `ensureLut()`; no shader change.

### 3. Chart-datum pivot resolution (full scope)

New `src/camp_map/raster/chart_datum_service.{h,cpp}`, a lazily-constructed
process-wide singleton (mirrors the vdatum_query.hpp usage contract: "build
the query ONCE ... do NOT rebuild per point"):

- Reads grid paths from `QSettings` (`ChartDatum/geoid_grid`,
  `ChartDatum/vdatum_grid_dir`), defaulting to
  `~/data/world/datum/geoid/us_noaa_g2018u0.tif` and
  `~/data/world/datum/vdatum` — the #288-decided `world/` layout, confirmed
  present on this dev host. No new provisioning step is needed for the
  Tuesday survey; this only needs to exist on the operating machine's disk,
  same as today's ENC corpus.
- Calls `marine_vertical_datum::make_vdatum_query()` once, with a `diag`
  callback wired to `qWarning` (setup-time problems only — missing grids,
  bad pipeline — are logged once, not per point).
- Exposes `std::optional<float> pivotAt(double lat, double lon) const`,
  which calls the query fn, then `marine_vertical_datum::resolve_datum(lat,
  lon, /*lake_datum*/{}, /*lake_datum_mhhw*/{}, vdatum_result, /*entries*/{})`
  and returns `chart_datum_z` as `float`, or `nullopt` if either the query or
  `resolve_datum` comes back empty (no grids, no coverage, or the factory
  failed entirely — same code path either way, see Fallback below).
- **Polygon overrides / `lake_datum` are deferred** (empty `entries`,
  `nullopt` lake_datum passed for now) — see Open Questions. VDatum-only
  coverage is exactly what the issue's motivating case (Isles of Shoals
  harbor/coastal) needs; Massabesic-style lake overrides are a documented,
  separate follow-up, not required for Tuesday's nearshore-rocks survey.
- **Threading**: the returned `VDatumQueryFn` is explicitly *not*
  concurrency-safe across copies sharing one PROJ context (per
  `vdatum_query.hpp`). `chart_datum_service` must only ever be called from
  the Qt GUI thread (where `paint()`/`renderImage()` already run) — never
  from `RasterLayer`'s `QtConcurrent` load thread. State this constraint in
  the header doc and enforce it by construction: `pivotAt()` is called from
  `RasterLayer::renderImage()` / `GggsTileLayer`'s render path only, never
  from `loadAndReprojectFile()`.

### 4. Per-layer wiring: representative point + fallback

`RasterLayer::renderImage()` (and the equivalent in `GggsTileLayer`), before
calling `renderer_.renderToImage()`:

- Compute the lat/lon of the current render's Web-Mercator center
  (`clip_bounds` center, converted via the existing `web_mercator` inverse —
  confirm/add an inverse helper if `web_mercator.h` only has `geoToMap`
  today) once per render.
- `const auto pivot = chart_datum_service().pivotAt(lat, lon);`
- `renderer_.setPivot(pivot);`
- **Fallback — the required design element.** When `pivot` is `nullopt`:
  the layer renders exactly as it does today — plain linear auto/manual
  range, no anchor, whatever palette is selected — via step 1's "absent
  pivot" branch. It does **not** pivot at 0, and it does **not** silently
  claim correctness. The layer's existing `setStatus()` mechanism
  (`map_item.h`, already used for `"(loading...)"` / `"(load failed)"`)
  gets a new status string, e.g. `"(no chart datum for this view)"`, set/
  cleared each render based on whether `pivot` resolved. This reuses an
  existing, already-visible-to-the-operator affordance instead of adding new
  UI, so it is in scope for Tuesday. A wrong pivot is the failure this issue
  exists to remove; an honestly-labeled absence of one is not.
- Only recompute/re-render when the pivot changes meaningfully (avoid
  thrashing `ensureLut()` on sub-metre pan deltas): round the query point to
  a coarse grid (e.g. ~100 m, matching "datum surfaces vary slowly") before
  calling `pivotAt()`, or cache the last resolved pivot and skip the query
  when the viewport center hasn't moved past that threshold. Land the
  simplest form (recompute every render, no threshold) first and add the
  threshold only if the PR's own manual exercise shows visible cost —
  don't pre-optimize a path that's O(1) PROJ calls per repaint burst.

### 5. Record the decision as a camp ADR

`docs/decisions/0015-topo-bathy-pivoted-lut.md` (next free number after
0014). Documents: the BreakpointMap-bake seam (step 1-2), the
`palette.domain()->shoreline_position` gate, the `chart_datum_service`
per-region resolution + its `nullopt` fallback contract, the GUI-thread-only
constraint on the vdatum query, and the LUT-now-depends-on-range consequence.
Cross-references ADR-0008 (Consequence #1 compliance) and ADR-0009 (states
explicitly that this PR does **not** touch the range dialog — see step 6).

### 6. ADR-0009 dialog: explicitly out of scope, follow-up filed

The "Colormap range…" dialog and `ColormapRangeState` are **not** touched by
this PR. The pivot is derived from the layer's geography, not from the
operator-editable range, so there is no immediate conflict — a Manual
range override still works exactly as before (it changes `lo`/`hi`, which
step 2 already threads into the pivot bake as the `BreakpointMap` domain).
What the dialog does **not** yet do is show the operator *where* the pivot
landed (no shoreline handle/marker in `ColormapLegendWidget`). File a
follow-up camp issue for that (surfacing the resolved pivot value and
whether it's live/stale/absent in the range dialog) rather than scope-creep
it into this PR.

## Files to Change

| File | Change |
|------|--------|
| `src/camp_map/raster/pivoted_lut.h` (new) | `bake_pivoted_lut()` declaration |
| `src/camp_map/raster/pivoted_lut.cpp` (new) | BreakpointMap-based bake + unpivoted fallback |
| `src/camp_map/raster/raster_gl_renderer.h` | `setPivot()`/`pivot()`, pivot in dirty-tracking |
| `src/camp_map/raster/raster_gl_renderer.cpp` | `ensureLut()` calls `bake_pivoted_lut()`; dirty on range change |
| `src/camp_map/raster/chart_datum_service.h` (new) | Singleton accessor, `pivotAt(lat, lon)` |
| `src/camp_map/raster/chart_datum_service.cpp` (new) | `make_vdatum_query` + `resolve_datum` wiring, QSettings grid paths |
| `src/camp_map/raster/raster_layer.cpp` | Compute view-center lat/lon, call `pivotAt`, `setPivot`, fallback `setStatus` |
| `src/camp_map/raster/gggs_tile_layer.cpp` | Same wiring as `raster_layer.cpp` |
| `src/camp_map/map_view/web_mercator.h`/`.cpp` | Add inverse (mapToGeo) if not already present — confirm during implementation |
| `package.xml` | `<depend>marine_vertical_datum</depend>` |
| `CMakeLists.txt` | New sources + `ament_target_dependencies` entry; new gtests |
| `test/test_pivoted_lut.cpp` (new) | Pivot placement + degenerate cases (pivot outside range, at boundary, zero-width range, non-topo-bathy palette unaffected) |
| `test/test_chart_datum_service.cpp` (new) | `nullopt` on missing grids; resolved value shape (mockable via a test grid dir, or documents as a manual-exercise gap if PROJ fixtures aren't practical in gtest — decide during implementation) |
| `test/test_raster_gl_renderer.cpp` | Extend: pivoted vs. unpivoted LUT bytes differ correctly; range-change triggers rebake |
| `docs/decisions/0015-topo-bathy-pivoted-lut.md` (new) | ADR per step 5 |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Capture decisions, not just implementations | New camp ADR-0015 (step 5), triggered explicitly by ADR-0008/0009 precedent |
| A change includes its consequences | LUT-now-range-dependent perf note; ADR-0009 dialog explicitly scoped out with a follow-up filed, not silently dropped |
| Test what breaks | Pivot placement + all four degenerate cases from the Issue Review's test action item; fallback path tested |
| Only what's needed | Polygon/`lake_datum` overrides deferred (VDatum-only covers the motivating case); range-dialog pivot surfacing deferred to a follow-up issue |
| Never document from assumptions | Grid paths, API shapes (`resolve_datum`, `VDatumQueryFn`, `BreakpointMap`) all verified by reading source, not inferred |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| camp ADR-0008 (marine_colormap LUT bake) | Yes | Consequence #1 (`bake_lut` TransferParams stays identity) is structurally enforced — the pivot path never calls `bake_lut`, it calls `Palette::sample()` directly through a `BreakpointMap`-remapped index |
| camp ADR-0009 (colormap range dialog) | Yes (decision recorded) | Explicitly NOT touched by this PR (step 6); follow-up issue to be filed for pivot visibility in the dialog |
| camp ADR-0007 (shared raster render path) | Yes | `ensureLut()`/`RasterGlRenderer` is the one shared seam modified; all three consumers (`RasterLayer`, `GggsTileLayer`, `SonarLiveCacheLayer`) inherit the fix uniformly via the palette-domain gate |
| New camp ADR-0015 | This PR | Records the anchoring mechanism, the `nullopt` fallback contract, and the GUI-thread constraint |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `ensureLut()` dirty-tracking (now range-dependent) | Perf note in ADR-0015; watch for repaint-storm cost | Yes — flagged in step 2/4, threshold deferred unless measured necessary |
| `RasterGlRenderer` gains a pivot concept | `SonarLiveCacheLayer` inherits it for free via the shared renderer | Yes — no separate wiring needed there since it goes through the same `ensureLut()`; it simply never resolves a pivot unless someone points a `SonarLiveCacheLayer` at `oleron`/`hypsometric`, which is a legitimate (if unusual) operator choice |
| New `marine_vertical_datum` dependency | `package.xml`, `CMakeLists.txt`, build docs if any list camp's deps | Yes |
| Chart-datum resolution now touches the GUI thread per render | Threading constraint documented in `chart_datum_service.h` and ADR-0015 | Yes |
| `ColormapRangeState`/dialog gains no pivot display | Follow-up camp issue | Yes — filed, not silently dropped |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): None — no existing package README/API doc describes the pre-pivot LUT-bake behavior in prose (ADR-0008 itself documents the identity-TransferParams contract, which stays true; ADR-0015 is the new record, not a correction to an existing one).
- **Agent-instruction candidates**: None — this is a project-repo (camp) implementation detail; no workspace-level pattern or pitfall surfaced that belongs in `.agent/knowledge/` or `AGENTS.md`.

## Open Questions

- **Polygon overrides / `lake_datum`**: deferred to empty (VDatum-only). If
  Tuesday's survey area has any inland/estuarine fringe where VDatum has no
  coverage (plausible near rocks/shoreline), those spots render with the
  fallback "(no chart datum for this view)" status rather than a pivot —
  correct and honest, but the operator should know this is expected, not a
  bug, going into the survey. Confirm acceptable before Tuesday, or scope in
  a minimal polygon-config read as a fast-follow if the survey area is known
  to need it.
- **Grid-path defaults are new** (`chart_datum_service`'s `~/data/world/datum/`
  QSettings defaults are the first hardcoded default to that path anywhere in
  the workspace). Confirm this is the intended precedent versus requiring an
  explicit one-time settings entry — leaning toward defaulting, since a
  missing/wrong path degrades to the same honest "(no chart datum)" fallback
  rather than a hard failure.
- **`test_chart_datum_service.cpp` fixture approach**: whether a real (small)
  VDatum grid fixture is practical to check into the test tree, or whether
  the PROJ-dependent resolution path is validated only by manual exercise
  against the real `~/data/world/datum/` grids (consistent with how ADR-0009
  documents its own modal-dialog testing gap) — decide during implementation;
  either way, the `nullopt`-on-missing-grids path IS unit-testable without
  real grids (point at an empty/nonexistent directory) and must be covered.
- **Rebake-on-every-pan-tick cost**: unmeasured. Plan takes the "land the
  simple form, add a coarse-grid threshold only if the PR's manual exercise
  shows visible cost" position (step 4) rather than guessing at a threshold
  value up front.

## Estimated Scope

Single PR, sequenced internally so risk is separable within it:

1. Step 1 (pure `bake_pivoted_lut`, fully unit-tested, no I/O, no GL) —
   land and get this reviewed/verified first; it is the operator-visible
   correctness core and has zero dependency on grid availability.
2. Steps 2 (renderer wiring) + tests — still GL-only, no PROJ/I/O.
3. Steps 3-4 (`chart_datum_service`, per-layer resolution, fallback status)
   — the I/O-touching, harder-to-verify-in-CI part; riskiest for the
   Tuesday timeline. If this slips, steps 1-2 alone are safe to ship (every
   palette renders exactly as today, since no pivot source ever resolves) —
   NOT a silent narrowing of scope, but a real fallback state the fallback
   design in step 4 already handles correctly by construction.
4. Steps 5-6 (ADR + follow-up issue) — documentation, land alongside 1-4.

If steps 3-4 cannot be verified against real grids by Monday, say so at
`review-code`/merge time rather than merging unverified I/O-touching code
into a Tuesday survey path — this plan does not pre-decide that outcome.
