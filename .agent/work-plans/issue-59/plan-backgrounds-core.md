# Plan: Backgrounds-as-layers core (ADR-0003 stages 3 + 2 + 5)

> **Status (2026-06-07): IMPLEMENTED.** All six steps landed on `feature/issue-59`
> (commits `6349e42` step 3, `73559df` steps 1+4+6, `55ca7f2` step 5). Build + 44
> tests pass; GUI sim-verify gates listed in `progress.md`. BackgroundRaster and
> its single-current-chart model are gone; charts are stacked Map RasterLayers
> with QSettings app-state persistence and a Layers-tab Remove action.

Sub-plan of #59. Implements the heart of [ADR-0003](../../../docs/decisions/0003-backgrounds-as-layers-and-depth-tree.md):
backgrounds fully dissolve into Map-owned layers. Stage 1 (RasterLayer
synchronous extent, `b741d1d`) is done. Stage 4 (depth-layer tree) and stage 6
(depth-only → generate visual) are deliberately OUT of scope — follow-ups.

## Issue

https://github.com/rolker/camp/issues/59 (PR #60, branch `feature/issue-59`)

## Context

After increments 1–3 the `BackgroundRaster` is vestigial: depth is in
`DepthRaster`, every overlay parents to the Map anchor, icon scale is
`metresPerPixel`. It still wrongly lives as a mission-tree node (the chart shows
in the Mission tab + accumulates on swap) and AVP still models a single current
background (`m_currentBackground`/`m_currentRasterLayer`). ADR-0003 makes
backgrounds independent, stackable, Map-owned `RasterLayer`s; missions keep
project files; backgrounds persist as Map/app state. Multiple backgrounds is an
intentional, wanted feature (do NOT collapse to one).

## Approach

Ordered; each step builds. Steps 1–4 are stage 3, step 5 is stage 2, step 6 is
stage 5. **Steps 1 and 6 land together** (one push) so a loaded chart never has
no persistence home (it leaves the mission file in step 1, gains Map-state in 6).

1. **Load a chart as a Map layer, not a mission node.** Replace
   `openBackground`'s `new BackgroundRaster(fname, m_root)` + mission `beginInsertRows`
   with: append a `RasterLayer` to `m_map->topLevelLayers()` (stacking — do not
   replace the previous) and a `DepthRaster` to a depth-provider list. Retire
   `m_currentBackground` / `setCurrentBackground` / `getBackgroundRaster()` and
   the single `m_currentRasterLayer`. `getDepth(geo)` walks the depth-provider
   list (load order for now; stage 4 makes it a tree).
2. **Re-source fit-to-extent.** Have the load emit a signal carrying the new
   `RasterLayer`; `ProjectView` fits to its `sceneBoundingRect()` (valid
   synchronously via stage 1). Preserves zoom-to-chart-on-open. Drop
   `ProjectView::updateBackground`'s `bg->boundingRect()/pixelToGeo()`.
3. **Collapse `geoToPixel` + scale readers.** Remove the `geoToPixel(point, AVP*)`
   and `geoToPixel(point, bg)` overloads; keep bg-free `geoToPixel(point)`; update
   the ~40 `geoToPixel(., avp/bg)` call sites (mechanical). Redirect the two
   `bgr->mapScale()` glyph readers (`geographicsmissionitem` drawArrow,
   `waypoint` shape) to `AutonomousVehicleProject::mapScale()`. Delete the dead
   `findParentBackgroundRaster()` + the `BackgroundRasterType` enum entry.
4. **Delete `BackgroundRaster`.** Remove `backgroundraster.{h,cpp}`,
   `backgrounddetails.{h,cpp}`, the `detailsview` bg branch, the mission
   read/write `"BackgroundRaster"` dispatch (no back-compat per ADR-0003), and
   CMake refs. **Keep `georeferenced.{h,cpp}`** (DepthRaster/VectorDataset use it).
5. **Layers-tab Remove action (stage 2).** Add "Remove" to `Layer::contextMenu`
   (base, so all layers): `map->setMapItemParent(layer, nullptr)` + `delete`
   (mirrors the safe detach already in the chart-swap fix). For a chart layer,
   also drop its `DepthRaster` from the provider list. Charts are now removable
   from where they live.
6. **Map-state persistence (stage 5).** camp2 persists per-layer *settings* by
   `itemID()` but not the *list* of loaded chart rasters. Add: persist the loaded
   chart-layer list (filename, visible, order, opacity; colormap already persists)
   to QSettings on change/close; on startup recreate the `RasterLayer`s +
   `DepthRaster`s from it. The mission file no longer writes backgrounds.

## Files to Change

| File | Change |
|------|--------|
| `src/camp/autonomousvehicleproject.{h,cpp}` | retire current-background ownership; load → Map layer + depth-provider list; getDepth over list; drop getBackgroundRaster/setCurrentBackground |
| `src/camp/projectview.{h,cpp}` | fit-to-extent from the new RasterLayer; drop bg georef use |
| `src/camp/geographicsitem.{h,cpp}` | remove geoToPixel(AVP*)/(bg) overloads + findParentBackgroundRaster + BackgroundRasterType |
| `src/camp/geographicsmissionitem.cpp`, `waypoint.cpp` | mapScale() → AVP::mapScale() |
| `src/camp/missionitem.cpp`, `detailsview.cpp`, `mainwindow.cpp` | drop BackgroundRaster read/write dispatch + details branch |
| ~40 call sites | `geoToPixel(., avp/bg)` → `geoToPixel(.)` (mechanical) |
| `src/camp2/map/layer.{h,cpp}` | base `contextMenu` Remove action |
| `src/camp2/map/map.{h,cpp}` | persist/restore the loaded chart-layer list |
| `src/camp/CMakeLists.txt` | drop backgroundraster/backgrounddetails |
| delete `backgroundraster.{h,cpp}`, `backgrounddetails.{h,cpp}` | retired |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Robustness (robot safety) | Depth + collision/overlays already decoupled (incr 1–2); this is display/ownership only. getDepth provider-list keeps depth available; verify shoal A* + cursor depth unchanged. |
| Do it completely, with tests | geoToPixel collapse is mechanical (build-verified); add a persistence round-trip test (save list → restore) where practicable; GUI parts sim-verified. |
| No workaround when the real fix exists | ADR-0003 explicitly drops old-format compat — no BackgroundRaster serialization shim. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0003 | Yes | This plan IS stages 3/2/5 of it. |
| ADR-0002 | Yes | Completes the background-as-layer migration it began (§1–§3). |
| ADR-0013 | Yes | progress.md entries use the typed vocabulary. |

## Consequences

| If we change... | Also update... | In plan? |
|---|---|---|
| Mission save/load (backgrounds removed) | verify existing `.json` projects still load (old BackgroundRaster node ignored, no crash) | Yes (step 4 + test) |
| Scene/overlay geoToPixel | the ~40 call sites | Yes (step 3) |
| Chart removal UX | Layers-tab Remove action | Yes (step 5) |
| Hypack/mission export | reads mission model — unaffected (ADR-0002) | N/A |

## Open Questions

- [ ] Persistence store: QSettings (matches camp2's existing per-layer mechanism) vs a config file? Lean QSettings.
- [ ] Depth-provider ordering across stacked charts before stage 4's tree exists: follow load order / visible chart layers? Lean load order, superseded by the depth tree.
- [ ] fit-to-extent on each open: fit to the newly-added layer (zoom-on-open) — confirm we re-fit on every chart open, not just the first. Lean: every open (matches today).

## Estimated Scope

Multiple commits on `feature/issue-59` / PR #60. Step 3 (collapse) + step 4
(delete) are the largest; steps 1+6 land together; step 2 small; step 5 medium.
Sim-verify chart load/stack/remove + existing-project load before merge.
