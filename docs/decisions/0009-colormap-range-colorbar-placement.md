# ADR-0009: Place the interactive colorbar in a context-menu dialog

## Status

Accepted

Implements CAMP issue [#142](https://github.com/rolker/camp/issues/142) — give the
operator an interactive way to override a scalar layer's colormap range when an
outlier skews the auto-range (the motivating case: backscatter band 2 with
Max ≈ 925 / Mean ≈ 0.16 collapses the auto colorbar). Builds on the per-layer
`marine_colormap::RangeModel` and numeric "Set range…" prompts landed in PR1
(also #142), and on [ADR-0008](0008-adopt-marine-colormap-lut-bake.md) (the shared
`marine_colormap` palettes) and [ADR-0007](0007-raster-field-source-interface.md)
(the unified raster render path the three scalar layers share).

> **Numbering.** This is **camp ADR-0009**, a *project* ADR in this repo's
> `docs/decisions/` series — independent of the workspace agent-framework ADR
> series, where the numbers collide by coincidence (see ADR-0007/0008).

## Context

PR1 shipped the range-override backend (`RangeModel` Auto/Manual on
`GggsTileLayer`, `RasterLayer`, `SonarLiveCacheLayer`; `setRangeOverride` /
`resetRangeToAuto`; per-layer persistence) with a minimal UI: two sequential
`QInputDialog::getDouble` prompts. That works but is blind — the operator types
numbers without seeing the palette, the data extents, or where the handles sit
relative to the outlier.

The shared `marine_colormap_widgets::ColormapLegendWidget` (marine_colormap
ADR-0002) paints the value→colour ramp and lets the operator **drag** min/max
handles to pin a `Manual` range. PR2 embeds it. A draggable widget cannot live
*inside* a transient `QMenu` (the menu closes on click), so "context menu" means a
small window opened *from* the menu.

Three placements were considered:

1. **Modal dialog opened from the layer context menu** — one entry, "Colormap
   range…", per scalar layer. No permanent screen real estate; discoverable from
   the same place as the colormap/band actions.
2. **Docked panel in the Layers tab** showing the selected layer's colorbar —
   always visible, better for continuous live tweaking, but needs panel layout
   work and a selection-follows-colorbar wiring.
3. **Map overlay** near the active layer (scale-bar style) — closest to a chart
   legend, but the most placement/occlusion code.

## Decision

Use **option 1**: a modal `QDialog` opened from each scalar layer's context menu
("Colormap range…"), hosting the `ColormapLegendWidget` plus min/max
`QDoubleSpinBox`es. Implemented once in `raster/colormap_range_dialog.{h,cpp}`
(`camp::raster::showColormapRangeDialog`) and called from all three layers, so the
wiring is not duplicated. This replaces PR1's two numeric prompts; the spin boxes
preserve precise numeric entry, now kept in sync with the bar.

Wiring contract:

- On open, the dialog seeds the widget from a `ColormapRangeState` snapshot
  (palette index, data extents as the domain, current mode/lo/hi). An existing
  `Manual` override is shown via `ColormapLegendWidget::setManual` (the slot added
  in marine_colormap#10 precisely so a consumer can display a persisted override);
  otherwise the bar tracks Auto.
- The widget's `rangeChanged` fires for both a manual pin and an auto fold; the
  dialog disambiguates on `mode()` — `Manual` → `on_range(lo, hi)` (the layer's
  `setRangeOverride`), otherwise `on_reset()` (the layer's `resetRangeToAuto`).
- Edits fire **live** while the dialog is open, so the map behind it re-renders as
  the operator drags or types.

## Consequences

- The colorbar is a per-layer, on-demand tool — no always-on UI, no layout churn.
- The shared dialog keeps the three layers' menus identical and DRY; a future
  fourth scalar layer reuses it.
- Modal `exec()` blocks the rest of the UI while open (the map still repaints
  underneath). Acceptable for a brief range adjustment; a future modeless or docked
  variant (option 2) can reuse the same `ColormapRangeState` seam if continuous
  multi-layer tweaking becomes a need.
- The dialog is thin Qt glue over already-tested components — the widget's
  `setManual`/drag/reset/`rangeChanged` (marine_colormap gtest) and the layers'
  `setRangeOverride`/`resetRangeToAuto`/persistence (PR1 `test_range_persist`). The
  modal event loop makes the glue itself impractical to unit-test headlessly, so it
  is validated by manual exercise in the deployed app (consistent with how the #108
  band picker was verified).
- The uncertainty-band default (`quality` palette + `[0, threshold]` on open) is a
  separate follow-up (needs #104 band semantics), not part of this dialog.
