// Copyright 2026 Roland Arsenault
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CAMP_MAP__RASTER__COLORMAP_RANGE_DIALOG_H_
#define CAMP_MAP__RASTER__COLORMAP_RANGE_DIALOG_H_

#include <functional>
#include <optional>
#include <string>

#include <QString>

#include "marine_colormap/transfer.hpp"
#include "shoreline_anchor.h"

class QWidget;

namespace camp
{
namespace raster
{

/// A layer's current colormap-range state, passed into showColormapRangeDialog()
/// to seed the interactive colorbar (camp#142). The three scalar raster layers
/// (GggsTileLayer, RasterLayer, SonarLiveCacheLayer) each own a
/// marine_colormap::RangeModel; this is the snapshot the dialog needs to open
/// reflecting that model.
struct ColormapRangeState
{
  int palette_index = 0;        ///< marine_colormap registry index for the painted ramp
  float data_min = 0.0f;        ///< data extent low (colorbar domain); data_min > data_max == no data
  float data_max = 1.0f;        ///< data extent high
  marine_colormap::RangeMode mode = marine_colormap::RangeMode::Auto;
  float lo = 0.0f;              ///< current resolved low bound (the active window)
  float hi = 1.0f;              ///< current resolved high bound

  // [camp#181 / ADR-0015] Shoreline anchor. Populated only by the scalar layers
  // whose palette can carry a shoreline (oleron / hypsometric). When
  // `supports_anchor` is false the dialog shows no anchor control at all, so a
  // sonar/general-purpose ramp is unaffected.
  std::string palette_name;     ///< the ramp name — used to bake the anchored colorbar
  bool supports_anchor = false; ///< palette declares a shoreline_position
  ShorelineAnchor::Source anchor_mode = ShorelineAnchor::Source::None;
  std::optional<double> manual_anchor;   ///< current manual anchor value, if set
};

/// Open a modal dialog hosting marine_colormap_widgets::ColormapLegendWidget plus
/// min/max spin boxes for one layer's colormap range (camp#142 PR2, the
/// interactive successor to PR1's numeric prompts).
///
/// Dragging a handle (or editing a spin box) pins a Manual override and fires
/// `on_range(lo, hi)`; the "Reset to auto" button returns to the data-driven
/// Auto range and fires `on_reset()`. Both fire **live** while the dialog is open
/// so the map re-renders as the operator adjusts. The colorbar and spin boxes
/// stay in sync with each other.
/// [camp#181 / ADR-0015] When the layer's palette carries a shoreline
/// (`state.supports_anchor`), the dialog also shows a "Shoreline anchor" control:
/// Chart datum and Platform tide are listed in D3 order but shown **disabled and
/// honestly labelled unavailable** (no source resolves them until PR2 / PR3);
/// Manual (with a value) and None are operator-selectable. Selecting Manual repaints
/// the colorbar through `bake_anchored_lut()` so the colour↔value mapping is exact
/// under the anchor, and adds a readout naming the active anchor and its source.
/// `on_anchor(mode, manual_value)` fires live as the operator changes it.
void showColormapRangeDialog(
  QWidget * parent, const QString & title, const ColormapRangeState & state,
  std::function<void(float, float)> on_range, std::function<void()> on_reset,
  std::function<void(ShorelineAnchor::Source, std::optional<double>)> on_anchor = {});

}  // namespace raster
}  // namespace camp

#endif  // CAMP_MAP__RASTER__COLORMAP_RANGE_DIALOG_H_
