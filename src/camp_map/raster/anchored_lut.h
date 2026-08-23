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

#ifndef CAMP_MAP__RASTER__ANCHORED_LUT_H_
#define CAMP_MAP__RASTER__ANCHORED_LUT_H_

#include <cstddef>
#include <optional>
#include <vector>

#include <marine_colormap/color.hpp>
#include <marine_colormap/palette.hpp>

namespace camp
{
namespace raster
{

/// Bake a 256-ish entry RGBA LUT that pins the palette's shoreline break to an
/// absolute data value (camp#181, camp ADR-0015).
///
/// **Why this exists.** The renderer's shader normalizes linearly
/// (`t = (v - lo) / (hi - lo)`) and samples the LUT at `t`, so with a plain
/// `bake_lut()` the palette's land/sea colour transition lands wherever the
/// *range midpoint* happens to fall. For a topo-bathy ramp that transition is
/// the shoreline, and the operator needs it at a real water level — chart datum
/// or a tide height — not at an artefact of the current range. Anchoring is done
/// in the **bake**, by pre-warping the LUT through a
/// `marine_colormap::BreakpointMap`, so the shader is untouched: entry `i` still
/// corresponds to linear position `i / (n - 1)`, exactly what the shader
/// computes. This is what makes the anchor free at render time.
///
/// **Contract.** When @p anchor_value is set **and** @p palette carries a
/// `domain()->shoreline_position`, the returned table maps @p anchor_value onto
/// that shoreline position. Otherwise the result is **byte-identical** to
/// `marine_colormap::bake_lut(palette, TransferParams{}, n)` — the gate is the
/// palette's own declared shoreline, so a general-purpose ramp (grayscale,
/// viridis) is never silently warped.
///
/// Sampling `Palette::sample()` directly, rather than routing through
/// `bake_lut()` with non-identity params, keeps camp ADR-0008 Consequence #1
/// (identity `TransferParams` on this path) structurally unbreakable: there is
/// no `TransferParams` here to get wrong, so gain/contrast can never be applied
/// twice.
///
/// **Degenerate inputs never throw.** An anchor outside `[lo, hi]`, an inverted
/// or zero-width range, and non-finite values are all handled by
/// `BreakpointMap`, which clamps and stays monotonic. An anchor outside the
/// range is the ordinary case, not an error: a survey line with no land in view
/// has its datum break above `hi`.
///
/// @param palette      the palette to sample.
/// @param lo,hi        the data range the shader will normalize over — the same
///                     `u_min`/`u_max` passed to the draw call. The LUT is a
///                     function of these, so callers must re-bake when they
///                     change (see `RasterGlRenderer::ensureLut()`).
/// @param anchor_value absolute data value to pin the shoreline to, if any.
/// @param n            entry count; `n < 1` is treated as 1, matching `bake_lut`.
std::vector<marine_colormap::Rgba8> bake_anchored_lut(
  const marine_colormap::Palette & palette, float lo, float hi,
  std::optional<float> anchor_value, std::size_t n);

/// True when @p palette declares a shoreline position, i.e. when anchoring is
/// meaningful for it. The UI uses this to offer the anchor control only where it
/// does something (camp#181 D2).
bool palette_supports_anchor(const marine_colormap::Palette & palette);

}  // namespace raster
}  // namespace camp

#endif  // CAMP_MAP__RASTER__ANCHORED_LUT_H_
