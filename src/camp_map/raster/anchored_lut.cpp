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

#include "anchored_lut.h"

#include <cmath>

#include <marine_colormap/colormap.hpp>
#include <marine_colormap/lookup.hpp>
#include <marine_colormap/transfer.hpp>

namespace camp
{
namespace raster
{

bool palette_supports_anchor(const marine_colormap::Palette & palette)
{
  const std::optional<marine_colormap::PaletteDomain>& domain = palette.domain();
  return domain.has_value() && domain->shoreline_position.has_value();
}

std::vector<marine_colormap::Rgba8> bake_anchored_lut(
  const marine_colormap::Palette & palette, float lo, float hi,
  std::optional<float> anchor_value, std::size_t n)
{
  // Unanchored path: defer to the library so the bytes are identical to what
  // this renderer produced before camp#181 existed. Anything else here would be
  // a silent behaviour change for every general-purpose ramp.
  if(!anchor_value || !std::isfinite(*anchor_value) || !palette_supports_anchor(palette))
    return marine_colormap::bake_lut(palette, marine_colormap::TransferParams{}, n);

  if(n < 1)
    n = 1;

  // One break: the palette's declared land/sea transition, pinned to the
  // operator's water level. BreakpointMap clamps a break outside [lo, hi] onto
  // the nearer end and swaps an inverted range, so no guard is needed here --
  // an all-water view (break above hi) is an ordinary case, not an error.
  const float shoreline = *palette.domain()->shoreline_position;
  const marine_colormap::BreakpointMap map(
    lo, hi, {marine_colormap::Breakpoint{*anchor_value, shoreline}});

  std::vector<marine_colormap::Rgba8> lut;
  lut.reserve(n);
  for(std::size_t i = 0; i < n; ++i)
  {
    // Entry i is the data value the shader's LINEAR normalize maps to i/(n-1);
    // the breakpoint map then re-places it in palette space. Composing the two
    // is what lets the shader stay untouched.
    // Sweep the map's OWN domain, not the raw lo/hi: BreakpointMap swaps an
    // inverted range and resets a non-finite one, and the sweep has to agree
    // with the map it is being fed into or the two disagree at the ends.
    const float f = (n > 1) ? static_cast<float>(i) / static_cast<float>(n - 1) : 0.0f;
    const float value = map.lo() + (map.hi() - map.lo()) * f;
    lut.push_back(marine_colormap::to_rgba8(palette.sample(map.normalize(value))));
  }
  return lut;
}

}  // namespace raster
}  // namespace camp
