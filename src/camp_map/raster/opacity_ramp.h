#ifndef RASTER_OPACITY_RAMP_H
#define RASTER_OPACITY_RAMP_H

#include <cmath>
#include <limits>

namespace camp
{
namespace raster
{

/// [camp#239 / ADR-0017] Uncertainty-driven opacity for a depth colormap: a cell's
/// opacity is 1 where its uncertainty is at or below an "opaque" threshold, 0 where
/// it is at or above a "transparent" threshold, and linear between. This header is
/// the single CPU reference for that mapping; the fragment shader in
/// raster_gl_renderer.cpp mirrors it line for line, and test_opacity_ramp pins it.
/// It is ROS-, GL- and Qt-free.

/// IHO S-44 vertical-uncertainty parameters: the allowed total vertical uncertainty
/// (TVU, a 95 % figure) at depth d is sqrt(a^2 + (b*d)^2), with `a` in metres and `b`
/// dimensionless.
struct Iho44Order
{
  float a;
  float b;
};

/// Shipped default standard: IHO S-44 Order 1a (a = 0.5 m, b = 0.013).
constexpr Iho44Order kDefaultStandard{0.5f, 0.013f};

/// Fully transparent at this multiple of the standard's allowed uncertainty.
constexpr float kDefaultTransparentMultiple = 3.0f;

struct OpacityRamp
{
  enum class Kind
  {
    Standard,   ///< thresholds follow a survey standard, per cell depth
    Manual      ///< operator-pinned absolute uncertainty thresholds [lo, hi]
  };

  Kind kind = Kind::Standard;

  // Standard: opaque at allowed(depth), transparent at multiple * allowed(depth).
  float a = kDefaultStandard.a;
  float b = kDefaultStandard.b;
  float multiple = kDefaultTransparentMultiple;

  // Manual: opaque at/below lo, transparent at/above hi (metres, same unit as the
  // uncertainty band).
  // Seeded with the Order 1a figures at zero depth (a, multiple * a) so that switching to
  // Manual starts from a sensible, valid (hi > lo) ramp rather than a blank [0, 0].
  float lo = kDefaultStandard.a;
  float hi = kDefaultTransparentMultiple * kDefaultStandard.a;
};

/// Make a Manual ramp valid (hi strictly greater than lo), the one rule both the dialog
/// and the persisted-settings reader enforce so that what is applied is what is restored.
/// `lo + 0.01` is not enough on its own: for a large `lo` the float spacing exceeds 0.01
/// and the sum rounds back to `lo`, so the next representable float is used as a floor.
inline void normalizeManualRamp(OpacityRamp& ramp)
{
  if (ramp.kind != OpacityRamp::Kind::Manual || ramp.hi > ramp.lo) {
    return;
  }
  const float bumped = ramp.lo + 0.01f;
  const float next = std::nextafter(ramp.lo, std::numeric_limits<float>::infinity());
  ramp.hi = bumped > ramp.lo ? bumped : next;
}

/// The standard's allowed uncertainty at @p depth (sign-agnostic: the live depth
/// band may be negative-down).
inline float allowedUncertainty(const OpacityRamp& ramp, float depth)
{
  const float bd = ramp.b * depth;
  return std::sqrt(ramp.a * ramp.a + bd * bd);
}

/// Opacity in [0,1] for a cell with uncertainty @p u at depth @p depth.
///  - non-finite @p u -> 0 (nothing is known about the cell's quality, so it is not
///    asserted; ADR-0017 missing-uncertainty policy);
///  - non-finite @p depth -> 0 (the cell is not drawn at all);
///  - u <= opaque threshold -> 1; u >= transparent threshold -> 0; linear between;
///  - a degenerate ramp (transparent <= opaque) is a hard step at the opaque threshold.
inline float opacityFromUncertainty(float u, float depth, const OpacityRamp& ramp)
{
  if(!std::isfinite(u) || !std::isfinite(depth))
    return 0.0f;
  float opaque_at = ramp.lo;
  float transparent_at = ramp.hi;
  if(ramp.kind == OpacityRamp::Kind::Standard)
  {
    opaque_at = allowedUncertainty(ramp, depth);
    transparent_at = ramp.multiple * opaque_at;
  }
  if(u <= opaque_at)
    return 1.0f;
  if(transparent_at <= opaque_at || u >= transparent_at)
    return 0.0f;
  return (transparent_at - u) / (transparent_at - opaque_at);
}

}  // namespace raster
}  // namespace camp

#endif
