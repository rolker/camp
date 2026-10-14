// [camp#239 / ADR-0017] The CPU reference for uncertainty-driven opacity: pins the
// mapping the fragment shader mirrors. Pure maths, no Qt/GL/ROS.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "raster/opacity_ramp.h"

using camp::raster::OpacityRamp;
using camp::raster::allowedUncertainty;
using camp::raster::kDefaultStandard;
using camp::raster::kDefaultTransparentMultiple;
using camp::raster::opacityFromUncertainty;

namespace
{
const float kNaN = std::numeric_limits<float>::quiet_NaN();
}

TEST(OpacityRampTest, DefaultsAreIhoS44Order1aWithThreeTimesMultiple)
{
  EXPECT_FLOAT_EQ(kDefaultStandard.a, 0.5f);
  EXPECT_FLOAT_EQ(kDefaultStandard.b, 0.013f);
  EXPECT_FLOAT_EQ(kDefaultTransparentMultiple, 3.0f);
  const OpacityRamp r;
  EXPECT_EQ(r.kind, OpacityRamp::Kind::Standard);
}

TEST(OpacityRampTest, AllowedUncertaintyFollowsSqrtAPlusBD)
{
  const OpacityRamp r;
  EXPECT_NEAR(allowedUncertainty(r, 0.0f), 0.5f, 1e-6);
  // d = 100 m: sqrt(0.25 + 1.3^2) = sqrt(1.94)
  EXPECT_NEAR(allowedUncertainty(r, 100.0f), std::sqrt(1.94f), 1e-5);
  // sign-agnostic (negative-down depth)
  EXPECT_FLOAT_EQ(allowedUncertainty(r, -100.0f), allowedUncertainty(r, 100.0f));
}

TEST(OpacityRampTest, StandardEndpointsAndLinearMidpoint)
{
  const OpacityRamp r;
  const float depth = 0.0f;                       // allowed = 0.5 m
  EXPECT_FLOAT_EQ(opacityFromUncertainty(0.0f, depth, r), 1.0f);
  EXPECT_FLOAT_EQ(opacityFromUncertainty(0.5f, depth, r), 1.0f);   // at allowed: opaque
  EXPECT_FLOAT_EQ(opacityFromUncertainty(1.5f, depth, r), 0.0f);   // 3x allowed
  EXPECT_FLOAT_EQ(opacityFromUncertainty(9.0f, depth, r), 0.0f);
  EXPECT_NEAR(opacityFromUncertainty(1.0f, depth, r), 0.5f, 1e-6);  // midpoint
}

TEST(OpacityRampTest, StandardThresholdsScaleWithDepth)
{
  const OpacityRamp r;
  // The same 1.0 m uncertainty is acceptable-ish in shallow water and fine in deep.
  const float shallow = opacityFromUncertainty(1.0f, 0.0f, r);
  const float deep = opacityFromUncertainty(1.0f, 100.0f, r);
  EXPECT_LT(shallow, deep);
  EXPECT_FLOAT_EQ(deep, 1.0f);   // 1.0 < sqrt(1.94) = 1.39 allowed at 100 m
}

TEST(OpacityRampTest, ManualRampOverridesTheStandard)
{
  OpacityRamp r;
  r.kind = OpacityRamp::Kind::Manual;
  r.lo = 1.0f;
  r.hi = 3.0f;
  EXPECT_FLOAT_EQ(opacityFromUncertainty(1.0f, 500.0f, r), 1.0f);
  EXPECT_FLOAT_EQ(opacityFromUncertainty(3.0f, 500.0f, r), 0.0f);
  EXPECT_NEAR(opacityFromUncertainty(2.0f, 500.0f, r), 0.5f, 1e-6);
  EXPECT_NEAR(opacityFromUncertainty(2.5f, 0.0f, r), 0.25f, 1e-6);   // depth-independent
}

TEST(OpacityRampTest, DegenerateManualRampIsAHardStep)
{
  OpacityRamp r;
  r.kind = OpacityRamp::Kind::Manual;
  r.lo = 2.0f;
  r.hi = 2.0f;
  EXPECT_FLOAT_EQ(opacityFromUncertainty(1.9f, 10.0f, r), 1.0f);
  EXPECT_FLOAT_EQ(opacityFromUncertainty(2.1f, 10.0f, r), 0.0f);
}

// hi must end up strictly greater than lo for ANY lo, including values where adding
// 0.01 rounds back to lo in float (the persisted-settings reader rejects hi <= lo).
TEST(OpacityRampTest, NormalizeManualRampAlwaysYieldsHiGreaterThanLo)
{
  for(float lo : {0.0f, 0.5f, 2.0f, 1000.0f, 1.0e5f, 1.0e6f, 1.0e7f})
  {
    OpacityRamp r;
    r.kind = OpacityRamp::Kind::Manual;
    r.lo = lo;
    r.hi = lo;
    camp::raster::normalizeManualRamp(r);
    EXPECT_GT(r.hi, r.lo) << "lo=" << lo;
  }
  OpacityRamp valid;
  valid.kind = OpacityRamp::Kind::Manual;
  valid.lo = 1.0f;
  valid.hi = 3.0f;
  camp::raster::normalizeManualRamp(valid);
  EXPECT_FLOAT_EQ(valid.hi, 3.0f) << "a valid ramp is left alone";
  OpacityRamp standard;
  standard.lo = standard.hi = 5.0f;
  camp::raster::normalizeManualRamp(standard);
  EXPECT_FLOAT_EQ(standard.hi, 5.0f) << "only Manual ramps are normalised";
}

TEST(OpacityRampTest, NonFiniteInputsAreTransparent)
{
  const OpacityRamp r;
  EXPECT_FLOAT_EQ(opacityFromUncertainty(kNaN, 10.0f, r), 0.0f);
  EXPECT_FLOAT_EQ(opacityFromUncertainty(0.1f, kNaN, r), 0.0f);
  EXPECT_FLOAT_EQ(opacityFromUncertainty(std::numeric_limits<float>::infinity(), 10.0f, r),
                  0.0f);
}
