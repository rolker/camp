// Unit tests for camp::map::ColorMap (camp#63).
#include <gtest/gtest.h>
#include <cmath>
#include "map/color_map.h"

using camp::map::ColorMap;

TEST(ColorMap, GrayscaleEndpointsAndMidpoint)
{
  ColorMap cm(ColorMap::Grayscale);
  EXPECT_EQ(cm.colorNormalized(0.0), QColor(0, 0, 0));
  EXPECT_EQ(cm.colorNormalized(1.0), QColor(255, 255, 255));
  // Midpoint is a mid grey on the diagonal.
  const QColor mid = cm.colorNormalized(0.5);
  EXPECT_EQ(mid.red(), mid.green());
  EXPECT_EQ(mid.green(), mid.blue());
  EXPECT_NEAR(mid.red(), 128, 1);
}

TEST(ColorMap, RangeMapsToNormalized)
{
  ColorMap cm(ColorMap::Grayscale);
  // value at min -> 0, at max -> 1, halfway -> 0.5.
  EXPECT_EQ(cm.color(10.0, 10.0, 20.0), cm.colorNormalized(0.0));
  EXPECT_EQ(cm.color(20.0, 10.0, 20.0), cm.colorNormalized(1.0));
  EXPECT_EQ(cm.color(15.0, 10.0, 20.0), cm.colorNormalized(0.5));
}

TEST(ColorMap, ClampsOutOfRange)
{
  ColorMap cm(ColorMap::Turbo);
  // Below min clamps to the t=0 colour; above max clamps to the t=1 colour.
  EXPECT_EQ(cm.color(-5.0, 0.0, 10.0), cm.colorNormalized(0.0));
  EXPECT_EQ(cm.color(99.0, 0.0, 10.0), cm.colorNormalized(1.0));
}

TEST(ColorMap, NanAndDegenerateRangeAreTransparent)
{
  ColorMap cm(ColorMap::Viridis);
  EXPECT_EQ(cm.colorNormalized(std::nan("")).alpha(), 0);
  EXPECT_EQ(cm.color(std::nan(""), 0.0, 1.0).alpha(), 0);
  // max <= min is degenerate -> transparent (no division-by-zero colour).
  EXPECT_EQ(cm.color(5.0, 10.0, 10.0).alpha(), 0);
  EXPECT_EQ(cm.color(5.0, 10.0, 1.0).alpha(), 0);
}

TEST(ColorMap, OpaqueColoursInRange)
{
  for(auto type : ColorMap::allTypes())
  {
    ColorMap cm(type);
    EXPECT_EQ(cm.colorNormalized(0.0).alpha(), 255);
    EXPECT_EQ(cm.colorNormalized(0.5).alpha(), 255);
    EXPECT_EQ(cm.colorNormalized(1.0).alpha(), 255);
  }
}

TEST(ColorMap, NamedRampsDifferFromGrayscale)
{
  // A perceptual ramp's midpoint is not a neutral grey.
  const QColor v = ColorMap(ColorMap::Viridis).colorNormalized(0.5);
  EXPECT_FALSE(v.red() == v.green() && v.green() == v.blue());
}

TEST(ColorMap, NameRoundTrip)
{
  for(auto type : ColorMap::allTypes())
  {
    bool ok = false;
    EXPECT_EQ(ColorMap::typeFromName(ColorMap::name(type), &ok), type);
    EXPECT_TRUE(ok);
  }
  bool ok = true;
  EXPECT_EQ(ColorMap::typeFromName("nope", &ok), ColorMap::Grayscale);
  EXPECT_FALSE(ok);
}
