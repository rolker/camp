// Safe-degradation tests for DepthRaster (#59 PR6 increment 1 — depth extracted
// from BackgroundRaster). Depth feeds the planner's shoal avoidance, so a
// missing/invalid chart must degrade to "no depth" (NaN) rather than crashing or
// returning a garbage value that could read as deep water. Correctness against a
// real depth chart (the load mirrors BackgroundRaster's exactly) is verified in
// sim: cursor depth readout + planner shoal avoidance.

#include <gtest/gtest.h>

#include <cmath>

#include <QGeoCoordinate>

#include "depth_raster.h"

// A file that does not exist must not crash and must report no depth.
TEST(DepthRasterTest, NonexistentFileHasNoDepth)
{
  DepthRaster dr("/nonexistent/path/no_such_chart.tif");
  EXPECT_FALSE(dr.depthValid());
  EXPECT_EQ(dr.width(), 0);
  EXPECT_EQ(dr.height(), 0);
}

// Queries against an invalid raster return NaN (so AutonomousVehicleProject::
// getDepth surfaces "unknown", which the planner treats as unsafe) — never a
// finite garbage depth, and never a crash via the unset georeference.
TEST(DepthRasterTest, InvalidRasterReturnsNanDepth)
{
  DepthRaster dr("/nonexistent/path/no_such_chart.tif");
  EXPECT_TRUE(std::isnan(dr.getDepth(0, 0)));
  EXPECT_TRUE(std::isnan(dr.getDepth(-1, -1)));
  EXPECT_TRUE(std::isnan(dr.getDepth(QGeoCoordinate(43.07, -70.71))));
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
