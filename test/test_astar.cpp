// Shoal-avoidance tests for the A* planner (#59 PR6 — depth/A* decoupled from
// BackgroundRaster onto a self-defined Web-Mercator grid). These pin the
// safety-critical contract the planner provides on an autonomous boat:
//
//   * water shallower than minDepth is an obstacle the path must route around;
//   * a barrier of unsafe water with no gap yields NO path (fail safe — the
//     planner must not invent a route through a shoal);
//   * UNKNOWN depth (no coverage) is treated as an obstacle, never as deep
//     water. This realises the AutonomousVehicleProject::getDepth(geo) contract
//     downstream: getDepth returns NaN where there is no depth provider, the
//     grid stores those cells as Context::unknownDepth, and A* must treat them
//     as impassable.
//
// AutonomousVehicleProject::getDepth(geo)/hasDepth() provider wiring (a single
// DepthRaster today) is a QAbstractItemModel path that can't be cheaply unit-
// instantiated; it is verified in sim (cursor depth readout + live shoal
// avoidance over a real depth chart). What is unit-coverable is the value
// contract those functions feed into — that lives here and in test_depth_raster.

#include <gtest/gtest.h>

#include <vector>

#include "astar.h"

using astar::AStar;
using astar::Context;
using astar::Position;

namespace
{
constexpr int kGrid = 20;
constexpr float kDeep = 100.0f;     // safely navigable
constexpr float kShallow = 1.0f;    // below minDepth -> obstacle
constexpr float kMinDepth = 10.0f;

int idx(int x, int y) { return y * kGrid + x; }

// All-deep grid; obstacles are painted in by the individual tests.
Context makeOpenWaterContext()
{
  Context c;
  c.gridSize = kGrid;
  c.depthGrid.assign(kGrid * kGrid, kDeep);
  c.start = Position(0, 0);
  c.finish = Position(kGrid - 1, kGrid - 1);
  c.minDepth = kMinDepth;
  c.maxDepth = kDeep;     // depth >= maxDepth -> zero depth-cost, so cost == distance
  c.shipDraft = 1.0;
  return c;
}

// True iff every step of the path sits on navigable water (strictly deeper than
// minDepth) — i.e. the planner never routed across an obstacle cell.
bool pathStaysInSafeWater(Context const& c, std::vector<Position> const& path)
{
  for (auto const& p : path)
    if (c.depthAt(p.x, p.y) <= c.minDepth)
      return false;
  return true;
}
}  // namespace

// Sanity: in fully open water the planner returns a path from start to finish.
TEST(AStarShoalTest, OpenWaterReachesGoal)
{
  Context c = makeOpenWaterContext();
  AStar astar(1);  // 8-connected Moore neighbourhood — single-cell steps

  std::vector<Position> path = astar.search(c);

  ASSERT_FALSE(path.empty());
  EXPECT_TRUE(path.front() == c.start);
  EXPECT_TRUE(path.back() == c.finish);
  EXPECT_TRUE(pathStaysInSafeWater(c, path));
}

// A shoal wall spanning most of the grid with a gap near the bottom: the path
// must reach the goal AND never step on a shallow cell — it has to detour
// through the gap rather than cut straight across.
TEST(AStarShoalTest, PathDetoursAroundShoal)
{
  Context c = makeOpenWaterContext();

  // Vertical shallow wall at x=10 for y=0..14; gap at y=15..19.
  const int wallX = 10;
  for (int y = 0; y <= 14; ++y)
    c.depthGrid[idx(wallX, y)] = kShallow;

  AStar astar(1);
  std::vector<Position> path = astar.search(c);

  ASSERT_FALSE(path.empty()) << "a gap exists, so a route must be found";
  EXPECT_TRUE(path.back() == c.finish);
  EXPECT_TRUE(pathStaysInSafeWater(c, path))
      << "planner routed across the shoal instead of through the gap";
}

// A shoal wall spanning the FULL height with no gap separates start from goal.
// The planner must fail safe (empty path) rather than fabricate a crossing.
TEST(AStarShoalTest, FullBarrierYieldsNoPath)
{
  Context c = makeOpenWaterContext();

  const int wallX = 10;
  for (int y = 0; y < kGrid; ++y)
    c.depthGrid[idx(wallX, y)] = kShallow;

  AStar astar(1);
  std::vector<Position> path = astar.search(c);

  EXPECT_TRUE(path.empty())
      << "no navigable gap exists; planner must not route through a shoal";
}

// Same full barrier, but the unsafe cells carry UNKNOWN depth rather than a
// known-shallow value. Unknown coverage must be treated exactly like an
// obstacle, so the result is still no path. This is the unit-level proof of the
// getDepth==NaN -> unsafe contract.
TEST(AStarShoalTest, UnknownDepthBarrierIsImpassable)
{
  Context c = makeOpenWaterContext();

  const int wallX = 10;
  for (int y = 0; y < kGrid; ++y)
    c.depthGrid[idx(wallX, y)] = Context::unknownDepth;

  AStar astar(1);
  std::vector<Position> path = astar.search(c);

  EXPECT_TRUE(path.empty())
      << "unknown depth must be treated as an obstacle, never as deep water";
}

// Reads outside the grid resolve to unknownDepth (an obstacle), so the search
// can never wander off the planning area. depthAt is the single chokepoint that
// guarantees this for every neighbour probe.
TEST(AStarShoalTest, OutOfBoundsReadsAsObstacle)
{
  Context c = makeOpenWaterContext();

  EXPECT_FLOAT_EQ(c.depthAt(0, 0), kDeep);
  EXPECT_EQ(c.depthAt(-1, 0), Context::unknownDepth);
  EXPECT_EQ(c.depthAt(0, -1), Context::unknownDepth);
  EXPECT_EQ(c.depthAt(kGrid, 0), Context::unknownDepth);
  EXPECT_EQ(c.depthAt(0, kGrid), Context::unknownDepth);

  // unknownDepth is below any sane minDepth, so it always reads as an obstacle.
  EXPECT_LT(Context::unknownDepth, c.minDepth);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
