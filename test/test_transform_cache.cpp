// Tests for camp::ros::lookupEarthTransformCached — the cache-with-staleness
// decision that lets a transient TF gap reuse the last good earth<-frame
// transform instead of dropping the layer's frame. A zero-timeout lookup that
// dropped on every miss caused white flicker / dropped maps; this pins down the
// fresh / reuse-within-budget / expire / cold-start branches. The per-instance
// mutex wrapper lives in Layer::lookupEarthTransform.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "ros/transform_cache.h"

using namespace std::chrono_literals;

namespace
{
// earth -> child with a recognisable translation, so a returned transform can
// be told apart from a stale cached one by its x component.
geometry_msgs::msg::TransformStamped earthTransform(const std::string& child, double tx)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "earth";
  t.child_frame_id = child;
  t.transform.rotation.w = 1.0;
  t.transform.translation.x = tx;
  return t;
}
}  // namespace

class TransformCacheTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    auto node = std::make_shared<rclcpp::Node>("transform_cache_test");
    buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  }

  std::shared_ptr<tf2_ros::Buffer> buffer;
  camp::ros::TransformCache cache;
  // A fixed, clock-independent base instant so offsets below are exact.
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::time_point{} + 100s;
  std::chrono::duration<double> budget{1.0};
};

// Nothing in the buffer, nothing cached: the lookup failure propagates.
TEST_F(TransformCacheTest, ColdStartThrows)
{
  EXPECT_THROW(
    camp::ros::lookupEarthTransformCached(*buffer, cache, "map", t0, budget),
    tf2::TransformException);
  EXPECT_TRUE(cache.empty());
}

// A live lookup returns the live transform and seeds the cache at `now`.
TEST_F(TransformCacheTest, FreshLookupCaches)
{
  buffer->setTransform(earthTransform("map", 11.0), "test", /*is_static=*/true);

  auto out = camp::ros::lookupEarthTransformCached(*buffer, cache, "map", t0, budget);

  EXPECT_DOUBLE_EQ(out.transform.translation.x, 11.0);
  ASSERT_EQ(cache.count("map"), 1u);
  EXPECT_EQ(cache.at("map").stamp, t0);
}

// Live lookup fails but a cache entry younger than the budget is reused.
TEST_F(TransformCacheTest, WithinBudgetReuses)
{
  cache["map"] = {earthTransform("map", 22.0), t0};  // buffer left empty -> lookup throws

  auto out = camp::ros::lookupEarthTransformCached(*buffer, cache, "map", t0 + 500ms, budget);

  EXPECT_DOUBLE_EQ(out.transform.translation.x, 22.0);
}

// Boundary: an entry exactly at the budget age still reuses (<=).
TEST_F(TransformCacheTest, AtBudgetReuses)
{
  cache["map"] = {earthTransform("map", 22.0), t0};

  auto out = camp::ros::lookupEarthTransformCached(*buffer, cache, "map", t0 + 1000ms, budget);

  EXPECT_DOUBLE_EQ(out.transform.translation.x, 22.0);
}

// Past the budget the stale entry is abandoned and the failure propagates.
TEST_F(TransformCacheTest, OverBudgetThrows)
{
  cache["map"] = {earthTransform("map", 22.0), t0};

  EXPECT_THROW(
    camp::ros::lookupEarthTransformCached(*buffer, cache, "map", t0 + 1500ms, budget),
    tf2::TransformException);
}

// A later successful lookup overwrites the value and refreshes the stamp,
// re-extending the reuse window from the new instant.
TEST_F(TransformCacheTest, SuccessRefreshesStamp)
{
  cache["map"] = {earthTransform("map", 22.0), t0};
  buffer->setTransform(earthTransform("map", 33.0), "test", /*is_static=*/true);

  auto out = camp::ros::lookupEarthTransformCached(*buffer, cache, "map", t0 + 5s, budget);

  EXPECT_DOUBLE_EQ(out.transform.translation.x, 33.0);  // live value, not stale 22
  EXPECT_EQ(cache.at("map").stamp, t0 + 5s);             // stamp advanced past the old base
}

// A gap on one frame must not evict another frame's still-fresh entry.
TEST_F(TransformCacheTest, PerFrameIndependence)
{
  cache["map"] = {earthTransform("map", 44.0), t0};

  // "odom" has nothing cached and nothing live -> throws, leaving "map" intact.
  EXPECT_THROW(
    camp::ros::lookupEarthTransformCached(*buffer, cache, "odom", t0 + 200ms, budget),
    tf2::TransformException);

  auto out = camp::ros::lookupEarthTransformCached(*buffer, cache, "map", t0 + 200ms, budget);
  EXPECT_DOUBLE_EQ(out.transform.translation.x, 44.0);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
