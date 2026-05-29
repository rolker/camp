// Tests for camp_collision_monitor::convertPolygon — the per-polygon logic
// that runs inside the TF dispatcher's converter on the executor thread.
//
// Exercises empty-frame handling, unreachable-frame handling, and TF-driven
// per-vertex geo conversion. The threading and TF gating around the converter
// are tested separately in test_topic_bridge.cpp.

#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "collision_monitor/collision_monitor_converter.h"

namespace
{

// Earth-fixed -> child transform placing `child` at the WGS-84 surface near
// (lat=0, lon=0), avoiding the singular (0,0,0) ECEF point that converts to
// NaN lat/lon. Mirrors test_markers_converter.cpp.
geometry_msgs::msg::TransformStamped surfaceTransform(
  const std::string & parent, const std::string & child, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = stamp;
  t.header.frame_id = parent;
  t.child_frame_id = child;
  t.transform.rotation.w = 1.0;
  t.transform.translation.x = 6378137.0;  // WGS-84 semi-major axis (meters)
  return t;
}

geometry_msgs::msg::Point32 point32(float x, float y)
{
  geometry_msgs::msg::Point32 p;
  p.x = x;
  p.y = y;
  p.z = 0.0f;
  return p;
}

}  // namespace

class CollisionMonitorConverterTest : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<tf2_ros::Buffer> buffer;

  void SetUp() override
  {
    static std::atomic<unsigned> counter{0};
    auto n = std::to_string(counter.fetch_add(1));
    node = rclcpp::Node::make_shared("collision_monitor_converter_test_" + n);
    buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  }

  rclcpp::Time now() const { return node->get_clock()->now(); }
  rclcpp::Logger logger() const { return node->get_logger(); }
};

TEST_F(CollisionMonitorConverterTest, validPolygonProducesAllVertices)
{
  buffer->setTransform(surfaceTransform("earth", "base_link", now()),
                       "test", /*is_static=*/true);

  geometry_msgs::msg::PolygonStamped poly;
  poly.header.frame_id = "base_link";
  poly.header.stamp = now();
  poly.polygon.points = {point32(1, 1), point32(1, -1), point32(-1, -1), point32(-1, 1)};

  auto out = camp_collision_monitor::convertPolygon(poly, *buffer, logger());
  ASSERT_TRUE(out.has_value());
  ASSERT_EQ((*out)->points.size(), 4u);
  for(const auto& gc : (*out)->points)
    EXPECT_TRUE(gc.isValid());
}

TEST_F(CollisionMonitorConverterTest, emptyFrameIdIsDropped)
{
  geometry_msgs::msg::PolygonStamped poly;
  poly.header.frame_id = "";  // empty
  poly.header.stamp = now();
  poly.polygon.points = {point32(1, 1), point32(-1, -1)};

  auto out = camp_collision_monitor::convertPolygon(poly, *buffer, logger());
  EXPECT_FALSE(out.has_value())
    << "polygon with empty frame_id must be dropped, not propagated";
}

TEST_F(CollisionMonitorConverterTest, unreachableFrameIsDropped)
{
  // No TF set up — frame is not reachable to "earth".
  geometry_msgs::msg::PolygonStamped poly;
  poly.header.frame_id = "no_such_frame";
  poly.header.stamp = now();
  poly.polygon.points = {point32(1, 1), point32(-1, -1)};

  auto out = camp_collision_monitor::convertPolygon(poly, *buffer, logger());
  EXPECT_FALSE(out.has_value());
}

TEST_F(CollisionMonitorConverterTest, emptyPolygonProducesEmptyPayload)
{
  // A degenerate (no-vertex) polygon with a valid frame is not an error; it
  // yields an empty point list (the overlay simply draws nothing).
  buffer->setTransform(surfaceTransform("earth", "base_link", now()),
                       "test", /*is_static=*/true);

  geometry_msgs::msg::PolygonStamped poly;
  poly.header.frame_id = "base_link";
  poly.header.stamp = now();

  auto out = camp_collision_monitor::convertPolygon(poly, *buffer, logger());
  ASSERT_TRUE(out.has_value());
  EXPECT_TRUE((*out)->points.empty());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
