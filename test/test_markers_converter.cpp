// Tests for camp_markers::convertMarker — the per-marker logic that runs
// inside the TF dispatcher's converter on the executor thread.
//
// These tests exercise marker-action handling, lifetime expiry, missing
// frame_id handling, and TF-driven geo conversion. The threading and TF
// gating around the converter are tested separately in test_topic_bridge.cpp.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "markers/markers_converter.h"

using std::chrono_literals::operator""s;

namespace
{

geometry_msgs::msg::TransformStamped identityTransform(
  const std::string & parent, const std::string & child, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = stamp;
  t.header.frame_id = parent;
  t.child_frame_id = child;
  t.transform.rotation.w = 1.0;
  return t;
}

// Earth-fixed → child transform that places `child` at the surface near the
// equator/prime meridian (ECEF ~ (a, 0, 0), where a is the WGS-84 semi-major
// axis). Avoids the singular (0, 0, 0) ECEF point that converts to NaN
// lat/lon.
geometry_msgs::msg::TransformStamped surfaceTransform(
  const std::string & parent, const std::string & child, const rclcpp::Time & stamp)
{
  auto t = identityTransform(parent, child, stamp);
  t.transform.translation.x = 6378137.0;  // WGS-84 semi-major axis (meters)
  return t;
}

}  // namespace

class MarkersConverterTest : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<tf2_ros::Buffer> buffer;

  void SetUp() override
  {
    static std::atomic<unsigned> counter{0};
    auto n = std::to_string(counter.fetch_add(1));
    node = rclcpp::Node::make_shared("markers_converter_test_" + n);
    buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  }

  rclcpp::Time now() const { return node->get_clock()->now(); }
  rclcpp::Logger logger() const { return node->get_logger(); }
};

TEST_F(MarkersConverterTest, addMarkerWithValidTfProducesPayload)
{
  buffer->setTransform(surfaceTransform("earth", "base_link", now()),
                       "test", /*is_static=*/true);

  visualization_msgs::msg::Marker m;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.ns = "ns_a";
  m.id = 1;
  m.header.frame_id = "base_link";
  m.header.stamp = now();
  m.pose.orientation.w = 1.0;

  auto out = camp_markers::convertMarker(m, *buffer, now(), logger());
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ((*out)->marker.ns, "ns_a");
  EXPECT_EQ((*out)->marker.id, 1);
  // earth-frame identity at the origin is roughly the geo origin (lat=0, lon=0).
  EXPECT_TRUE((*out)->position.isValid());
}

TEST_F(MarkersConverterTest, addMarkerWithEmptyFrameIdIsDropped)
{
  visualization_msgs::msg::Marker m;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.header.frame_id = "";  // empty
  m.header.stamp = now();

  auto out = camp_markers::convertMarker(m, *buffer, now(), logger());
  EXPECT_FALSE(out.has_value())
    << "ADD with empty frame_id must be dropped, not propagated";
}

TEST_F(MarkersConverterTest, addMarkerWithUnreachableFrameIsDropped)
{
  // No TF set up — frame is not reachable.
  visualization_msgs::msg::Marker m;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.header.frame_id = "no_such_frame";
  m.header.stamp = now();
  m.pose.orientation.w = 1.0;

  auto out = camp_markers::convertMarker(m, *buffer, now(), logger());
  EXPECT_FALSE(out.has_value());
}

TEST_F(MarkersConverterTest, addMarkerWithExpiredLifetimeIsDropped)
{
  buffer->setTransform(surfaceTransform("earth", "base_link", now()),
                       "test", /*is_static=*/true);

  // Marker stamped 10 s in the past with a 1 s lifetime → already expired.
  auto stamp = rclcpp::Time(now()) - rclcpp::Duration(10s);
  visualization_msgs::msg::Marker m;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.header.frame_id = "base_link";
  m.header.stamp = stamp;
  m.lifetime = rclcpp::Duration(1s);
  m.pose.orientation.w = 1.0;

  auto out = camp_markers::convertMarker(m, *buffer, now(), logger());
  EXPECT_FALSE(out.has_value())
    << "ADD with expired lifetime must be dropped";
}

TEST_F(MarkersConverterTest, addMarkerWithZeroLifetimeIsAcceptedRegardlessOfStamp)
{
  buffer->setTransform(surfaceTransform("earth", "base_link", now()),
                       "test", /*is_static=*/true);

  // 1 hour ago, lifetime=0 means "infinite" — must not be dropped.
  auto stamp = rclcpp::Time(now()) - rclcpp::Duration(3600s);
  visualization_msgs::msg::Marker m;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.header.frame_id = "base_link";
  m.header.stamp = stamp;
  m.lifetime = rclcpp::Duration(0s);
  m.pose.orientation.w = 1.0;

  auto out = camp_markers::convertMarker(m, *buffer, now(), logger());
  EXPECT_TRUE(out.has_value())
    << "ADD with lifetime=0 means 'permanent' and must not be dropped";
}

TEST_F(MarkersConverterTest, deleteActionPassesThroughWithoutTf)
{
  // No TF set — DELETE actions don't need a position lookup, but in the
  // current shape the converter still produces a payload (with no position)
  // so the receiver can act on the action.
  visualization_msgs::msg::Marker m;
  m.action = visualization_msgs::msg::Marker::DELETE;
  m.ns = "ns_b";
  m.id = 7;
  m.header.frame_id = "anything";
  m.header.stamp = now();

  auto out = camp_markers::convertMarker(m, *buffer, now(), logger());
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ((*out)->marker.action, visualization_msgs::msg::Marker::DELETE);
  EXPECT_EQ((*out)->marker.ns, "ns_b");
  EXPECT_EQ((*out)->marker.id, 7);
}

TEST_F(MarkersConverterTest, deleteAllActionPassesThrough)
{
  visualization_msgs::msg::Marker m;
  m.action = visualization_msgs::msg::Marker::DELETEALL;
  m.ns = "ns_c";
  m.header.stamp = now();

  auto out = camp_markers::convertMarker(m, *buffer, now(), logger());
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ((*out)->marker.action, visualization_msgs::msg::Marker::DELETEALL);
  EXPECT_EQ((*out)->marker.ns, "ns_c");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
