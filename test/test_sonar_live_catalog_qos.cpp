// [field 2026-08-27] Headless regression test: the `coverage_catalog` subscription QoS
// must be RELIABLE / depth 1 / **VOLATILE**.
//
// camp runs on the operator side and receives the catalog as republished by
// `udp_bridge`, which publishes VOLATILE. The subscription had been written to
// match the boat-side producer (transient-local) instead, which is
// QoS-incompatible with a volatile publisher: the subscription never matched,
// handleCatalog() never fired, and the anti-entropy reconcile (ADR-0006 D4)
// never pruned — stale coverage stayed on screen for the whole session.
// Measured live on pandy during the BizzyBoat deployment: publisher
// /operator/udp_bridge RELIABLE/VOLATILE vs subscriber /operator/camp
// RELIABLE/TRANSIENT_LOCAL.
//
// Nothing pinned this durability, which is precisely why the mismatch was
// invisible. This test pins it. No ROS graph, no Qt objects, no GL — it asserts
// on the QoS profile the subscribe site uses.

#include <gtest/gtest.h>

#include <rclcpp/qos.hpp>

#include "ros/live_coverage/sonar_live_cache_layer.h"

using camp::ros::live_coverage::catalogSubscriptionQos;

TEST(SonarLiveCatalogQos, DurabilityIsVolatileToMatchUdpBridge)
{
  const rmw_qos_profile_t profile = catalogSubscriptionQos().get_rmw_qos_profile();
  EXPECT_EQ(profile.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

TEST(SonarLiveCatalogQos, ReliableDepthOne)
{
  const rmw_qos_profile_t profile = catalogSubscriptionQos().get_rmw_qos_profile();
  EXPECT_EQ(profile.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(profile.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(profile.depth, 1u);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
