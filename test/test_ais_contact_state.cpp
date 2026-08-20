// AISContactState must never present an unknown speed or course as a number.
//
// AIS reports SOG/COG as unavailable often enough that this is the common case,
// not an edge case: the parser writes NaN into the twist, and a contact that is
// not moving has no derivable course at all. Both used to reach the hover label
// as very large numbers -- cog because it was an uninitialised member that the
// constructor only assigns when the contact is moving, and sog because casting
// NaN to int is undefined behaviour.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "ais/ais_contact.h"

namespace
{

marine_ais_msgs::msg::AISContact contactWithVelocity(double x, double y)
{
  marine_ais_msgs::msg::AISContact message;
  message.pose.position.latitude = 43.05;
  message.pose.position.longitude = -70.72;
  message.twist.twist.linear.x = x;
  message.twist.twist.linear.y = y;
  return message;
}

}  // namespace

TEST(AISContactState, DefaultConstructedValuesAreUnknownNotGarbage)
{
  AISContactState state;
  EXPECT_TRUE(std::isnan(state.sog));
  EXPECT_TRUE(std::isnan(state.cog));
  EXPECT_TRUE(std::isnan(state.heading));
}

TEST(AISContactState, UnavailableVelocityStaysUnknown)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  AISContactState state(contactWithVelocity(nan, nan));
  EXPECT_TRUE(std::isnan(state.sog));
  EXPECT_TRUE(std::isnan(state.cog)) << "NaN velocity must not leave cog unset";
}

TEST(AISContactState, StationaryContactHasNoCourse)
{
  // A stopped vessel reports a real SOG of zero, but no meaningful course.
  // The constructor cannot compute one, so cog must remain unknown rather
  // than retaining whatever was in the member beforehand.
  AISContactState state(contactWithVelocity(0.0, 0.0));
  EXPECT_FLOAT_EQ(0.0f, state.sog);
  EXPECT_TRUE(std::isnan(state.cog));
}

TEST(AISContactState, MovingContactYieldsSpeedAndCourse)
{
  AISContactState state(contactWithVelocity(3.0, 4.0));
  EXPECT_FLOAT_EQ(5.0f, state.sog);
  ASSERT_FALSE(std::isnan(state.cog));
  EXPECT_GE(state.cog, 0.0f);
  EXPECT_LT(state.cog, 360.0f);
}
