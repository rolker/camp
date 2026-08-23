// Unit tests for ShorelineAnchor — the source-agnostic anchor seam
// (camp#181, camp ADR-0015 D3/D5).
//
// The two contracts worth breaking a build over:
//
//  D3 — FALLBACK ORDER. Resolution walks downward from the selected mode to the
//       first source with a value: chart datum > platform tide > manual > none.
//       Order is declaration order, so a reordering of the enum silently changes
//       display semantics; these tests pin it.
//
//  D5 — AN ABSENT ANCHOR IS NEVER 0.0. This is the one that would actually hurt
//       someone. Anchor values are ellipsoidal heights, and chart datum at the
//       Isles of Shoals is -28.038 m. Substituting 0.0 for "no anchor" would put
//       the land/sea colour break ~28 m into deep water — a plausible-looking
//       display that is wrong in the direction that matters when the question is
//       under-keel clearance among rocks. nullopt must stay nullopt.
//
// Also covered: changed() fires on a real move and NOT on a no-op refresh (a
// source re-pushing an identical value must not cost a repaint), and PR1's
// honesty property — ChartDatum/PlatformTide are selectable and correctly
// ordered but nothing resolves them yet, so selecting one falls through rather
// than pretending.
#include <gtest/gtest.h>

#include <limits>

#include <optional>

#include <QObject>

#include "../src/camp_map/raster/shoreline_anchor.h"

using camp::raster::ShorelineAnchor;
using Source = camp::raster::ShorelineAnchor::Source;

namespace
{

// Counts changed() emissions. A direct connection needs no event loop.
class ChangeCounter
{
public:
  explicit ChangeCounter(ShorelineAnchor* anchor)
  {
    QObject::connect(anchor, &ShorelineAnchor::changed, [this]() { ++count_; });
  }
  int count() const { return count_; }
private:
  int count_ = 0;
};

}  // namespace

TEST(ShorelineAnchorTest, RuntimeDefaultIsUnanchoredNotZero)
{
  // PR1's runtime default. The POLICY default is chart datum (ADR-0015), but it
  // has no source until PR2, so the honest runtime state is unanchored.
  ShorelineAnchor anchor;
  EXPECT_EQ(anchor.mode(), Source::None);
  EXPECT_FALSE(anchor.value().has_value()) << "D5: absent anchor must not resolve";
  EXPECT_EQ(anchor.activeSource(), Source::None);
}

TEST(ShorelineAnchorTest, AbsentAnchorIsNulloptNeverZero)
{
  // D5, stated as bluntly as possible: every no-resolution path returns nullopt.
  ShorelineAnchor anchor;
  for(Source mode : {Source::ChartDatum, Source::PlatformTide, Source::Manual, Source::None})
  {
    anchor.setMode(mode);
    ASSERT_FALSE(anchor.value().has_value())
      << "a mode with no available source must resolve to nullopt, not 0.0";
    EXPECT_EQ(anchor.activeSource(), Source::None);
  }
}

TEST(ShorelineAnchorTest, ManualResolvesAndReportsItsSource)
{
  ShorelineAnchor anchor;
  anchor.setManual(-28.038);
  anchor.setMode(Source::Manual);
  ASSERT_TRUE(anchor.value().has_value());
  EXPECT_DOUBLE_EQ(*anchor.value(), -28.038);
  EXPECT_EQ(anchor.activeSource(), Source::Manual);
}

TEST(ShorelineAnchorTest, D3FallbackWalksDownwardToTheFirstAvailableSource)
{
  ShorelineAnchor anchor;
  anchor.setManual(-28.0);
  anchor.setPlatformTide(-26.5);

  // Chart datum selected but unavailable -> falls to platform tide.
  anchor.setMode(Source::ChartDatum);
  ASSERT_TRUE(anchor.value().has_value());
  EXPECT_DOUBLE_EQ(*anchor.value(), -26.5);
  EXPECT_EQ(anchor.activeSource(), Source::PlatformTide)
    << "fall-through must be REPORTED, not silently applied";

  // Chart datum arrives -> it wins, being higher in the order.
  anchor.setChartDatum(-28.038);
  ASSERT_TRUE(anchor.value().has_value());
  EXPECT_DOUBLE_EQ(*anchor.value(), -28.038);
  EXPECT_EQ(anchor.activeSource(), Source::ChartDatum);

  // Tide drops out -> chart datum still wins; nothing below it is consulted.
  anchor.setPlatformTide(std::nullopt);
  EXPECT_DOUBLE_EQ(*anchor.value(), -28.038);

  // Chart datum drops out -> falls past the now-empty tide to manual.
  anchor.setChartDatum(std::nullopt);
  ASSERT_TRUE(anchor.value().has_value());
  EXPECT_DOUBLE_EQ(*anchor.value(), -28.0);
  EXPECT_EQ(anchor.activeSource(), Source::Manual);
}

TEST(ShorelineAnchorTest, ModeDoesNotConsultSourcesAboveIt)
{
  // Selecting Manual must not silently upgrade to chart datum just because a
  // datum value exists — the operator asked for manual.
  ShorelineAnchor anchor;
  anchor.setChartDatum(-28.038);
  anchor.setManual(-10.0);
  anchor.setMode(Source::Manual);
  ASSERT_TRUE(anchor.value().has_value());
  EXPECT_DOUBLE_EQ(*anchor.value(), -10.0);
  EXPECT_EQ(anchor.activeSource(), Source::Manual);
}

TEST(ShorelineAnchorTest, NoneRendersUnanchoredEvenWhenSourcesHaveValues)
{
  ShorelineAnchor anchor;
  anchor.setChartDatum(-28.038);
  anchor.setManual(-10.0);
  anchor.setMode(Source::None);
  EXPECT_FALSE(anchor.value().has_value()) << "None is a deliberate choice, not a fallback";
}

TEST(ShorelineAnchorTest, ChangedFiresOnRealMovesAndNotOnNoOpRefresh)
{
  ShorelineAnchor anchor;
  ChangeCounter counter(&anchor);

  anchor.setManual(-28.0);
  anchor.setMode(Source::Manual);
  const int after_setup = counter.count();
  EXPECT_GT(after_setup, 0) << "a resolved anchor appearing is a real move";

  // Same value again: the resolved anchor did not move, so no repaint is owed.
  anchor.setManual(-28.0);
  EXPECT_EQ(counter.count(), after_setup)
    << "an identical refresh must not cost a repaint";

  // A genuine move.
  anchor.setManual(-27.0);
  EXPECT_GT(counter.count(), after_setup);
}

TEST(ShorelineAnchorTest, InvalidToValidTransitionIsReported)
{
  // The tide/datum sources start empty and populate later; that transition has
  // to reach the layers or the display stays stale.
  ShorelineAnchor anchor;
  anchor.setMode(Source::ChartDatum);
  ChangeCounter counter(&anchor);
  ASSERT_FALSE(anchor.value().has_value());

  anchor.setChartDatum(-28.038);
  EXPECT_GT(counter.count(), 0) << "invalid -> valid must emit changed()";
  ASSERT_TRUE(anchor.value().has_value());
  EXPECT_EQ(anchor.activeSource(), Source::ChartDatum);
}

// [camp#181 / ADR-0015 D5] A mode-only transition between two UNRESOLVABLE sources
// moves nothing resolved — but the layer status names mode() in that state
// ("shoreline chart datum unavailable"), so it must still reach the layers or the
// readout keeps naming the source the operator just left.
TEST(ShorelineAnchorTest, UnresolvableModeChangeStillReportsItself)
{
  ShorelineAnchor anchor;
  anchor.setMode(Source::ChartDatum);        // no datum provider in PR1
  ChangeCounter counter(&anchor);
  ASSERT_FALSE(anchor.value().has_value());

  anchor.setMode(Source::PlatformTide);      // no tide provider either
  EXPECT_FALSE(anchor.value().has_value());
  EXPECT_EQ(anchor.mode(), Source::PlatformTide);
  EXPECT_GT(counter.count(), 0)
    << "the status names mode(); a mode change must be reported even when the "
       "resolved anchor does not move";
}

// [camp#181 / ADR-0015] A non-finite push is not an anchor. NaN != NaN would defeat
// the holder's own no-op test (every identical re-push would emit changed()) and
// the renderer's LUT cache key (a bake + texture upload every frame), and the bake
// falls back to the unanchored ramp anyway — so the layer would claim an anchor it
// is not applying. It is reported as an absence instead.
TEST(ShorelineAnchorTest, NonFiniteValuesAreRejectedAsAbsent)
{
  ShorelineAnchor anchor;
  anchor.setMode(Source::Manual);
  anchor.setManual(std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(anchor.manualValue().has_value());
  EXPECT_FALSE(anchor.value().has_value()) << "a NaN anchor is no anchor, not 0.0";

  anchor.setChartDatum(std::numeric_limits<double>::infinity());
  EXPECT_FALSE(anchor.value().has_value());

  // A finite value after a rejected one still lands, and identical re-pushes are
  // still free (the equality the NaN would have broken).
  anchor.setManual(-28.038);
  ASSERT_TRUE(anchor.value().has_value());
  ChangeCounter counter(&anchor);
  anchor.setManual(-28.038);
  EXPECT_EQ(counter.count(), 0) << "an identical refresh must not cost a repaint";
}

TEST(ShorelineAnchorTest, SourceLabelsAreDistinctAndNonEmpty)
{
  // The layer status and the dialog readout share these, so they cannot drift.
  for(Source s : {Source::ChartDatum, Source::PlatformTide, Source::Manual, Source::None})
    EXPECT_FALSE(ShorelineAnchor::sourceLabel(s).isEmpty());
  EXPECT_NE(ShorelineAnchor::sourceLabel(Source::ChartDatum),
            ShorelineAnchor::sourceLabel(Source::PlatformTide));
  EXPECT_NE(ShorelineAnchor::sourceLabel(Source::Manual),
            ShorelineAnchor::sourceLabel(Source::None));
}
