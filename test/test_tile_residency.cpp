// [camp#195 / uma-ADR-0013 D4] Unit tests for the TileResidency splice
// partition — the structure that makes "cannot evict what this frame selected"
// a property of the type rather than a checked condition.
//
// Pure: no Qt, no GL, no GDAL. The invariant under test is the one an eviction
// pass depends on — candidates() never contains an index protected in the
// current frame — plus the frame/epoch mechanics that let beginFrame() be O(1).

#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "raster/tile_residency.h"

namespace
{

std::vector<std::size_t> asVector(const camp::raster::TileResidency& residency)
{
  return std::vector<std::size_t>(residency.candidates().begin(),
                                  residency.candidates().end());
}

bool contains(const std::vector<std::size_t>& v, std::size_t index)
{
  return std::find(v.begin(), v.end(), index) != v.end();
}

}  // namespace

// Freshly synced tiles are all candidates: nothing is protected until a frame
// selects it.
TEST(TileResidencyTest, SyncAdmitsEverythingAsEvictable)
{
  camp::raster::TileResidency residency;
  residency.sync(4);
  EXPECT_EQ(residency.size(), 4u);
  EXPECT_EQ(residency.protectedCount(), 0u);
  EXPECT_EQ(asVector(residency).size(), 4u);
}

// The core guarantee: a protected index is absent from candidates(). This is
// what the eviction pass relies on — it is handed candidates() and nothing else.
TEST(TileResidencyTest, ProtectedIndexIsNeverACandidate)
{
  camp::raster::TileResidency residency;
  residency.sync(5);
  residency.beginFrame();
  residency.protect(1);
  residency.protect(3);

  const std::vector<std::size_t> candidates = asVector(residency);
  EXPECT_EQ(candidates.size(), 3u);
  EXPECT_FALSE(contains(candidates, 1));
  EXPECT_FALSE(contains(candidates, 3));
  EXPECT_TRUE(contains(candidates, 0));
  EXPECT_TRUE(contains(candidates, 2));
  EXPECT_TRUE(contains(candidates, 4));
  EXPECT_EQ(residency.protectedCount(), 2u);
  EXPECT_TRUE(residency.isProtected(1));
  EXPECT_FALSE(residency.isProtected(0));
}

// protect() is idempotent within a frame — paint() may protect the same tile
// through more than one predicate (selected AND hole-covering), and a double
// splice would corrupt the partition sizes.
TEST(TileResidencyTest, RepeatedProtectWithinAFrameIsIdempotent)
{
  camp::raster::TileResidency residency;
  residency.sync(3);
  residency.beginFrame();
  residency.protect(2);
  residency.protect(2);
  residency.protect(2);

  EXPECT_EQ(residency.protectedCount(), 1u);
  EXPECT_EQ(asVector(residency).size(), 2u);
}

// beginFrame() returns the WHOLE protected partition to the candidates, so a
// tile the previous frame selected is evictable again once it leaves the view.
TEST(TileResidencyTest, BeginFrameReleasesThePreviousFramesProtection)
{
  camp::raster::TileResidency residency;
  residency.sync(4);
  residency.beginFrame();
  residency.protect(0);
  residency.protect(1);
  ASSERT_EQ(residency.protectedCount(), 2u);

  residency.beginFrame();
  EXPECT_EQ(residency.protectedCount(), 0u);
  EXPECT_EQ(asVector(residency).size(), 4u);
  EXPECT_FALSE(residency.isProtected(0));

  // A new frame protects a different tile; the old ones stay candidates.
  residency.protect(3);
  const std::vector<std::size_t> candidates = asVector(residency);
  EXPECT_TRUE(contains(candidates, 0));
  EXPECT_TRUE(contains(candidates, 1));
  EXPECT_FALSE(contains(candidates, 3));
}

// rescan() appends tiles mid-session; sync() must admit them without disturbing
// the current frame's protection (the iterators of existing entries stay valid
// across the append, which is why protect() can stay O(1)).
TEST(TileResidencyTest, SyncAppendsWithoutDisturbingProtection)
{
  camp::raster::TileResidency residency;
  residency.sync(2);
  residency.beginFrame();
  residency.protect(0);

  residency.sync(5);   // rescan() found three more tiles
  EXPECT_EQ(residency.size(), 5u);
  EXPECT_EQ(residency.protectedCount(), 1u);
  EXPECT_TRUE(residency.isProtected(0));

  const std::vector<std::size_t> candidates = asVector(residency);
  EXPECT_EQ(candidates.size(), 4u);
  EXPECT_FALSE(contains(candidates, 0));
  EXPECT_TRUE(contains(candidates, 4));

  // The newly admitted tiles are protectable like any other.
  residency.protect(4);
  EXPECT_EQ(residency.protectedCount(), 2u);
  EXPECT_FALSE(contains(asVector(residency), 4));
}

// sync() is append-only: the owner's tile vector never shrinks, and a smaller
// count must not drop entries the frame may still be protecting.
TEST(TileResidencyTest, SyncIgnoresShrink)
{
  camp::raster::TileResidency residency;
  residency.sync(4);
  residency.sync(2);
  EXPECT_EQ(residency.size(), 4u);
  EXPECT_EQ(asVector(residency).size(), 4u);
}

// Out-of-range protection is a no-op rather than undefined behaviour: the layer
// syncs before protecting, but a stale index must never corrupt the partition.
TEST(TileResidencyTest, OutOfRangeProtectIsIgnored)
{
  camp::raster::TileResidency residency;
  residency.sync(2);
  residency.beginFrame();
  residency.protect(7);
  EXPECT_EQ(residency.protectedCount(), 0u);
  EXPECT_FALSE(residency.isProtected(7));
  EXPECT_EQ(asVector(residency).size(), 2u);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
