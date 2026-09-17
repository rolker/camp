// [camp#22] Regression guard for the insertion target of a restored VectorDataset.
//
// AutonomousVehicleProject::openGeometry() always created the VectorDataset under
// the project's m_currentGroup — a global "where the user is working" pointer —
// and MissionItem::readChildren(), which dispatches a saved
// `"type": "VectorDataset"` child to openGeometry while rebuilding a project,
// passed no parent at all. A VectorDataset the operator had placed inside a Group
// therefore came back somewhere else (the root, normally) on every reload. The
// filename round-tripped; the placement did not.
//
// The rule now lives in camp::mission::resolveInsertionParent so it can be tested
// at all: no test in this suite can construct AutonomousVehicleProject (its TU
// pulls in the whole mission tree plus the platform and mission managers), and a
// rule that cannot be exercised is a rule that silently regresses. The two call
// sites that feed it — readChildren passing `this`, the File > Open Geometry
// action passing nothing — are verified by reading the source, not by this test.

#include <gtest/gtest.h>

#include "mission_insertion.h"

using camp::mission::resolveInsertionParent;

namespace
{
// Opaque stand-ins: the seam only ever compares and returns the pointer, so the
// test needs addresses, not MissionItems (which would drag in QObject + the
// project model). MissionItem is only forward-declared in the header, so these
// are reinterpret_cast'd sentinels rather than real objects — never dereferenced.
MissionItem* const kRestoringNode = reinterpret_cast<MissionItem*>(0x1000);
MissionItem* const kCurrentGroup = reinterpret_cast<MissionItem*>(0x2000);
}  // namespace

// readChildren's case: the node being restored asks for itself and wins, so a
// nested VectorDataset lands back in its own Group.
TEST(MissionInsertion, RequestedParentWins)
{
  EXPECT_EQ(resolveInsertionParent(kRestoringNode, kCurrentGroup), kRestoringNode);
}

// The menu action's case: no opinion, so the project's current group is used —
// the pre-#22 behaviour, unchanged for that caller.
TEST(MissionInsertion, NullRequestFallsBackToCurrentGroup)
{
  EXPECT_EQ(resolveInsertionParent(nullptr, kCurrentGroup), kCurrentGroup);
}

// A requested parent wins even when there is no current group to fall back to...
TEST(MissionInsertion, RequestedParentWinsWithNoCurrentGroup)
{
  EXPECT_EQ(resolveInsertionParent(kRestoringNode, nullptr), kRestoringNode);
}

// ...and with neither, the result is null — which is NOT "insert at top level":
// AutonomousVehicleProject::RowInserter dereferences the parent it is handed, so a
// caller that can see a null current group must check before inserting
// (mission_insertion.h states the contract; openGeometry() makes the check). The
// resolver's own job here is only to report that there is no parent to insert
// under, which is exactly what passing a null m_currentGroup meant before.
TEST(MissionInsertion, BothNullResolvesToNull)
{
  EXPECT_EQ(resolveInsertionParent(nullptr, nullptr), nullptr);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
