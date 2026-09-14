#ifndef MISSION_INSERTION_H
#define MISSION_INSERTION_H

class MissionItem;

namespace camp::mission
{

/// [camp#22] Resolve which node a newly-created MissionItem should be inserted
/// under.
///
/// `AutonomousVehicleProject::openGeometry()` always inserted the reopened
/// VectorDataset under the project's `m_currentGroup` — a project-global "where
/// the user is working now" pointer — even when the caller knew the intended
/// parent. `MissionItem::readChildren()` is exactly that caller: restoring a
/// saved project, it dispatches a `"type": "VectorDataset"` child to
/// openGeometry, so a VectorDataset the operator had put inside a Group came
/// back at whatever `m_currentGroup` happened to point at (the root, normally)
/// instead of where they left it.
///
/// The rule: an explicitly requested parent wins; `nullptr` means "the caller has
/// no opinion", which falls back to the current group. It is a free function in
/// its own TU so the rule is testable without constructing
/// AutonomousVehicleProject, whose translation unit pulls in the entire mission
/// tree, the platform manager and the mission manager.
///
/// @param requested  The parent the caller intends, or nullptr for no opinion.
/// @param currentGroup  The project's current group — the fallback. May itself be
///                      null (a project with no root yet), in which case the
///                      result is null and the caller inserts at top level, which
///                      is what passing m_currentGroup directly used to do.
MissionItem* resolveInsertionParent(MissionItem* requested, MissionItem* currentGroup);

}  // namespace camp::mission

#endif  // MISSION_INSERTION_H
