# Plan: ais: remove dead legacy marine_interfaces/Contact references (frees the Contact name)

## Issue

https://github.com/rolker/camp/issues/93

## Context

CAMP's live AIS path runs entirely on `marine_ais_msgs/msg/AISContact`:
`AISManager::scanForSources` subscribes only to `marine_ais_msgs/msg/AISContact`
(`ais_manager.cpp:30,36`) → `aisContactCallback` (`:46`), fed by
`marine_ais_tools/ais_contact_tracker.py`.

The legacy `marine_interfaces::msg::Contact` surface in `ais/*` is dead:
- `AISManager::contactCallback` is **declared** (`ais_manager.h:40`) but has **no
  definition** in `ais_manager.cpp` and is never connected to a subscription.
- The three `const marine_interfaces::msg::Contact&` constructor overloads in
  `ais_contact.{h,cpp}` have no callers (the live path uses the `AISContact`
  overloads).
- Nothing in-tree publishes `marine_interfaces/Contact`.

Removing this dead code frees the `Contact` name so `rolker/unh_marine_autonomy#156`
can redefine `Contact` as the unified perception contact. This is the prerequisite
cleanup; #156 task 4 (deleting `marine_interfaces/msg/Contact.msg`) must wait until
this merges or camp's build breaks on the missing `contact.hpp`.

## Approach

1. **`ais_contact.h`** — remove the legacy `Contact` declarations:
   - delete `#include "marine_interfaces/msg/contact.hpp"` (line 6)
   - delete `AISContactDetails(const marine_interfaces::msg::Contact&)` (line 15)
   - delete `AISContactState(const marine_interfaces::msg::Contact&)` (line 28)
   - delete `AISReport(const marine_interfaces::msg::Contact&, QObject*)` (line 42)
2. **`ais_contact.cpp`** — remove the matching definitions:
   - `AISContactDetails(const marine_interfaces::msg::Contact&)` (lines 13–21)
   - `AISContactState(const marine_interfaces::msg::Contact&)` (lines 39–53)
   - `AISReport(const marine_interfaces::msg::Contact&, QObject*)` (lines 93–99)
3. **`ais_manager.h`** — remove `#include "marine_interfaces/msg/contact.hpp"`
   (line 5) and the dangling `void contactCallback(const marine_interfaces::msg::Contact&)`
   declaration (line 40).
4. **Leave the `marine_interfaces` package dependency in place** —
   `package.xml`/`CMakeLists.txt` keep it; camp still uses `marine_interfaces` in
   `mission_manager`, `platform_manager`, `helm_manager`, `nav_source`,
   `autonomousvehicleproject`. This change touches only `ais/*`.
5. **Build to verify** — `./ui_ws/build.sh camp` (or build the camp package) to
   confirm no remaining references and a clean compile.

## Files to Change

| File | Change |
|------|--------|
| `src/camp/ais/ais_contact.h` | Drop legacy `Contact` include + 3 ctor decls |
| `src/camp/ais/ais_contact.cpp` | Drop the 3 legacy `Contact` ctor definitions |
| `src/camp/ais/ais_manager.h` | Drop legacy `Contact` include + dead `contactCallback` decl |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | Build-verify after removal; confirm no docs reference the removed ctors (none found). Coordinate ordering with #156 task 4 (noted in issue). |
| Only what's needed | Remove only the dead legacy-`Contact` surface; keep the `marine_interfaces` dep and the live `AISContact` path untouched. |
| Improve incrementally | Self-contained, reviewable cleanup; no behavior change (dead code only). |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ws ADR-0002 — Worktree isolation | Yes | Work in `issue-camp-93` worktree on `feature/issue-93`; PR into `jazzy`. |
| ws ADR-0008 — ROS 2 conventions | No | No new package/msg/launch; only removes a message usage. |
| camp ADR-0003 — backgrounds-as-layers | No | Untouched; the `[#59 ADR-0003]` comments in `ais_contact.cpp` are in the live path and stay. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Remove a message consumer (`ais/*`) | Build still compiles; the `marine_interfaces` dep stays (other packages need it) | Yes — step 4/5 |
| Frees `Contact` name | `unh_marine_autonomy#156` task 4 may then delete `Contact.msg` | Cross-repo ordering noted in issue; out of scope here |

## Open Questions

- Verification: camp has no CI and no unit tests for `ais/*`, so the gate is a
  clean build (+ optional sim smoke-check that AIS still renders). Is a build
  pass sufficient for merge, or do you want a sim-verify of the AIS layer first?

## Estimated Scope

Single small PR (3 files, dead-code removal, no behavior change).
