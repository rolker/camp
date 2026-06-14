---
issue: 93
---

# Issue #93 — ais: remove dead legacy marine_interfaces/Contact references (frees the Contact name)

## Plan Authored
**Status**: complete
**When**: 2026-06-14 11:45 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-93/plan.md` at `a15b6a8`
**PR**: https://github.com/rolker/camp/pull/94 (`[PLAN]` prefix)
**Phases**: single

### Open questions
- [x] Verification gate: build is sufficient (Roland confirmed 2026-06-14 — "build test is enough", no sim-verify needed).

## Implementation
**Status**: complete
**When**: 2026-06-14 12:05 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Commit**: `db1ed5f` — ais: remove dead legacy marine_interfaces/Contact references
**Verification**: `./ui_ws/build.sh camp` — package built clean (2min 4s; only pre-existing unused-parameter warnings in unrelated files).

### What changed
- Removed 2 `#include "marine_interfaces/msg/contact.hpp"`, 3 legacy `Contact` ctor overloads (`ais_contact.{h,cpp}`), and the dangling `contactCallback` decl (`ais_manager.h`). 40 deletions, no behavior change.
- `marine_interfaces` package dependency left in place (still used by mission_manager/platform_manager/helm_manager/nav_source).
- No plan deviations.

### Downstream
- [ ] Unblocks `rolker/unh_marine_autonomy#156` task 4 (delete `marine_interfaces/msg/Contact.msg`) — only after this PR merges.
