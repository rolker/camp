---
issue: 98
---

# Issue #98 — CAMP silently crashes on map zoom/pan during live ops (recurring) — likely OOM

## Issue Review
**Status**: complete
**When**: 2026-06-22 12:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #98
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Summary

The root cause (per the verified host note, 2026-06-22) is `MapTiles::paint()` accumulating
`Tile*` entries in `tiles_` without eviction: off-screen tiles are only hidden
(`setVisible(false)`) — deletion only occurs in `setLayout()`. The radar overlay's
`setRefreshInterval` path calls `setLayout()` periodically and self-bounds; the OSM/WMTS
basemap has no refresh interval and never calls `setLayout()` after initial load, so its
`tiles_` map grows monotonically with each newly-visited tile area → OOM-kill → the silent
crash. The fix is an LRU eviction cap on `tiles_` in `paint()`, scoped to the basemap
accumulation path, preserving the radar's refresh semantics and never evicting
currently-visible tiles.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Fix is mechanical memory bounding; no user-visible behavior change except elimination of silent OOM crash |
| Enforcement over documentation | OK | Fix is in code + backed by a test |
| Capture decisions, not just implementations | Watch | Eviction policy parameters (N × visible-tile cap, LRU strategy) are design choices; should be recorded in a code comment or follow-up ADR note so the bound is traceable |
| A change includes its consequences | Action needed | Issue requires a test asserting `tiles_` stays bounded across simulated pan; the `onRefreshTimer` comment in `map_tiles.cpp:150–155` calls out "paint() still only hides tiles" — this comment must be updated when the fix lands |
| Only what's needed | OK | Targeted: add eviction to paint() basemap path; preserve radar refresh semantics; no other scope |
| Improve incrementally | OK | Single class, single mechanism; completable in one PR |
| Test what breaks | Action needed | A `test_map_tiles_eviction.cpp` (or extension of `test_map_tiles_refresh.cpp`) must assert that `tiles_` size stays bounded after a simulated pan across many new tile addresses |
| Workspace vs. project separation | OK | Change is entirely within the `camp` project repo (`ui_ws/src/camp/`) |
| Safety First (project) | Watch | CAMP is the operator display; OOM mid-survey is a vigilance/safety hit. Eviction must never discard currently-visible tiles — the issue states this, but it must be enforced by the test |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0002 — Worktree isolation | Yes | Already satisfied: work is in the `issue-camp-98` worktree |
| ADR-0008 — ROS 2 conventions | Partial | Qt/C++ code in a ROS 2 package; follow C++ conventions; no new package interfaces |
| ADR-0013 — progress.md vocabulary | Yes | This entry uses `## Issue Review` per the vocabulary |

### Consequences

- `map_tiles.cpp` comment at lines 150–155 (`onRefreshTimer` note about "paint() still only hides tiles") will need updating after the eviction fix lands — it will no longer be true that paint() never deletes tiles
- Test suite: add `test_map_tiles_eviction.cpp` (or extend `test_map_tiles_refresh.cpp`) to `ui_ws/src/camp/test/`

### Actions
- [ ] Add LRU eviction cap to `MapTiles::paint()` (basemap path) so `tiles_` cannot grow unbounded; N cap should be configurable or at least documented in a comment
- [ ] Ensure eviction never removes currently-visible tiles (guard in eviction logic)
- [ ] Add test asserting `tiles_` size stays bounded across a simulated multi-area pan (follow `test_map_tiles_refresh.cpp` pattern)
- [ ] Update `onRefreshTimer` comment in `map_tiles.cpp:150–155` to reflect that eviction now also bounds within-cycle accumulation
- [ ] Document the eviction bound choice (N × visible tile count, or similar) in a comment near the eviction logic so future maintainers can trace the rationale

## Plan Authored
**Status**: complete
**When**: 2026-06-22 14:30 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-98/plan.md` at `618c2f6`
**Branch**: feature/issue-98 at `618c2f6`
**Phases**: single

### Open questions
- [ ] Confirm `kTileEvictionMinCap` and `kTileEvictionMultiplier` constants are appropriate for deployment hardware (salmon) memory constraints — lower cap may be needed if RSS budget is tight.
- [ ] Confirm that `QGraphicsScene::render()` with varying source rect reliably drives distinct `painter->worldTransform()` values inside `MapTiles::paint()` in offscreen CI; fall back to protected slot if not.

## Plan Review
**Status**: complete
**When**: 2026-06-22 05:10 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-98/plan.md` at `618c2f6`
**PR**: PR-less (`--issue` dispatch)
**Verdict**: approve-with-suggestions

### Findings
- [ ] (must-fix) Step 2 deletes `Tile` (a `QGraphicsObject` child of `MapTiles`) inside `paint()` — removing scene items mid-paint risks use-after-free per Qt; defer eviction to the event loop and promote the planned `evictIfNeeded` slot from test-fallback to the primary deletion path (`QMetaObject::invokeMethod(..., Qt::QueuedConnection)` / `QTimer::singleShot(0,...)`) — `plan.md:33`
- [ ] (suggestion) Step 5 test reuses single-zoom-level `makeLayout`, but `setLayout()` pre-seeds the whole `zoom_levels.front()` grid at construction, so `paint()` mints no new tiles — the test won't reproduce the field mechanism (deeper-zoom tiles minted by `paint()`); use a multi-zoom-level layout and `processEvents()` after `render()` if eviction is deferred — `plan.md:51`
- [ ] (suggestion) Confirm `tile_last_visible_gen_` keys identically to `tiles_` — `TileAddress::operator<` ignores `layout_epoch_` (`map_tiles.cpp:183`), which is the desired behavior; add a one-line comment so the shared comparator semantics aren't silently broken — `plan.md:30`
- [ ] (noted, no action) Evicting a tile with an in-flight pixmap is already safe — `tileLoaded()`'s `tiles_.find` guard drops late pixmaps for evicted addresses (`map_tiles.cpp:182`)

### Summary
Diagnosis is correct; LRU-eviction approach is sound, minimal, and well-scoped with consequences and tests accounted for. The one structural concern — synchronous `Tile` deletion inside `paint()` — should be resolved by deferring eviction (the plan's own `evictIfNeeded` slot supports this) before implementation. With that addressed, the plan is ready.

### Recommended Actions
- [ ] Defer eviction out of `paint()`; make `evictIfNeeded` the primary deletion path.
- [ ] Use a multi-zoom-level test layout; `processEvents()` after `render()` when eviction is deferred.
- [ ] Comment the shared `TileAddress` comparator semantics on `tile_last_visible_gen_`.
