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

## Implementation
**Status**: complete
**When**: 2026-06-22 05:30 +00:00
**By**: Claude Opus

**Branch**: feature/issue-98
**Commits**: `0b99bc8` (eviction logic), `9ba8ded` (test + CMake + comment/plan sync)
**Build/test**: clean build; `./ui_ws/test.sh camp` → 105 tests, 0 failures, 2 skipped (the new test included)

### What was built
Deferred LRU eviction that bounds `MapTiles::tiles_`, fixing the unbounded pan/zoom
growth behind the #98 OOM (the OSM/WMTS basemap never refreshes, so `setLayout()` — the
only prior deletion path — was never reached after construction).

- **Cap** = `max(kTileEvictionMinCap, kTileEvictionMultiplier * visible_count)` =
  `max(256, 4 * visible)`. Constants are file-private in `map_tiles.cpp` with the
  ~256 KB/tile → ~64 MB floor rationale documented at the definition site (operator-confirmed).
- **Deferred, never synchronous (Plan Review must-fix).** `paint()` does NOT delete any
  `Tile`. After the existing visibility loop it records `tile_last_visible_gen_[addr] =
  paint_generation_` for each visible tile, bumps `paint_generation_`, and — if
  `tiles_.size() > cap` and no eviction is already pending — sets `eviction_pending_ =
  true` and queues `evictIfNeeded()` via `QMetaObject::invokeMethod(..., Qt::QueuedConnection)`.
  The `eviction_pending_` flag debounces so at most one eviction is queued at a time.
- **`evictIfNeeded()` private slot** (the primary deletion path, runs in the event loop
  where deleting scene children is safe): clears `eviction_pending_`; recomputes the cap
  from the live visible count; collects candidates guarded on `tile->isVisible()` (a tile
  visible *now* is never evicted, even if it was off-screen when scheduled); sorts ascending
  by `tile_last_visible_gen_` (missing → 0 = oldest); deletes oldest-first
  (`delete` + `tiles_.erase` + erase the gen entry) until `tiles_.size() <= cap`. Since
  `cap >= visible_count`, there are always enough non-visible candidates to reach the cap.
- **`setLayout()`** clears `tile_last_visible_gen_` and resets `eviction_pending_`;
  `paint_generation_` stays monotonic (not reset).
- `tile_last_visible_gen_` is keyed by `TileAddress` so it shares `tiles_`' comparator
  (`operator<` ignores the refresh epoch); a header comment records this so the shared
  semantics aren't silently broken.

### Test (`test_map_tiles_eviction.cpp`)
Multi-zoom layout (1×1 front + 20×20 deep) — required because `setLayout()` pre-seeds the
whole front grid at construction, so a single-zoom layout would mint nothing in `paint()`
and not reproduce the bug. `paint()` is driven directly with a hand-built `QPainter` world
transform (`QTransform(8,0,0,-8,tx,ty)`: uniform scale 8 with a web-mercator y-flip), chosen
so all index arithmetic lands on exact integers — deterministic under offscreen QPA, so the
plan's `QGraphicsScene::render()` fallback was unnecessary. A one-tile viewport is panned
across every position of the 20×20 grid with `QApplication::processEvents()` after each
paint (eviction is deferred); the test asserts `tileChildCount()` stays ≤ the 256 cap
throughout and after the sweep. A second test (`DeepLevelTilesAreMintedByPaint`) confirms
the deep tiles really are minted by `paint()`. Registered in `CMakeLists.txt` mirroring the
`test_map_tiles_refresh` block; the latter's SCOPE comment now points here.

### Non-vacuity check (without-fix-fails)
Verified empirically: with `evictIfNeeded()` stubbed to an early `return` (eviction
disabled) and camp rebuilt, the pan drives `tiles_` to **401** (1 front + 400 deep) and the
test fails (`actual: 401 vs 256`). The stub was reverted, camp rebuilt, and both eviction
tests pass. So the test is a genuine regression gate, not a tautology.

### Consequence updates
- `onRefreshTimer` comment in `map_tiles.cpp` updated: it no longer claims paint() "only
  hides (not deletes) tiles"; it now notes the [#98] eviction bounds within-cycle accumulation.
- `test_map_tiles_refresh.cpp` SCOPE comment updated to point at the new eviction test.
- `plan.md` synced to the as-built deferred design, chosen constants, multi-zoom test, and
  resolved Open Questions.

### Notes for the host
- A fresh container had empty `core_ws`/underlay installs. I built camp's deps first
  (`colcon build --packages-up-to marine_ais_msgs marine_interfaces marine_autonomy` in
  `core_ws`, ROS Jazzy supplying the message deps) before `build.sh camp` succeeded.
- Not pushed, per the handoff contract.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-22 05:43 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-98 at `75f0035`
**Mode**: pre-push
**Depth**: Deep (reason: 661 changed lines ≥ 200, plus Qt object-lifecycle / deferred-deletion dimension)
**Must-fix**: 0 | **Suggestions**: 1
**Round**: 1 | **Ship**: recommended — no Must-fix; only an optional eviction-determinism suggestion, diff is shippable

### Findings
- [ ] (suggestion) `evictIfNeeded()` sort has no tie-breaker for same-generation tiles — harmless to the memory bound & visible-tile safety; affects only eviction determinism — `src/camp2/map_tiles/map_tiles.cpp:164`

### Notes
- Static analysis (ament_cpplint + cppcheck): no PR-introduced defects. cpplint output was all whole-file ament-style conventions this Qt codebase deliberately doesn't adopt (copyright headers, header-guard style, namespace indentation, C-style `int(...)` casts, for-colon spacing); new code matches surrounding idiom. cppcheck `slots`-macro errors are Qt parse noise, not bugs.
- Both Claude Adversarial passes concur the deferred-eviction logic and Qt lifecycle are sound: in-flight pixmap for an evicted tile is dropped by `tileLoaded`'s `tiles_.find` guard; a queued `evictIfNeeded()` surviving a `setLayout()` is benign (rebuilds candidates from current state); no repaint busy-loop; visible tiles never evicted (`cap >= visible_count`).
- Plan adherence: matches `plan.md` step-for-step (deferred slot as primary deletion path per Plan Review must-fix; multi-zoom test; consequence comment updates). No scope creep.
- Build/test not re-run this round; relied on the Implementation entry's documented clean build + 105 tests (0 failures) and the empirically-verified non-vacuity check (401 without fix → test fails).
