# Plan: Bundle — camp#168 / camp#169 / camp#170 (2026-07-23 field incident)

## Issues

- https://github.com/rolker/camp/issues/168 — SonarLiveCacheManager never clears spawned-sources map on layer removal
- https://github.com/rolker/camp/issues/169 — Live coverage requests never resume after restart/re-enable
- https://github.com/rolker/camp/issues/170 — No cap on incoming coverage-tile dimensions (crash path)

**One PR, three atomic commits (user decision 2026-08-05).**

---

## Issue #168 — sources_ not cleared on layer destruction

### Context

`SonarLiveCacheManager::updateTopics()` guards against duplicate-spawn using
`std::map<std::string, bool> sources_`. After the operator removes a
`SonarLiveCacheLayer` (`deleteLater()` via `Layer::removeFromMap()`), `sources_`
is never cleared — the entry stays `true`, `updateTopics()` skips the base
permanently, and the layer cannot be re-added within the session.

### Approach (#168 commit)

1. **Switch `sources_` from `map<string,bool>` to `set<string>`** — the `bool`
   value was always `true` and vestigial; `set` expresses the invariant directly.
   Add `#include <set>` to the header, remove `#include <map>`.

2. **Fix the guard** — `if(sources_[base])` → `if(sources_.count(base))`.
   `operator[]` inserts a default-`false` entry for every scanned topic (side
   effect); `count()` is read-only. Replace `sources_[base] = true` →
   `sources_.insert(base)`.

3. **Wire the `destroyed` signal at spawn time** — immediately after
   `sources_.insert(base)`, add:
   ```cpp
   // Between deleteLater() and ~QObject the entry stays; the event loop drains
   // before an operator's re-add, so no spurious re-spawn in that window.
   connect(layer, &QObject::destroyed, this, [this, base]() { sources_.erase(base); },
           Qt::DirectConnection);
   ```
   `Qt::DirectConnection` is safe: the lambda touches only `std::set::erase`,
   which is fast and runs on the GUI thread (both `deleteLater()` and
   `QObject::destroyed` dequeue there).

4. **Update the header comment** (sonar_live_cache_manager.h:36-38) to note that
   removal now clears the entry.

5. **Regression test — white-box on the real manager** (amended per Plan Review
   round 2, adopted at the plan checkpoint): factor the tracking contract out of
   `updateTopics()` into a protected `trackSpawnedLayer(const std::string& base,
   QObject* layer)` (inserts into `sources_` + connects `destroyed` → erase), and
   add a public const accessor `isSourceTracked(const std::string& base)`. The
   test (`test_sonar_live_cache_manager_respawn.cpp`) subclasses the manager to
   promote `trackSpawnedLayer` (`using`-declaration), tracks a plain `QObject`
   (the contract is QObject-generic), asserts `isSourceTracked()` flips
   true → (delete layer) → false, and re-tracks to prove respawn. This exercises
   the manager's REAL set and REAL `connect` wiring — deleting the `connect`
   line fails the test — without needing the live `Node` ancestor that
   `updateTopics()` requires. One `ament_add_gtest` block per CMakeLists.txt:925
   pattern (same include-dirs, deps, link-libs as `test_sonar_live_eviction`).

### #168 Files to Change

| File | Change |
|------|--------|
| `src/camp_map/ros/live_coverage/sonar_live_cache_manager.h` | `map<string,bool>` → `set<string>`; update comment; swap includes |
| `src/camp_map/ros/live_coverage/sonar_live_cache_manager.cpp` | fix guard (`count`), fix insert, add `connect(destroyed → erase)` |
| `test/test_sonar_live_cache_manager_respawn.cpp` | new test (headless, null Node) |
| `CMakeLists.txt` | new `ament_add_gtest` block after `test_sonar_live_eviction` |

---

## Issue #169 — catalog latched while disabled; no reconcile on enable

### Context

`publishRequest()` has exactly one call site: `handleCatalog()`, which
early-returns when `!enabled_` (sonar_live_cache_layer.cpp:391-395) without
buffering the message. The catalog subscription is transient-local depth-1
(created in the constructor, never torn down). If the single latched catalog
arrives before `QTimer::singleShot(0)` fires the settings-restore enable,
`handleCatalog` discards it and no subsequent catalog arrives — requests never
fire. The same gap applies to disable→re-enable within a session
(`unsubscribeTiles()` tears down `request_pub_`; re-enable never reconciles).

### Approach (#169 commit)

1. **Add `last_catalog_`** — add `std::optional<marine_interfaces::msg::TileCatalog>
   last_catalog_;` to `SonarLiveCacheLayer`'s private members (sonar_live_cache_layer.h).
   Include `<optional>` (already present) and `marine_interfaces/msg/tile_catalog.hpp`
   (already included).

2. **Buffer in `handleCatalog()`** — before the `if(!enabled_)` early-return,
   always write `last_catalog_ = msg;`. When disabled, return after buffering (not
   before). No other change to the disabled-path logic.

3. **Replay on enable** — in `enableLiveCoverage()`, after `subscribeTiles()` and
   before `writeSettings()`, add:
   ```cpp
   if(last_catalog_)
     handleCatalog(*last_catalog_);   // GUI thread, direct call — no marshal needed
   ```
   At this point `enabled_` is `true`, so `handleCatalog` will proceed to reconcile
   and (if `request_pub_` is ready) publish requests.

4. **Regression test** (`test_sonar_live_catalog_resume.cpp`): write N tiles to
   the layer's cache dir, enable (warm-loads N tiles → `residentTileCount() == N`),
   disable, invoke `handleCatalog` via `QMetaObject::invokeMethod` (it is a
   `private slots:` entry — Qt's meta-object system allows this) with an **empty**
   catalog (prune-all), re-enable, assert `residentTileCount() == 0`. Without the
   fix the prune-all catalog is discarded; with the fix anti-entropy fires on
   enable and prunes all N resident tiles. Uses the same headless harness
   (offscreen QApplication, null Node). One `ament_add_gtest` block.

### #169 Files to Change

| File | Change |
|------|--------|
| `src/camp_map/ros/live_coverage/sonar_live_cache_layer.h` | add `last_catalog_` member |
| `src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp` | buffer in `handleCatalog`, replay in `enableLiveCoverage` |
| `test/test_sonar_live_catalog_resume.cpp` | new headless test |
| `CMakeLists.txt` | new `ament_add_gtest` block |

---

## Issue #170 — no cap on tile dimensions before allocation (crash path)

### Context

`handleTile()` constructs `SonarLiveTile(index, msg.width, msg.height)` before
any bounds check (sonar_live_cache_layer.cpp:358-359). `SonarLiveTile::applyPatch`
then allocates `width * height` floats per band (sonar_live_tile.cpp:105). A
single oversized or corrupt message is an immediate unbounded allocation on the
GUI thread. The ADR-0010 eviction budget only applies to tiles *already resident*.

**GUI-thread load items (deferred with rationale):** `evictIfOverBudget()` on
every patch (O(N tiles × N bands) per call) and `cached_image_` invalidation on
every patch are acknowledged as aggravating factors during sustained load but are
performance concerns, not crash-risks. Addressing them requires rate-limiting and
eviction amortization that are non-trivial and not necessary for the crash fix.
They are deferred to follow-up issues referencing #154/#155/#156.

### Approach (#170 commit)

1. **Add dimension cap to `handleTile()`** — before the `SonarLiveTile` emplace,
   validate `msg.width`, `msg.height`, and `msg.bands.size()` against reasonable
   upper bounds:
   ```cpp
   constexpr int kMaxBandCount = 64;
   if(msg.width <= 0 || msg.height <= 0 ||
      msg.width > kMaxImageEdge || msg.height > kMaxImageEdge ||
      static_cast<int>(msg.bands.size()) > kMaxBandCount)
   {
     qWarning().noquote() << "[live coverage" << ...
                          << "] rejected tile with absurd dimensions "
                          << msg.width << "x" << msg.height
                          << "bands:" << msg.bands.size();
     return;
   }
   ```
   `kMaxImageEdge = 4096` is already a private static constexpr on the class
   (sonar_live_cache_layer.h:221). `kMaxBandCount` is a new local constant in
   the `.cpp`.

   **Amended per Plan Review round 2 (adopted at the plan checkpoint)**: also
   enforce a combined per-message allocation ceiling — per-dimension caps alone
   still admit `4096×4096×64 bands×4 B ≈ 4 GB`. Add
   `kMaxTileBytes = 256 MiB` (a full 4096×4096 float tile is 64 MiB/band, so
   this admits any legitimate ≤4-band full-size tile while bounding the worst
   case at ~6% of the old ceiling) and reject when
   `width × height × bands × sizeof(float) > kMaxTileBytes`, same warning path.

2. **Regression test** (extend `test_sonar_live_cache.cpp` or new
   `test_sonar_live_tile_validation.cpp`): enable a headless layer, invoke
   `handleTile` (private slot, via `QMetaObject::invokeMethod`) with
   `msg.width = 8193` (over the 4096 cap), assert `residentTileCount() == 0`.
   Then invoke with valid dimensions (e.g., 8×8), assert `residentTileCount() == 1`.
   Uses the existing `test_sonar_live_cache` harness. One `ament_add_gtest` block
   if adding a new file, or just new `TEST()` entries if extending the existing one.

### #170 Files to Change

| File | Change |
|------|--------|
| `src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp` | add dimension/band cap in `handleTile()` |
| `test/test_sonar_live_cache.cpp` or new `test_sonar_live_tile_validation.cpp` | add rejection and acceptance test cases |
| `CMakeLists.txt` | add `ament_add_gtest` block if new file |

---

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | Each fix is surgical: 2-6 lines of production code per issue; no new abstraction or new class |
| Improve incrementally | Three atomic commits on one branch; GUI-thread load items deferred with explicit rationale |
| Test what breaks | Each fix has a headless regression test targeting the specific failure mode |
| A change includes its consequences | CMakeLists.txt, header comment, and optional buffered-catalog member all land with the fix |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0001 | Yes | `Qt::DirectConnection` in #168 fix is safe (GUI thread only, fast lambda); `handleCatalog` replay in #169 is a direct call on the GUI thread (post-enable, no marshal needed) |
| ADR-0006 D2/D4 | Yes (#169, #170) | Catalog buffer is CPU-only (no ROS thread touches it); tile dimension cap is at the node boundary, before allocation |
| ADR-0006 D5 | Yes (#168) | Re-spawned layer still starts inactive (opt-in gate preserved) |
| ADR-0010 | Tangential (#170) | Eviction budget applies post-allocation; the cap prevents the unbounded pre-allocation |

## Consequences

| If we change... | Also update... | Included? |
|---|---|---|
| `sources_` type in manager header | Replace `<map>` with `<set>` include | Yes |
| `handleCatalog` adds buffering | `enableLiveCoverage` must replay buffer | Yes |
| Tile dimension cap | Any future code that bypasses `handleTile` | N/A — no such path exists |

## Documentation & Instruction Impact

- **Stale docs**: The class-level comment on `sonar_live_cache_manager.h:36-38`
  documents the dedup behavior without mentioning removal—update it in the #168
  commit. No other docs reference the spawned-source map or catalog latch behavior.
- **Agent-instruction candidates**: The `QObject::destroyed` pattern for
  manager-tracks-children problems is worth a knowledge note if similar managers
  emerge; premature now (single instance). The catalog-buffer-and-replay pattern
  (always buffer transient-local subscriptions in case of opt-in-gate timing races)
  is a candidate for `.agent/knowledge/` — proposed for operator consideration.

## Open Questions

- None — all three test strategies are concrete (destroyed-signal mock set for #168;
  `QMetaObject::invokeMethod` + `residentTileCount()` for #169 prune-all;
  oversized-tile rejection via `residentTileCount()` for #170).

## Estimated Scope

Single PR (`Closes #168 / Closes #169 / Closes #170`), three atomic commits.
Production-code changes: ~10 lines total across the three fixes. Test files: three
new headless test files plus CMakeLists additions.
