---
issue: 171
---

# Issue #171 — Live-coverage display collapses to low resolution during eviction (folding frees almost no memory)

## Issue Review
**Status**: complete
**When**: 2026-08-20 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #171 (and companion #172, reviewed together per operator directive)
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Summary

Issues #171 and #172 are the two halves of world-store LOD step 4: the live
coverage cache adopts the shared fold/pyramid machinery. Steps 1–3 already landed
(camp PR#173, uma PR#287, camp PR#183 / ADR-0013). The eventual PR closes both.

**#171 root cause** (verified in source at `foldIntoParent()` line 589):
`SonarLiveTile(parent_index, fine.width(), fine.height())` allocates each
overview tile at the same pixel count as the fine tile (960×960, ~11 MB). The
full chain to level 0 creates ~8 overview tiles per evicted fine tile. With 45
accumulated overview tiles at ~11 MB each (~500 MB) against a 512 MiB budget,
phase-1 eviction gains no headroom — each fine-tile eviction generates a same-size
overview — and runs until all 38 fine tiles are gone.

**#172 root cause** (ADR-0010 D2 / ADR-0013 §"The camp#172 hook"):
Evicted fine tiles remain in reconciler possession (`markHave`) so re-request is
never triggered, and `warmLoad()` is the only disk-reload path (startup only).
ADR-0013 deliberately left the `hasUnloadedVisibleTiles` + snapshot-filtered
worker hook enabled-but-unimplemented.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Capture decisions, not just implementations | Action needed | Camp ADR-0010 D3 defines same-pixel-size overviews; any fix that decimates overview pixel size (direction 1) or discounts overviews in the budget (direction 2) is a design change — ADR-0010 amendment required in the same PR. Camp ADR-0013 §"camp#172 hook" should be updated from "enabled, not implemented" to "implemented" once the reload hook lands. |
| A change includes its consequences | Action needed | Tests needed: (a) regression that eviction at survey scale (45 overview tiles, 38 fine tiles) leaves budget headroom after the fix; (b) regression that an evicted fine tile is reloaded when the viewport re-enters its area (#172). |
| Human control and transparency | Watch | Budget default (512 MiB) was designed before the overview-accumulation math was understood to be same-pixel-size. After the fix the parameter semantics change; `LiveTileCache/max_vram_bytes` documentation should note the new effective range. |
| Only what's needed | OK | Fix scope is tight: fix #171 in `foldIntoParent()` / `accountedBytes()`, implement the #172 hook. No unrelated rework. |
| Improve incrementally | OK | Staying within the established step-4 shape (uma shared fold engine + ADR-0013 reload seam); no redesign needed. |
| Test what breaks | Watch | The eviction collapse is a field-observed failure at realistic survey scale; a test that seeds realistic tile counts and confirms budget headroom after eviction is the highest-value regression. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| Camp ADR-0010 (bounded eviction / overview pyramid) | Yes — directly | D3 chose match-resolution overview tiles. Fix direction 1 (decimate pixel size) amends D3. Fix direction 2 (discount overviews in budget) amends D1. Either way an amendment is mandatory in the same PR. D2 limitation (no on-demand reload) is resolved by #172; amendment should reflect that too. |
| Camp ADR-0013 (LOD level selection / demand-driven load) | Yes — #172 hook | §"The camp#172 hook" is explicitly "enabled, not implemented"; implementing it updates the status of that clause. ADR-0013 should get a note once implemented. |
| Uma ADR-0011 (overview pyramids — sidecar layout / fold policy) | Yes — step-4 adoption | The shared `overview_builder.hpp` is the step-4 fold engine; camp's `foldIntoParent()` should converge to it. Uma ADR-0011 defines the sidecar layout (flat `overviews/<level>_<row>_<col>.tif`) that camp already follows — but the fold policy (MEAN for imagery) and pixel-size semantics (same pixel count per tile at every level) should be confirmed aligned or noted as a divergence. |
| Uma ADR-0010 (geospatial world model) | Watch | Referenced as context for D1 budget accounting; no direct change needed, but the nightly-regen anti-clobber / catalog prune propagation mentioned in the operator's locked decisions should be confirmed in scope. |
| Camp ADR-0006 (live tile cache persistence) | Watch | ADR-0010 extended ADR-0006; eviction and reload semantics built on ADR-0006 guarantees. No amendment needed if the fix doesn't touch the persistence contract. |
| Workspace ADR-0001 (adopt ADRs) | Yes | Design decision (which fix direction) must be recorded before or alongside implementation. |

### Consequences

Per the consequences map and the code:

- **If `foldIntoParent()` is changed** (pixel-size decimation): rendering in `items()` emits overviews first (coarse→fine) — smaller overview tiles still paint correctly because the renderer scales to scene coordinates, not pixel-for-pixel. `recomputeBounds()` and `foldAutoRange()` union both maps — no change needed there. Write-through path (`scheduleWriteThrough`) writes to `overviews/` — file sizes will change; on-disk warm-load (startup) must handle the new sizes. Tests that assert overview tile dimensions must be updated.
- **If budget accounting changes**: `accountedBytes()` changes — tests that assert budget tracking behavior need updating.
- **If the #172 reload hook is implemented**: `evictIfOverBudget()` and the reload hook must not ping-pong; a hysteresis margin (e.g., reload only when under some fraction of budget) or a cool-down must be explicit. ADR-0010 D4 (GUI-thread-only mutation) constrains how the reload can be dispatched — same constraint as the existing demand-driven worker in `GggsTileLayer` (ADR-0013 §"demand-driven loading").
- **Nightly-regen anti-clobber → catalog prune**: the operator's locked decisions include propagating nightly-regen anti-clobber to the operator cache via catalog prune; confirm whether this is in scope for the step-4 PR or a follow-up.

### Recommendations

- Fix direction 1 (decimate overview pixel size) is more faithful to the standard pyramid model (uma ADR-0011's per-tile-same-pixel-count but each level covers 4× more area means 1/4 the spatial resolution per pixel — still same pixel count). The memory math: a chain of 8 levels = 11 × (1 + 1/4 + 1/16 + …) ≈ 14.7 MB total per new fold chain vs. 88 MB with same-size tiles. This is the natural alignment with the shared fold engine.
- The reload-hysteresis requirement for #172 (don't ping-pong with eviction) should be codified in the ADR-0010 amendment, not just in a code comment.
- The budget default (512 MiB) may be appropriate to revisit in the amendment — document what fraction overview tiles should occupy at the chosen pixel-size.

### Actions
- [ ] Amend camp ADR-0010 to record the chosen fix direction (decimate overview pixel size or adjust budget accounting) and update D3 / D2 accordingly.
- [ ] Update camp ADR-0013 §"camp#172 hook" once the reload path is implemented.
- [ ] Add regression test: eviction at realistic survey scale (e.g., 38 fine tiles + 45 overview tiles) leaves budget headroom after the fix rather than shedding all fine tiles.
- [ ] Add regression test: an evicted fine tile is reloaded on-demand when the viewport re-enters its area (hysteresis / budget-margin must prevent immediate re-eviction).
- [ ] Confirm nightly-regen anti-clobber / catalog-prune propagation scope (in this PR or a tracked follow-up).

## Plan Authored
**Status**: complete
**When**: 2026-08-20 17:32 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-171/plan.md` at `447eec3`
**Branch**: feature/issue-171 at `447eec3`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-08-20 17:40 +00:00
**By**: Claude Code Agent (Claude Opus)

<!-- Independent review: the ## Plan Authored entry was written by Claude Code Agent
     (Claude Sonnet); this review is a separately-dispatched fresh-context sub-agent on
     a different model (Claude Opus). The name-based self-review heuristic collides
     because all agents share the "Claude Code Agent" name, so the annotation is
     deliberately omitted — this is a genuinely independent review. -->

**Plan**: `.agent/work-plans/issue-171/plan.md` at `447eec3`
**PR**: PR-less (local branch `feature/issue-171`; `gh` auth unavailable in this
gitcloud worktree, so a draft PR could not be confirmed)
**Verdict**: changes-requested

**Note on issue fetch**: `gh` is unauthenticated here, so issue #171/#172 bodies
could not be re-fetched. The review relies on the `## Issue Review` entry above
(which captured them) plus source verification against the code.

### Findings
- [ ] (must-fix) Plan mischaracterizes decimation as "adoption of / equivalent to the uma shared fold engine." Verified `buildParentTile` (`marine_tiled_raster_store/.../overview_builder.hpp:182`) builds the parent at the **fixed uniform** `TiledRasterTile::edge` size — the standard half-resolution-per-level pyramid. The plan's `⌊W/2⌋` decimation gives **quarter**-resolution per level (a parent cell spans ~4×4 child cells), a deliberate camp-specific **divergence**. Only the MEAN cell-fold policy matches uma. ADR-0010 amendment, the `foldIntoParent()` code comment, and the ADR-Compliance/Uma-ADR-0011 row must record it as a divergence-with-rationale, not convergence. — `plan.md:62-64,71,168`
- [ ] (must-fix) State the resolution trade explicitly: each pyramid level drops linear resolution by 4× (not 2×). The 1.33× memory series is correct for bytes, but the coarser mid-zoom fidelity vs a standard pyramid is unstated in the plan and the ADR-0010 amendment. — `plan.md:42,64`
- [ ] (must-fix) `handleCatalog()` prune (`sonar_live_cache_layer.cpp:460-476`) `fs::remove`s the fine-tile GeoTIFF and drops reconciler possession, but the plan's Files-to-Change table does not wire it to erase the index from `evicted_fine_indices_`. Step 3a promises this in prose only. Without it, a pruned-then-deleted index lingers in the set → `kickReload` snapshots it → `loadFromGeoTiff` fails → index never cleared. — `plan.md:76,149`
- [ ] (must-fix) The `paint()` reload trigger (Step 3c) omits the moved-since-last-kick guard the sibling loader uses (`gggs_tile_layer.cpp:428-429,479-483`: `load_viewport_ != last_kick_viewport_`). The plan declares `reload_kicked_viewport_` but never consults it in the trigger condition, and `onReloadFinished()` clears "loaded" indices only. A permanently-unloadable evicted index re-kicks `kickReload` every frame. Fix: gate on viewport-changed-since-last-kick AND clear attempted-but-failed indices in `onReloadFinished()`. — `plan.md:99-107,92-94`
- [ ] (must-fix) New test `test/test_sonar_live_reload.cpp` is not registered in `CMakeLists.txt` (tests are wired via `ament_add_gtest`, e.g. `CMakeLists.txt:925`). An unregistered gtest never runs. Add `CMakeLists.txt` to Files-to-Change. — `plan.md:151`
- [ ] (suggestion) Naming: ADR-0013 §"camp#172 hook" and the sibling `GggsTileLayer` both name the seam `hasUnloadedVisibleTiles()`. The plan introduces `hasEvictedVisibleTiles()`. Reuse the ADR-named seam or note the divergence in Step 4. — `plan.md:80,148`
- [ ] (suggestion) Reference `GggsTileLayer`'s existing demand-driven loader (`gggs_tile_layer.cpp` reload worker + `waitForLoad()` headless seam) as the implementation template, not just `write_watchers_` — it already carries the hysteresis, moved-since-kick guard, and headless-drive seam #172 needs. — `plan.md:83-111`
- [ ] (suggestion) Test seams: Step 5a's "overview footprint < budget/4" needs a per-pool byte seam (only `overviewTileCount()`/`accountedBytes()` exist today); Step 5b reload tests need a `waitForReload()` headless seam mirroring `GggsTileLayer::waitForLoad()`. Declare these in the header change list. — `plan.md:124,131-136`
- [ ] (suggestion) On-disk migration: overviews written by the pre-fix (uniform-size) code warm-load at their old full size and mix with decimated folds until re-evicted. Low impact (display-grade, self-heals on next eviction/regen) but worth a line in ADR-0010 Consequences. — `plan.md:69-70`
- [ ] (suggestion) ADR-0010 amendment should reconcile the existing "Alternatives considered" entry ("rejected fixed-64×64 in favour of matching fine dimensions … uniform tile size") which the decimation decision now reverses — otherwise stale contradictory rationale remains in the ADR. — `plan.md:36-42`

## Implementation
**Status**: complete
**When**: 2026-08-20 18:20 +00:00
**By**: Claude Opus

**Branch**: feature/issue-171
**Commits** (this work unit, atop `123654d`):
- `0e6af1e` plan: revise for uma-convergence geometry + review findings
- `460c726` docs: amend ADR-0010 (convergence + D6 hysteresis) and ADR-0013 (#172 hook implemented)
- `39a6f65` live coverage: implement #172 on-demand reload; document #171 uma convergence
- `5ba1457` test: eviction-headroom + on-demand reload regressions
- `b18c268` plan: record implementation divergences

**Build/tests**: `camp` builds clean (ui_ws build.sh chain: underlay `sound_play` +
core nav/tiled-raster-store built first, all green). Full suite via `colcon test`:
**224 tests, 0 errors, 0 failures, 13 skipped** (skips are offscreen-GL renders). The
two new/extended sonar tests pass (`test_sonar_live_eviction` 2/2,
`test_sonar_live_reload` 2/2).

### What landed

**Plan revision (operator checkpoint applied).** Pyramid geometry = the uma fold engine
AS-IS (fixed uniform `TiledRasterTile::edge` parents, half-res-per-level, MEAN) — the
plan's `⌊W/2⌋` decimation was DROPPED. Key finding: `foldChild()` **requires** a
same-size parent/child (it maps each child into a ¼ sub-window), so a decimated parent
would break it — the uniform edge is the *only* valid choice, and the shipped
`foldIntoParent()` already produces it. Hence **#171 carries no fold-geometry code
change**; it is documented as convergence with the merged store's pyramid. Catalog-prune
propagation is scoped to a tracked follow-up per operator decision 2.

**Governance.** ADR-0010: D3 reframed as convergence (not a camp divergence); the
`⌊W/2⌋` option recorded as rejected; "Alternatives considered" fixed-64×64 entry reconciled
(uniform size *is* the convergence choice — not reversed); D2 no-reload limitation removed;
**D6 reload-hysteresis** added; Consequences state the 1.33× memory series and that **no
on-disk migration** is needed (geometry unchanged). ADR-0013 §"camp#172 hook" marked
implemented, with the hysteresis addition noted.

**#172 reload (the field-collapse remedy).** `evictIfOverBudget()` records evicted fine
indices; `paint()` kicks a snapshot-filtered `QtConcurrent` reload worker under the D6
hysteresis (`< 0.75×` budget) + a moved-since-last-kick guard (reusing the ADR-0013-named
`hasUnloadedVisibleTiles()` seam); `onReloadFinished()` reinserts on the GUI thread and
clears every *attempted* index (loaded or not — a permanently-unloadable index can't
re-kick forever). `handleCatalog()` prune and `handleTile()` re-send both drop the index
from the reload set; destructor + `disableLiveCoverage()` join/cancel the worker. Mirrors
`GggsTileLayer`'s demand-driven loader incl. a `waitForReload()` headless seam.

### Review findings dispositions (all 5 must-fix + 5 suggestions)
- MF1/MF2 (decimation = divergence / resolution trade): resolved by DROPPING `⌊W/2⌋`;
  ADR-0010 D3 now records convergence, no quarter-res trade exists.
- MF3 (handleCatalog prune erases evicted index): done.
- MF4 (moved-since-kick guard + clear attempted-but-failed): done (both).
- MF5 (register new gtest): done via `ament_add_gtest(test_sonar_live_reload …)`.
- S6 (reuse `hasUnloadedVisibleTiles()` name): done.
- S7 (use GggsTileLayer loader as template): done (incl. `waitForReload()`).
- S8 (per-pool byte seam + `waitForReload()`): done (`fineResidentBytes`/
  `overviewResidentBytes`; `accountedBytes()` made public).
- S9 (on-disk migration note): resolved as "no migration" — geometry unchanged (see
  divergence note in plan.md; the original concern assumed the dropped `⌊W/2⌋`).
- S10 (reconcile "Alternatives considered"): done.

### Divergences from plan
Recorded in `plan.md` §"Implementation Divergences": eviction-headroom test uses a
**contiguous 10×10 GGGS block** with budget > pyramid (a sparse scatter or a
one-fine-tile budget cannot show headroom — that is LOD, remedied by reload); tiny-budget
test asserts per-pool seam consistency instead of a ratio; `waitForReload()` mirrors the
full `paint()` gate; added `setResidentBudgetForTest()` + `evictedFineCount()` seams.

### Next step
Ready for `/review-code`. No push performed (host pushes). The catalog-prune /
nightly-regen anti-clobber follow-up issue should be filed by the host at the publish
checkpoint (operator decision 2).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-20 18:32 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-171 at `63c4708`
**Mode**: pre-push
**Depth**: Deep (reason: cross-thread QtConcurrent reload worker + GUI-thread mutation + lifecycle join; substantive ADR-0010/0013 rewrites)
**Must-fix**: 1 | **Suggestions**: 6
**Round**: 1 | **Ship**: continue — one genuine concurrency/correctness must-fix in the GUI paint path

Static analysis: cppcheck clean (lone Qt `slots`-macro note is a config artifact). Claude Adversarial: 2 passes (Lens A + Lens B). Copilot: off (default). Local: skipped (no Ollama server). Finding 1 is a cross-pass catch — Lens A corrected Lens B's `isRunning()`-sufficiency assumption.

### Findings
- [x] (must-fix) Re-kick between worker-finish and the queued `finished` slot: paint/waitForReload/kickReload gate only on `!reload_watcher_.isRunning()`, which flips false before `onReloadFinished()` runs — a moved-viewport paint in that window calls `setFuture(F2)`, so the pending `onReloadFinished()` reads the NEW future's `result()` (can block the GUI thread), drops the first reload, and scrambles bookkeeping. Fix: add `reload_attempted_.empty()` to the kick gates. — `src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp:1157,892,821,849`
- [x] (suggestion) Per-kick reload volume is unbounded vs budget: `kickReload` snapshots every visible evicted index and `onReloadFinished` inserts all before `evictIfOverBudget()`; a wide zoom-out reloads the whole survey in one kick (transient over-budget spike). Cap to ~0.25×budget headroom, nearest-first. — `sonar_live_cache_layer.cpp:824`
- [x] (suggestion) Cross-pan reload↔evict thrash: D6 hysteresis blocks same-frame ping-pong, not cross-pan churn; bounding per-kick volume mitigates; note in ADR-0010 if intended. — `sonar_live_cache_layer.cpp:855`
- [x] (suggestion) EvictionHeadroomAtSurveyScale: `overviewResidentBytes() < budget` is near-tautological; assert `overviewTileCount()` against the geometric bound to actually prove the 4→1 collapse. — `test/test_sonar_live_eviction.cpp:611`
- [x] (suggestion) ReloadHysteresisPreventsPingPong never asserts `accountedBytes() <= budget` after a permitted reload; both reload tests drive only synchronous `waitForReload()`, never the production `paint()`→queued-`finished` path (finding 1). — `test/test_sonar_live_reload.cpp`
- [x] (suggestion) Reloaded tile reinserted without `reconciler_.markHave()`; harmless today but relies on a remote invariant — add a defensive markHave. — `sonar_live_cache_layer.cpp:864`
- [x] (suggestion) Document the deliberate threading-model divergence from `GggsTileLayer` (return-by-value + `result()`, no abort) — a large kick can't be cancelled by a subsequent pan. — `sonar_live_cache_layer.cpp:64`

## Implementation
**Status**: complete
**When**: 2026-08-20 18:48 +00:00
**By**: Claude Opus

**Branch**: feature/issue-171 at `cdcdfb2`
**Addressed**: Local Review (Pre-Push) (When 2026-08-20 18:32 +00:00, branch at `63c4708`) — all 1 must-fix + 6 suggestions
**Commits**: `b1c8ee2`, `5771e8d`, `a06f8ea`, `984e1df`, `cdcdfb2`

### Actions
- [x] (must-fix) Gate reload kicks on `reload_attempted_.empty()` — `sonar_live_cache_layer.cpp` (kickReload guard, `paint()` gate, `waitForReload()` gate) + header `reload_attempted_` doc. Closes the queued-`finished` race: the watcher's `finished` signal is queued, so `isRunning()` flips false one hop before `onReloadFinished()` consumes the result; a paint in that window would `setFuture()` a second future and the pending slot would read the *new* future's `result()`. `reload_attempted_` is non-empty for exactly that window, so gating on it empty is the load-bearing guard (commit `b1c8ee2`).
- [x] (suggestion) Bound per-kick reload volume, nearest-first — `kickReload` now caps the batch to a quarter-budget of fine tiles (using `last_evicted_fine_bytes_`, recorded in `evictIfOverBudget` phase 1) and keeps the ones nearest the viewport centre; a `qInfo` logs when the cap drops tiles (no silent truncation). The remainder reload on a later frame once the viewport moves (commit `5771e8d`).
- [x] (suggestion) Cross-pan reload↔evict thrash — documented in ADR-0010 D6: the hysteresis blocks same-frame ping-pong, not cross-pan churn, which is bounded (per-kick cap + disk-only/no-reconciler reloads) and judged acceptable at the lake/harbour envelope; a dwell-timer scheme was considered and deferred (commit `5771e8d`).
- [x] (suggestion) Document the no-abort threading divergence — comment on `reloadTilesFromCache`: unlike `GggsTileLayer`, this worker returns by value + `result()` with no abort, so a kick always runs to completion; kept cheap by the per-kick cap + the `reload_attempted_` gate (commit `5771e8d`).
- [x] (suggestion) Defensive `reconciler_.markHave()` on reload reinsert — `onReloadFinished` re-asserts possession so residency and possession can't silently diverge if D2's keep-possession invariant is ever weakened; `markHave` is idempotent (commit `a06f8ea`).
- [x] (suggestion) EvictionHeadroomAtSurveyScale — replaced the near-tautological `overviewResidentBytes() < budget` with a geometric-bound assertion: `overviewTileCount()` ≤ the set of every parent the full 10×10 block folds into (walking `gggs::parent()` to level 0), and that set is strictly smaller than the fine-tile count — the actual 4→1 collapse (commit `984e1df`).
- [x] (suggestion) ReloadHysteresisPreventsPingPong / budget-after-reload — added `accountedBytes() <= budget` after the permitted reload, plus a new `BudgetBoundedReloadCapsPerKickVolume` test (contiguous 8×8 block, budget derived from measured post-eviction footprint) that proves one reload brings back a *bounded* batch and leaves the rest evicted while residency stays within budget (commit `cdcdfb2`). **Partial:** the tests drive the deterministic `waitForReload()` seam (paint()'s headless analogue, same gate); the live `paint()`→queued-`finished` path is not exercised because it needs a real `QGraphicsView`/`QPainter` transform — impractical headless. The queued-`finished` race it would exercise is closed structurally by the must-fix `reload_attempted_` gate. Noted in the test comment; a paint-path integration test is left for a GUI-harnessed suite if one is added.

### Build / test note
The lower dependency layers (underlay/core/platforms/sensors/simulation) are **not built** in this
sub-agent's environment (no `install/` trees; `marine_ais_msgs` and the rest resolve nowhere), so
`camp` could not be compiled or the gtests run here — a full-chain rebuild is outside address-findings'
"quick local checks" scope. Changes were verified by static inspection and `cppcheck`
(`--enable=warning,performance`, clean on the diff aside from the two pre-existing intentional
by-value QtConcurrent-worker params and the Qt `slots`-macro config artifact the pre-push review already
dismissed). **The re-review must build camp and run `test_sonar_live_eviction` + `test_sonar_live_reload`
(now 3 reload cases) to confirm green** — treat the compile/test gate as unverified locally.

### Next step
Lifecycle: **Implementation** → **review-code** (re-review the fixes). Dispatch a fresh-context
sub-agent: `.agent/scripts/dispatch_subagent.sh --mode in-process --issue 171 --skill review-code`.
No push performed (host pushes).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-20 19:00 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-171 at `d5cb0b1`
**Mode**: pre-push
**Depth**: Deep (reason: cross-thread QtConcurrent reload worker + GUI-thread-only mutation + lifecycle joins; substantive ADR-0010/0013 rewrites)
**Must-fix**: 0 | **Suggestions**: 6
**Round**: 2 | **Ship**: recommended — 0 must-fix; Round 1's 1 must-fix + 6 suggestions all verified addressed. Remaining items are non-blocking robustness/doc suggestions.

Static analysis: cppcheck clean on the diff (only the two intentional by-value QtConcurrent worker params + the Qt `slots`-macro config artifact, both dismissed in Round 1). Claude Adversarial: 2 fresh-context passes (Lens A logic + Lens B concurrency). Copilot: off (default). Local: skipped (no Ollama server). **Build/test gate UNVERIFIED locally** — `core_ws/install` is empty in this worktree, so camp could not be compiled and `test_sonar_live_eviction` / `test_sonar_live_reload` could not be run (same limitation as Round 1); CI must confirm green before merge.

Round-1 verification: the load-bearing `reload_attempted_.empty()` gate (must-fix) correctly closes the queued-`finished` re-kick race (confirmed independently by Lens B); per-kick cap, D6 doc, no-abort doc, defensive markHave, geometric-bound eviction assertion, and budget-after-reload/cap tests all present and correct. Destructor ordering, worker self-containment (capture-by-value), GUI-thread-only invariant, and the double-invoke guard all hold under adversarial reading.

### Findings
- [ ] (suggestion) ADR-0010 D3 "identical fidelity to the merged store's pyramid" overstates: uma folds depth shallowest-preserving (D9), camp live folds by mean; defensible since the visualization tile is imagery-class (scalar, no value/uncertainty pair), but scope the claim to the imagery/mean path and note it is NOT the nav-grade shoal-preserving depth pyramid — `docs/decisions/0010-bounded-eviction-overview-pyramid.md` (D3)
- [ ] (suggestion) `disableLiveCoverage()` clears `reload_attempted_` out-of-band without consuming the queued `finished`; a stale `finished` after re-enable + a new kick could `result()` on the new in-flight future and block the GUI thread. Prefer `onReloadFinished()` / watcher reset over a bare clear — `src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp:317`
- [ ] (suggestion) Reload can resurrect a catalog-pruned tile: an in-flight reload of X that already read disk, interleaved with a `handleCatalog` prune of X, re-inserts + re-`markHave`s X (self-corrects next reconcile). Skip loaded indices `reconciler_.versionOf()` no longer holds — `src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp:917`
- [ ] (suggestion) Hysteresis headroom gate (`accountedBytes() < 0.75*budget`) is duplicated in `paint()` + `waitForReload()` but absent from `kickReload()` (which enforces only the per-batch cap); a future caller forgetting it reintroduces churn — consider moving it into `kickReload` — `src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp:827`
- [ ] (suggestion) `accountedBytes()` can transiently rise within a phase-1 eviction pass when `foldIntoParent` allocates a fresh ancestor chain; loop still terminates + phase 2 converges, but a burst-fold can momentarily peak above budget — worth a one-line comment on the transient overshoot — `src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp:750`
- [ ] (suggestion) `foldChild`'s same-size "1/4 sub-window" invariant (now called a hard invariant in D3) breaks at ±72/±80 GGGS latitude-band boundaries (parent lon-span != 2x child) → possible high-lat overview seam; already a tracked ADR follow-up, both tests anchor at 43N so it is unexercised. A high-latitude boundary test would close it — `src/camp_map/ros/live_coverage/sonar_live_tile.cpp:129`

### Next step
Lifecycle: **Local Review** → push / open PR → **triage-reviews**. Verdict is **approved** (0 must-fix); the 6 suggestions are non-blocking. Optionally apply the two cheap robustness suggestions (disable-clear, kickReload gate) before push, then hand off to a fresh-context sub-agent for triage-reviews once pushed. No push performed (host pushes).

## Integrated Review
**Status**: complete
**When**: 2026-08-20 15:32 -04:00
**By**: Claude Code Agent (Claude Fable 5)

**PR**: #190 at `9c69f69`
**Sources**: 3 (Copilot R1 @ `9c69f69`, Local Review (Pre-Push) R1–R2 @ prior SHAs, CI rollup)
**Cross-source confirmations**: 1
**CI**: all-pass (hosted build-and-test green at head)

### Findings
- [x] (cross-confirmed: Local R2 deferred-suggestion + Copilot @ head) `disableLiveCoverage()` joins the worker (`waitForFinished()`) but clears `reload_attempted_` out-of-band while the `QFutureWatcher::finished` delivery stays queued; after re-enable + a new kick sets a new future, the stale slot's `result()` binds to the NEW in-flight future — GUI-thread block + lost old batch. Fix per the in-file pattern: synchronously consume the completed reload in `disableLiveCoverage()` (as `waitForReload()` does) so the queued `finished` no-ops on the double-invoke guard — `src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp:309-324`. Code-verified this triage: the deferral rationale ("harmless") does not survive the re-enable interleave; two independent sources now agree.

### False positives
- (none)

**Local-timeline reconciliation**: R2's other five deferred suggestions were
not raised by Copilot and stay deferred with their recorded rationale
(self-correcting prune-resurrect window → hardened in camp#191; hysteresis-gate
duplication; transient burst-fold peak; high-lat band seam — pre-existing store
behavior; ADR fidelity wording). The disable/re-enable edge graduates from
deferred to must-fix on cross-confirmation.

## Implementation
**Status**: complete
**When**: 2026-08-20 19:36 +00:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-171 at `05ff4ff` (progress commit follows this entry)
**Addressed**: Integrated Review of 2026-08-20 15:32 -04:00 (PR #190 at `9c69f69`) — its single cross-confirmed must-fix (Local R2 deferred-suggestion + Copilot @ head). No other open actions (the five other R2 suggestions stay deferred per that entry's reconciliation; not this skill's to reopen).
**Commits**: `05ff4ff` (fix + box-check)

### Actions
- [x] (cross-confirmed) `disableLiveCoverage()` now synchronously consumes the completed reload via `onReloadFinished()` after `reload_watcher_.waitForFinished()`, replacing the bare `reload_attempted_.clear()`. `onReloadFinished()` clears `reload_attempted_`, so the still-queued `QFutureWatcher::finished` delivery no-ops on the `reload_attempted_.empty()` double-invoke guard — a re-enable + new kick can no longer bind the stale slot's `result()` to a NEW in-flight future (which blocked the GUI thread and dropped the old batch). Mirrors the in-file `waitForReload()` pattern; safe unconditionally since `onReloadFinished()` early-returns when no reload was in flight. `evicted_fine_indices_.clear()` is retained (drops the remaining evicted bookkeeping `onReloadFinished()` doesn't touch). — `src/camp_map/ros/live_coverage/sonar_live_cache_layer.cpp:309-333` (`05ff4ff`)

### Build / test note
`cppcheck --enable=warning,performance` clean on the changed file (the sole diagnostic is a pre-existing `unknownMacro` on Qt's `slots` in an unrelated header, suppressed). Lower dependency layers are unbuilt in this worktree, so camp could not be compiled and `test_sonar_live_eviction` / `test_sonar_live_reload` could not be run — same limitation recorded in Rounds 1–2. **The re-review must build camp and run both gtests to confirm green** — the compile/test gate is unverified locally. No new test was added: the disable/re-enable → queued-`finished` → GUI-thread `result()` interleave requires a live QEventLoop delivering the queued `finished` slot after a real `paint()`-driven kick, which is not headlessly reproducible (same non-headless constraint noted for the finding-1 gate in prior rounds); the fix is covered structurally by the double-invoke guard it now leans on.

### Next step
Lifecycle: **Implementation** → **review-code** (re-review the fix). Dispatch a fresh-context sub-agent:
`.agent/scripts/dispatch_subagent.sh --mode in-process --issue 171 --skill review-code`.
No push performed (host pushes).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-20 19:44 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-171 at `27954ed` (fix commit `05ff4ff`)
**Mode**: pre-push
**Depth**: Deep (reason: cross-thread QtConcurrent reload worker + GUI-thread-only mutation + lifecycle join in the changed function)
**Must-fix**: 0 | **Suggestions**: 0 new
**Round**: 3 | **Ship**: recommended — single incremental fix (`05ff4ff`) verified; 0 must-fix; Integrated Review's cross-confirmed must-fix closed.

Round-3 focused re-review of the ONLY new code since the Round-2 approval (`d5cb0b1`) + Integrated Review (PR #190 @ `9c69f69`): the 10-line `disableLiveCoverage()` fix (`05ff4ff`). The twice-reviewed remainder is not re-litigated. Static analysis: cppcheck clean on the diff (only the pre-existing Qt `slots` `unknownMacro` artifact, dismissed Rounds 1–2). Claude Adversarial: 2 fresh-context passes (Lens A logic + Lens B concurrency), scoped to the fix. Copilot: off (default). Local: skipped (no Ollama server).

### Outcome
The fix replaced a bare `reload_attempted_.clear()` in `disableLiveCoverage()` with `onReloadFinished()` right after `reload_watcher_.waitForFinished()`. Verified correct end-to-end and cross-confirmed by an independent concurrency pass:
- **Race closed**: after the worker is joined, the synchronous `onReloadFinished()` consumes `result()` (no GUI block) and clears `reload_attempted_`; the still-queued `finished` slot drains as a no-op via the `reload_attempted_.empty()` guard before any re-enable-driven re-kick (posted-event ordering). No interleaving binds the stale slot to a new in-flight future.
- **Consistent with the destructor** (which relies on `shutting_down_` early-return) and **mirrors the reviewed `waitForReload()` pattern**.

### Adversarial adjudication (Lens A false positives — recorded)
- FP1 — "add `if(!enabled_) return;` to `onReloadFinished()`": actively wrong. `disableLiveCoverage()` sets `enabled_=false` BEFORE calling `onReloadFinished()`; an `enabled_` early-return would stop `reload_attempted_` from being cleared and REOPEN the exact race. `onReloadFinished()` guards only on `shutting_down_` and `reload_attempted_.empty()` (`sonar_live_cache_layer.cpp:908,914`).
- FP2 — "clearing `evicted_fine_indices_` leaks evicted tiles": `evicted_fine_indices_` is transient GUI-thread reload bookkeeping; on-disk GeoTIFFs are the source of truth. `enableLiveCoverage()`→`warmLoad()` (`:291,:334`) rebuilds resident state + replays the catalog on every re-enable, so clearing on disable is intended (pre-existing behavior).
- Benign behavior change (not a finding): a reload batch completing during disable is now folded into `tiles_` + budget-maintained (was discarded pre-fix) — consistent with "disabled layer keeps its in-memory cache", bounded by the per-kick cap. Double `updateDisplay()`/`update()` coalesces to one repaint — harmless.

### Deferred (unchanged, not reopened this round)
The 6 Round-2 suggestions + prune-resurrect window stay deferred/tracked per the Integrated Review reconciliation (prune-resurrect hardened in camp#191; hysteresis-gate duplication; transient burst-fold peak; high-lat band seam; ADR-0010 D3 fidelity wording).

### Caveat — build/test gate UNVERIFIED locally
Dependency layers (underlay/core/platforms/sensors/simulation) have empty `install/` trees in this worktree, so camp could not be compiled and `test_sonar_live_eviction` / `test_sonar_live_reload` could not be run (same limitation as Rounds 1–2). `05ff4ff` landed AFTER the implementation sub-agent's clean 224-test build (18:20) — never compiled here. No new symbols (calls existing `onReloadFinished()`), so compile risk is minimal. **CI must confirm both gtests green before merge.**

### Findings
- [ ] No must-fix. Verdict approved. CI must confirm `test_sonar_live_eviction` + `test_sonar_live_reload` green (standing build/test gate, all rounds).

### Next step
Lifecycle: **Local Review (approved)** → push / open PR → **triage-reviews**. 0 must-fix, so no address-findings round is required. Once pushed, dispatch a fresh-context sub-agent: `.agent/scripts/dispatch_subagent.sh --mode in-process --issue 171 --skill triage-reviews`. No push performed (host pushes).
