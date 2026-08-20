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
- [ ] (suggestion) Per-kick reload volume is unbounded vs budget: `kickReload` snapshots every visible evicted index and `onReloadFinished` inserts all before `evictIfOverBudget()`; a wide zoom-out reloads the whole survey in one kick (transient over-budget spike). Cap to ~0.25×budget headroom, nearest-first. — `sonar_live_cache_layer.cpp:824`
- [ ] (suggestion) Cross-pan reload↔evict thrash: D6 hysteresis blocks same-frame ping-pong, not cross-pan churn; bounding per-kick volume mitigates; note in ADR-0010 if intended. — `sonar_live_cache_layer.cpp:855`
- [ ] (suggestion) EvictionHeadroomAtSurveyScale: `overviewResidentBytes() < budget` is near-tautological; assert `overviewTileCount()` against the geometric bound to actually prove the 4→1 collapse. — `test/test_sonar_live_eviction.cpp:611`
- [ ] (suggestion) ReloadHysteresisPreventsPingPong never asserts `accountedBytes() <= budget` after a permitted reload; both reload tests drive only synchronous `waitForReload()`, never the production `paint()`→queued-`finished` path (finding 1). — `test/test_sonar_live_reload.cpp`
- [ ] (suggestion) Reloaded tile reinserted without `reconciler_.markHave()`; harmless today but relies on a remote invariant — add a defensive markHave. — `sonar_live_cache_layer.cpp:864`
- [ ] (suggestion) Document the deliberate threading-model divergence from `GggsTileLayer` (return-by-value + `result()`, no abort) — a large kick can't be cancelled by a subsequent pan. — `sonar_live_cache_layer.cpp:64`
