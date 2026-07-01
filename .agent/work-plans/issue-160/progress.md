---
issue: 160
---

# Issue #160 — Consumer-side overview pyramid + bounded eviction for live sonar coverage

## Issue Review
**Status**: complete
**When**: 2026-07-01 00:50 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #160
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Actions
- [ ] Record eviction policy + pyramid design decisions via camp ADR-0006 addendum (ADR-0012 carve-out) or new camp ADR — the eviction trigger metric (distance-from-vessel + byte budget vs. tile-count), fold timing (fold-on-evict vs. fold-on-arrival), and overview chain depth are open sub-decisions that ADR-0001 requires to be captured before or during implementation.
- [ ] Clarify reconciler interaction on LRU-evict: does the evicted tile stay in the reconciler's `tiles_` map (as disk-backed) or get `drop()`d? This affects whether camp re-requests tiles from the boat after LRU eviction — plan-task should settle this and record it.
- [ ] Plan multi-repo worktree: changes span `camp` (eviction + pyramid + LOD) and `unh_marine_autonomy` (GGGS `parent(GridIndex)` helper); use `--packages camp,unh_marine_autonomy` or sequence a GGGS PR first and a camp PR consuming it — plan-task should decide.
- [ ] Threading: verify eviction + overview fold happen on the GUI thread (ADR-0006 D4 invariant: reconciler and tile map are not thread-safe).

### Orchestrator note (checkpoint 1 resolved by operator, 2026-07-01)
- **GGGS `parent()` helper: ADD IT NOW (cross-repo).** Operator chose the dedicated
  `parent(GridIndex)`/`children()` primitive in `unh_marine_autonomy/gggs` (handling the
  polar 1/3/9 column scaling) over a camp-local geographic round-trip, so the boat can reuse
  it later for producer-side overviews. **plan-task**: plan the multi-repo coordination —
  recommended sequencing is a small standalone GGGS-helper PR (own issue in
  `unh_marine_autonomy`, `Part of rolker/camp#160`) merged + underlay rebuilt first, then the
  camp PR consuming it. Settle the remaining open sub-decisions (eviction trigger metric,
  fold timing, overview chain depth, reconciler-on-evict semantics) with the issue's stated
  leanings (distance-from-vessel + byte budget; full overview chain) and record them in the
  ADR-0006 addendum / new camp ADR.

## Plan Authored
**Status**: complete
**When**: 2026-07-01 13:30 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-160/plan.md` at `d523a1d`
**Branch**: feature/issue-160 at `d523a1d`
**Phases**: 2 (Phase A: unh_marine_autonomy GGGS helpers PR; Phase B: camp eviction + overview PR)

### Open questions
- [ ] Vessel position source: which ROS topic does camp subscribe to for operator vessel position — if not yet wired to SonarLiveCacheLayer, fall back to view-center distance or pure LRU.
- [ ] Overview tile resolution: fixed 64×64 per overview tile (caps memory, simpler) vs. matching the fine tile's width/height?

## Plan Review
**Status**: complete
**When**: 2026-07-01 01:26 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-160/plan.md` at `d523a1d`
**PR**: PR-less (`--issue 160`, layer worktree `issue-camp-160`)
**Verdict**: approve-with-suggestions

Independent review (fresh-context sub-agent, Opus; plan authored by a Sonnet
sub-agent). Not annotated as author self-review: the name-based detection is
degenerate here (all Claude agents share the name "Claude Code Agent"), and the
"in-context — author self-review" tag would be factually false for an
out-of-context, different-model review.

### Findings
- [ ] (suggestion) `parent()` sketch calls `Level(child.level()-1).gridIndex(center_lat, center_lon)`, but `GridIndex` has no `center_lat`/`center_lon` accessors — compute the center from `northLatitude()/southLatitude()/eastLongitude()/westLongitude()` (verified in `gggs/grid_index.h:87-102`) — `plan.md:43`
- [ ] (suggestion) Phase A "separate PR" doesn't name a separate `unh_marine_autonomy` issue; the issue-review orchestrator note calls for a standalone GGGS-helper issue tagged `Part of rolker/camp#160` — state it explicitly — `plan.md:40`, `plan.md:227`
- [ ] (suggestion) `scheduleWriteThrough(const SonarLiveTile&)` currently writes to `cache_dir_` root (`sonar_live_cache_layer.h:151`); folding overviews into an `overviews/` sub-dir needs a destination/subdir parameter — spell out that small API change — `plan.md:129`
- [ ] (suggestion) Vessel-position source (open question) materially drives the eviction ordering in `evictIfOverBudget()`; resolve it or commit to the LRU/view-center fallback before implementing step 8, rather than mid-implementation — `plan.md:216`
- [ ] (suggestion) `children()` is added to GGGS but not consumed by camp ("Only what's needed" tension); operator-sanctioned per the checkpoint note and unit-tested, so acceptable — no change required, noted for the record — `plan.md:190`

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-01 03:34 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-160 at `479071a`
**Mode**: pre-push
**Depth**: Deep (reason: new ADR + ~700 LOC memory-lifecycle/concurrency-critical caching)
**Must-fix**: 1 | **Suggestions**: 9
**Round**: 1 | **Ship**: continue — one genuine design/honesty concern (overview footprint is unbudgeted, contradicting the ADR's boundedness claim); address before pushing as "closes #153"

Static analysis (ament_cpplint): no new actionable findings — only camp house-style
(Allman `else`, `CAMP_ROS_` guards, no copyright headers, C++-first includes, C-style
numeric casts) applied consistently with surrounding code, plus one trivial new
`kOverviewSubdir` `std::string`-vs-`char[]` nit. camp does not enforce stock ament_cpplint.
Cross-repo dep satisfied: GGGS `parent()`/`children()` (unh_marine_autonomy #249/#250)
merged into the underlay `jazzy` and present. Adversarial: 2 disjoint-lens Claude passes.
Concurrency / GL-texture lifetime / shutdown-join ordering all verified sound.

### Findings
- [ ] (must-fix) Overview pyramid is excluded from the budget and never evicted, growing ≈O(survey area) at fine-tile byte size (recurses to level 0; `accountedBytes()` sums only `tiles_`) — contradicts ADR-0010's "no longer grow memory/VRAM without bound" / "closes #153"; cross-confirmed by both adversarial lenses + governance. Correct the ADR-0010 Consequences + ADR-0006 addendum and cap/evict overviews or open a tracked follow-up — `docs/decisions/0010-bounded-eviction-overview-pyramid.md:116`, `sonar_live_cache_layer.h:225`
- [ ] (suggestion) `handleCatalog` prune erases `tiles_`/disk/reconciler but never `overview_tiles_` (or `overviews/` on disk) — retracted regions leave stale/orphaned overview coverage + slow leak (compounds the must-fix) — `sonar_live_cache_layer.cpp:413`
- [ ] (suggestion) `accountedBytes()` recomputed per eviction-loop iteration + in the sort; bounded by trim-every-64 (so ~linear in N, not O(N²)), but a running incremental byte total removes the rescans — `sonar_live_cache_layer.cpp:562`
- [ ] (suggestion) `evictIfOverBudget()` after warm-load re-writes just-read, byte-identical fine tiles back to disk; guard persist-then-drop with a dirty flag to avoid the write-back storm on enabling a large cache — `sonar_live_cache_layer.cpp:332`
- [ ] (suggestion) `foldChild` assumes the parent geographically contains the child; across GGGS latitude-band boundaries (±72°/±80°) bounds guards prevent corruption but child cells whose centre falls outside the parent are silently dropped → possible overview gaps on high-latitude surveys. Add a boundary test / note it — `sonar_live_tile.cpp:141`
- [ ] (suggestion) Warm-load eviction has no viewport and all tiles share `last_access_seq=0`, so survivors are filesystem order, not most-relevant (acceptable — no history yet) — `sonar_live_cache_layer.cpp:297`
- [ ] (suggestion) Missing reconciler-isolation test (plan step 13): no assertion that `overview_tiles_` keys never enter the reconciler's held set (D4 constraint, currently design-only) — `test/test_sonar_live_eviction.cpp`
- [ ] (suggestion) Overview level-parse guard is `lvl < 256` rather than the real gggs level count (~21); `valid()` catches it downstream but tighten the bound — `sonar_live_cache_layer.cpp:313`
- [ ] (suggestion) Plan Consequences table still says eviction calls `reconciler_.drop()`, contradicting the as-built `markHave`-keep note — reconcile — `plan.md:228`
- [ ] (suggestion) ADR-0006 addendum's four descriptive lines edge past ADR-0012's navigational scope (borderline; passes the "misleading?" test) — trim to pointer + one-liner — `docs/decisions/0006-live-tile-cache-persistence.md:161`
- [ ] (noted, not a finding) Evicted fine tiles don't reload on pan-back within a session — captured in ADR-0010 D2 as a deferred follow-up; accepted limitation.
