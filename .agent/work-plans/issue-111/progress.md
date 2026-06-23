---
issue: 111
---

# Issue #111 — Weather radar overlay shows stale precipitation (does not advance to latest mosaic)

## Issue Review
**Status**: complete
**When**: 2026-06-22 22:35 +00:00
**By**: Claude Code Agent (Claude Opus)

**Issue**: #111
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Summary

The NEXRAD radar overlay (added in #99, ADR-0004) displays stale precipitation: it
does not advance to the latest IEM mosaic across the #99 Phase-2 5-minute auto-refresh.
The issue lists three candidate root causes. Reading the current code (lines below are
post-#98; the issue's line numbers are stale) lets two of them be largely ruled out and
points strongly at the third:

- **Cause #1 — CDN / HTTP-level caching of the static tile URL (MOST LIKELY).** The IEM
  `nexrad-n0q-900913/{z}/{x}/{y}.png` URL carries no timestamp/cache-buster
  (ADR-0004:67-69). `CachedTileLoader::invalidateCache()`
  (`cached_tile_loader.cpp:50-85`) does correctly delete and recreate the per-layer disk
  subdir, so the next `load()` misses the disk cache and issues a real network GET — but
  that GET hits the *same* URL, which a CDN / proxy edge between CAMP and the IEM origin
  can answer from its own cache with an older tile. ADR-0004's freshness rationale
  (`Consequences`, lines 100-105) and the `onRefreshTimer` comment
  (`map_tiles.cpp:260-266`) both reason only about the *disk* cache and the origin alias
  "always serving the latest mosaic"; **neither accounts for HTTP/CDN caching between
  client and origin.** That is the gap. Likely fix: a per-refresh cache-buster
  (`?t=<epoch-minute>`) on the request URL — note this changes the URL contract recorded
  in ADR-0004 and therefore needs an ADR addendum (see ADR Applicability).

- **Cause #2 — global `CachedFileLoader` re-serving by URL/path (LARGELY DISPROVEN BY
  CODE).** `CachedFileLoader::load()` (`cached_file_loader.cpp:55-74`) gates re-use solely
  on on-disk `file_path.exists()`; if the file exists it rewrites the request to a
  `file://` URL, otherwise it does a network GET. There is **no in-memory URL-keyed cache,
  no in-flight dedup, and no `QNetworkDiskCache` installed on the `QNetworkAccessManager`**
  (constructor lines 18-24). So once `invalidateCache()` removes the per-layer subdir, the
  global loader cannot independently re-serve the previous content. This cause can be
  largely set aside — worth one confirming glance during implementation, not a primary
  investigation line.

- **Cause #3 — refresh-timer cadence / visibility pausing (LARGELY DISPROVEN BY CODE).**
  `onRefreshTimer()` (`map_tiles.cpp:248-271`) runs unconditionally once the timer is
  started; `setRefreshInterval()` (`:229-246`) only *stops* the timer for `msec <= 0`, and
  nothing in the visibility toggle path pauses it. So a default-OFF / toggled layer does
  not pause refresh once an interval is set. The one residual sub-question worth a quick
  check is whether `setRefreshInterval(5min)` is actually invoked on the radar layer in the
  deployed registration path (`background_manager.cpp`) — but the *staleness* is observed
  with the layer ON, so cadence is unlikely to be the mechanism.

Net: this is a well-scoped diagnostic bug whose most probable fix is a cache-buster on the
radar request URL. Recommend the implementer confirm cause #1 empirically (compare the
served tile bytes/headers across two refresh cycles, e.g. the `.json` reply-header sidecar
`CachedFileLoader` already writes at `cached_file_loader.cpp:102-112`) before coding, so
the fix targets the real layer.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Restores expected behavior (fresh radar). A cache-buster is a transparent, mechanical URL change with no operator-visible config impact |
| Enforcement over documentation | Action needed | The fix must ship a regression test. CDN behavior can't be unit-tested directly, but if the fix is a cache-buster the test can assert the request URL carries a *distinct* per-refresh query param across two `onRefreshTimer` cycles — that is the enforceable invariant |
| Capture decisions, not just implementations | Action needed | If cause #1 is confirmed and a cache-buster is added, ADR-0004's "always-latest / static URL + disk-invalidation suffices" reasoning is now known to be incomplete. Record the correction as an ADR-0004 cross-reference addendum (permitted by workspace ADR-0012) or a Consequences update — do not leave the corrected rationale only in a commit message |
| A change includes its consequences | Action needed | Same PR must update: (a) ADR-0004 Consequences (lines 100-105); (b) the `onRefreshTimer` freshness comment (`map_tiles.cpp:250-266`) which currently asserts disk invalidation is sufficient; (c) likely `test/test_map_tiles_refresh.cpp` |
| Only what's needed | Watch | Prefer a minimal cache-buster over building a general HTTP cache-control / `Cache-Control: no-cache` request-header layer. If a request header (e.g. no-cache) proves sufficient and avoids changing the URL/disk-path contract, that may be even smaller — weigh in plan-task |
| Improve incrementally | OK | Single subsystem, single mechanism; completable in one reviewable PR |
| Test what breaks | Action needed | The regression target is "a refresh produces a request the CDN cannot satisfy from cache" — assert per-cycle URL/param distinctness (see Enforcement row). Avoid a live-network test in CI |
| Workspace vs. project separation | OK | Entirely within the `camp` project repo (`ui_ws/src/camp/`); no workspace-infra change |
| Safety / operational awareness (project) | Watch | Inferred (no `camp/PRINCIPLES.md`; consistent with the #98 review's project-safety row). Stale radar is arguably worse than blank radar: a plausible-but-old precipitation frame can mislead the operator's live weather picture during a survey. Raises the priority of getting freshness genuinely correct, and argues for a visible-staleness safeguard as possible follow-up |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| camp ADR-0004 — Weather-radar tile provider (IEM NEXRAD N0Q) | **Yes (directly)** | The issue challenges ADR-0004's core freshness premise. The likely fix amends the URL/refresh contract this ADR records, so ADR-0004 needs a cross-reference addendum or Consequences update in the same PR |
| camp ADR-0002 — Web-Mercator scene & layer model | No (tangential) | Radar lives in the pure-Qt `libcamp_map` layer per this ADR, but no scene/layer-model change is implied |
| workspace ADR-0008 — Follow ROS 2 conventions | No | camp is a Qt/C++ app; no ROS package interface / launch / `package.xml` change |
| workspace ADR-0001 — Adopt ADRs | Yes (via 0004) | The corrected freshness reasoning should be captured, not buried in a commit — satisfied by the ADR-0004 addendum above |

### Consequences

- **ADR-0004** Consequences section (the "Refresh yields a genuinely fresh frame" bullet,
  lines 100-105) — update/addend to acknowledge intermediary HTTP/CDN caching and the
  cache-buster mitigation.
- **`src/camp2/map_tiles/map_tiles.cpp:250-266`** — the `onRefreshTimer` comment that
  asserts disk invalidation alone forces a fresh network frame; correct it to reflect the
  CDN-caching reality and the cache-buster.
- **`test/test_map_tiles_refresh.cpp`** — extend (or add a sibling) to assert the
  per-refresh request-distinctness invariant.

### Actions
- [ ] Enforcement: ship a regression test asserting the per-refresh request carries a
  distinct cache-buster (or otherwise can't be CDN-cached), without a live-network test in CI.
- [ ] Capture decisions: add an ADR-0004 cross-reference addendum / Consequences update
  recording that the static URL + disk-invalidation reasoning was incomplete (CDN caching).
- [ ] Consequences: in the same PR update ADR-0004 (lines 100-105), the `onRefreshTimer`
  freshness comment (`map_tiles.cpp:250-266`), and `test/test_map_tiles_refresh.cpp`.
- [ ] Test what breaks: define the regression invariant (per-cycle URL/param distinctness),
  not a live-network assertion.
- [ ] Recommendation (plan-task): empirically confirm cause #1 (compare served tile
  bytes/reply-headers across two refresh cycles via the `.json` sidecar) before coding, and
  weigh a minimal cache-buster vs. a `no-cache` request header for the smallest fix.
- [ ] Recommendation (project safety): consider a visible radar-staleness safeguard
  (e.g. surface frame age / dim on suspected-stale) as possible follow-up, given stale
  radar can mislead more than blank radar.

## Plan Authored
**Status**: complete
**When**: 2026-06-22 23:26 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-111/plan.md` at `5b45a72`
**Branch**: feature/issue-111 at `5b45a72`
**Phases**: single

Root cause confirmed by code reading: CDN/HTTP caching of the static IEM tile
URL (cause #1); causes #2 and #3 disproven. Fix = opt-in per-refresh cache-buster
on `CachedTileLoader` (`?t=<token>`), seeded from wall clock (cross-session
freshness) with a monotonic guard (strict per-cycle distinctness), driven by
`MapTiles::setRefreshInterval`/`onRefreshTimer`. Same PR corrects the
`onRefreshTimer` freshness comment, adds an ADR-0004 Consequences addendum, and
extends `test/test_map_tiles_refresh.cpp` (pure `withCacheBust` + token-bump +
opt-in invariants; no live-network test).

### Open questions
- [ ] Empirical pre-coding confirmation of cause #1: diff served tile bytes /
  reply-headers across two refresh cycles via the `.json` sidecar
  (`cached_file_loader.cpp:102`) before implementing.
- [ ] Verify IEM `tile.py` accepts and ignores an unknown `?t=` query param
  (returns the tile, not 4xx/blank) against the live endpoint.

## Plan Review
**Status**: complete
**When**: 2026-06-22 23:52 +00:00
**By**: Claude Code Agent (Claude Opus)
<!-- Independent: fresh-context sub-agent dispatched by the host for the review-plan
phase. The name-based self-review heuristic (compare $AGENT_NAME to the Plan Authored
By prefix) matches only because every workspace agent shares the name "Claude Code
Agent"; it is a false positive here, so the self-review annotation is intentionally
omitted. -->

**Plan**: `.agent/work-plans/issue-111/plan.md` at `5b45a72`
**PR**: PR-less (`--issue` / file-path mode; no draft PR)
**Verdict**: changes-requested

### Findings
- [ ] (must-fix) Plan cites workspace ADR-0012 as permitting an in-place
  **Consequences** edit to camp ADR-0004, but ADR-0012 lists "Adding or removing
  Consequences that weren't previously recorded" and "Reversing or softening the
  position" as substantive changes that **still require a new/superseding ADR**.
  Correcting the now-false "Refresh yields a genuinely fresh frame" bullet
  (ADR-0004:100-105) and adding the CDN-caching consequence is exactly that. Fix:
  record the corrected reasoning + cache-buster decision in a NEW camp ADR (e.g.
  ADR-0006) and give ADR-0004 only a permitted cross-reference addendum (Status
  note / References pointer) — or get explicit sign-off to edit ADR-0004 in place.
  — `plan.md:64-66`, `plan.md:104`, `plan.md:113`
- [ ] (suggestion) Empirical confirmation of cause #1 remains an open question, yet
  the plan commits to coding the cache-buster before confirming. Causes #2/#3 are
  disproven by code-reading (solid), but recommend either running the `.json`
  sidecar header/byte diff across two cycles, or explicitly framing the cache-buster
  as a low-risk defensive fix that is correct regardless of whether CDN caching is
  the sole mechanism. — `plan.md:119-126`
- [ ] (suggestion) Open question #2 (does IEM `tile.py` accept an unknown `?t=`
  param, or 4xx/blank?) is load-bearing: if the endpoint rejects unknown params the
  fix backfires into blank radar. Recommend a one-shot live `curl` check against the
  endpoint during implementation. — `plan.md:124-126`
- [ ] (suggestion) Minor citation: the Context section references
  `cached_file_loader.cpp:55` / `:102` without a path; the file lives at
  `src/camp2/util/cached_file_loader.cpp`, not under `map_tiles/`. Diagnostic-only
  (not a file-to-change), but fix for accuracy. — `plan.md:22-26`

### Verified (no action)
- File targeting: all 5 files-to-change exist; line refs (`cached_tile_loader.cpp:36`
  for `url_str`, `:38` for the disk path; `map_tiles.cpp:229`/`:248`/`:260-266`;
  ADR-0004:100-105) are accurate.
- "No CMake change" holds: `cached_tile_loader.cpp` is already in `CAMP_MAP_SOURCES`
  (CMakeLists.txt:245) → compiled into `camp_map`, which `test_map_tiles_refresh`
  already links (CMakeLists.txt:519-523) along with `Qt5::Test`.
- Test approach is feasible: `tile_loader_` is parented to `MapTiles`
  (`map_tiles.cpp:43`), so `findChild<CachedTileLoader*>()` works like the existing
  `findChild<QTimer*>()` pattern; the `+1` monotonic guard makes the per-cycle token
  assertion deterministic (no wall-clock flakiness).
- Opt-in design is sound: only the radar layer calls `setRefreshInterval`
  (`background_manager.cpp:62`; confirmed by `map_tiles.h:64`), so tying
  `enableCacheBusting()` to it leaves every static OSM/WMTS layer's URL unchanged.
- Scope (single PR, one subsystem, ~80-120 lines), consequences table (complete),
  and ROS conventions (N/A — pure Qt/C++) all check out.

### Summary
Diagnosis and fix are well-reasoned and the plan is implementation-ready on the code
side — file targeting, the test strategy, and the opt-in design are all verified
against source. The one must-fix is governance, not code: the ADR-0004 update must go
through a new/superseding ADR (ADR-0012 does **not** sanction the in-place Consequences
edit the plan describes). Resolve that and address the cache-buster empirical/endpoint
caveats and the plan is good to implement.

### Recommended Actions
- [ ] Re-route the ADR-0004 update: new camp ADR for the corrected freshness decision
  + cache-buster, with ADR-0004 receiving only a cross-reference addendum (per
  ADR-0012). Amend `plan.md` step 5 and the ADR Compliance / Consequences tables.
- [ ] Decide on empirical confirmation of cause #1 (run the sidecar diff, or accept
  the cache-buster as a defensive fix) and note the decision in the plan.
- [ ] Verify the live IEM endpoint tolerates `?t=<token>` before relying on it.
- [ ] Fix the `cached_file_loader.cpp` path citation in the Context section.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-23 10:14 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-111 at `123d3ef`
**Mode**: pre-push
**Depth**: Deep (reason: ADR removal under docs/decisions/ — Deep promotion trigger; >200 lines)
**Must-fix**: 0 | **Suggestions**: 4
**Round**: 1 | **Ship**: recommended — no must-fix; cache-buster fix is correct and self-contained, remaining items are advisory.

### Findings
- [ ] (suggestion) `enableCacheBusting()` re-seeds to wall clock unconditionally — can regress below an already-bumped token if `setRefreshInterval(>0)` is called twice (unreachable today); harden to `cache_bust_==0`-only or `max(cache_bust_+1, now)` — `src/camp2/map_tiles/cached_tile_loader.cpp:98`
- [ ] (suggestion) No end-to-end test that `load()` emits a `?t=`-busted network URL when busting is enabled; the `invalidateCache()`-before-`load()` ordering is unguarded — `test/test_map_tiles_refresh.cpp`
- [ ] (suggestion) ADR-0004 hard-deleted rather than retained as `Status: Superseded (camp#118)` per supersede-don't-delete convention; camp#118 unverifiable from this host (gh unauthenticated) — `docs/decisions/0004-weather-radar-tile-provider.md`
- [ ] (suggestion) Blank-on-`?t=`-rejection is a silent failure mode if IEM ever stops ignoring unknown query params — `src/camp2/map_tiles/cached_tile_loader.cpp:119`

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-23 10:40 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-111 at `7c4233e`
**Mode**: pre-push
**Depth**: Deep (reason: ADR removal under docs/decisions/ — Deep promotion trigger; >200 lines)
**Must-fix**: 1 | **Suggestions**: 4
**Round**: 2 | **Ship**: continue — a genuine correctness gap in the new on-show feature (`itemChange`, added since Round 1) warrants a fix + one more read; the core 5-min refresh fix is sound.

### Findings
- [ ] (must-fix) `itemChange` invalidate-on-show wipes the disk cache + bumps the token but never rebuilds the tile set, so tiles already in `tiles_` (prior show, or the ctor-seeded zoom-0 tile loaded at token 0) are not re-fetched; toggling radar off→on re-shows the prior stale frame until the next 5-min refresh, missing the plan-step-7 "first paint fetches fresh, never stale" guarantee. Fix: call `setLayout(tile_layout_)` after bump+invalidate (mirror `onRefreshTimer`). Core `onRefreshTimer` path unaffected. — `src/camp2/map_tiles/map_tiles.cpp:304`
- [ ] (suggestion) Test gap (ties to must-fix): no test exercises `load()`'s `cache_bust_!=0` branch (busted network URL) or that becoming visible re-fetches; on-show tests assert only token advance, so they pass even with the must-fix bug present (false confidence). — `test/test_map_tiles_refresh.cpp:245`
- [ ] (suggestion) Operational (post-fix): forcing reload on show makes rapid visibility toggling re-download the whole layer (`removeRecursively` + per-tile guaranteed-miss GET); consider a freshness guard/debounce for marginal field connectivity. — `src/camp2/map_tiles/map_tiles.cpp:304`
- [ ] (suggestion) ADR-0004 hard-deleted rather than `Status: Superseded`; decision still live in code; recorded operator decision (plan step 5, camp#118) — confirm camp#118/#119 exist (unverifiable offline). Cross-confirmed with Round-1 review. — `docs/decisions/0004-weather-radar-tile-provider.md`
- [ ] (suggestion) `StaticLayerVisibilityDoesNotBust` comment claims it guards disk-cache-on-show but only asserts token==0; align comment or add a disk-state check. — `test/test_map_tiles_refresh.cpp:266`
