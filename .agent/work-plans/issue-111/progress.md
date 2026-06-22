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
