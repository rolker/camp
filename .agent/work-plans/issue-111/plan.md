# Plan: Weather radar overlay shows stale precipitation (does not advance to latest mosaic)

## Issue

https://github.com/rolker/camp/issues/111

## Context

The NEXRAD radar overlay (#99, ADR-0004) shows stale precipitation: it never
advances to the latest IEM mosaic across the 5-minute auto-refresh, including
across sessions. Reading the current code (post-#98) rules out two of the three
candidate causes the issue lists and confirms the third:

- **Cause #1 — CDN/HTTP caching of the static tile URL (ROOT CAUSE).** The IEM
  request URL `.../nexrad-n0q-900913/{z}/{x}/{y}.png` carries no cache-buster.
  `CachedTileLoader::invalidateCache()` (`cached_tile_loader.cpp:50`) correctly
  wipes the per-layer disk subdir each cycle, so the next `load()` does a real
  network GET — but to the *same* URL, which a CDN/proxy between CAMP and IEM can
  satisfy from its own cache with an older tile. Neither the `onRefreshTimer`
  comment (`map_tiles.cpp:260`) nor ADR-0004 (Consequences, lines 100-105)
  accounts for caching *between* client and origin — that is the gap.
- **Cause #2 — global `CachedFileLoader` re-serving (DISPROVEN).**
  `CachedFileLoader::load` (`cached_file_loader.cpp:55`) gates re-use solely on
  `file_path.exists()`; there is no in-memory URL cache, no in-flight dedup, and
  no `QNetworkDiskCache` on the QNAM (ctor lines 18-24). Once the disk subdir is
  wiped it cannot re-serve.
- **Cause #3 — timer cadence / visibility pausing (DISPROVEN).**
  `onRefreshTimer()` runs unconditionally; `setRefreshInterval` only stops the
  timer for `msec<=0`; nothing in the visibility path pauses it.

Fix: attach a per-refresh cache-buster query param to the radar layer's request
URL so each cycle's GET is a URL the CDN cannot answer from cache.

## Approach

1. **Add an opt-in cache-buster to `CachedTileLoader`** (the per-layer network
   choke point, owned per-`MapTiles`). New state `quint64 cache_bust_ = 0;`
   (0 = disabled). Methods:
   - `void enableCacheBusting()` — seed `cache_bust_` from wall clock
     (`QDateTime::currentMSecsSinceEpoch()`); marks the layer as busting.
   - `void bumpCacheBust()` — advance with a monotonic guard
     `cache_bust_ = std::max<quint64>(cache_bust_ + 1, currentMSecsSinceEpoch())`.
     Time-seeding gives cross-session freshness (the "across sessions" symptom);
     the `+1` guard guarantees a *strictly distinct* value even on back-to-back
     calls, which is the testable invariant.
   - `quint64 cacheBustToken() const` — getter for tests.
   - `static std::string withCacheBust(const std::string& url, quint64 token)` —
     pure helper: appends `?t=<token>` (or `&t=` if the URL already has a `?`).
2. **Apply it in `CachedTileLoader::load`** (`cached_tile_loader.cpp:36`): when
   `cache_bust_ != 0`, set `url_str = withCacheBust(url_str, cache_bust_)`. The
   busted URL is used *only* as the network `url_str`; the disk path stays
   `z/x/y.png` (from `operator std::string`), so the buster does not bloat disk
   cache or change the within-cycle disk-hit behavior.
3. **Drive it from `MapTiles`:**
   - `setRefreshInterval(msec>0)` (`map_tiles.cpp:229`) calls
     `tile_loader_->enableCacheBusting()` — ties busting to "this is a refreshing
     layer". Static OSM/WMTS layers never call this, so their URLs are unchanged.
   - `onRefreshTimer()` (`map_tiles.cpp:248`) calls `tile_loader_->bumpCacheBust()`
     *before* `setLayout()`/`update()`, so the loads triggered by the next
     `paint()` carry the new token.
4. **Correct the stale freshness comment** at `map_tiles.cpp:260-266`: disk
   invalidation alone does NOT force a fresh frame across a CDN; the cache-buster
   is what defeats intermediary caching. Keep the timestamp-pinning caveat.
5. **Update ADR-0004** Consequences (lines 100-105): acknowledge intermediary
   HTTP/CDN caching and record the cache-buster mitigation — per the workspace
   principle "capture decisions, not just implementations".
6. **Tests** (extend `test/test_map_tiles_refresh.cpp`; already CMake-registered,
   links `camp_map` where `CachedTileLoader` lives — no CMake change):
   - `withCacheBust` pure-logic: distinct tokens → distinct URLs; correct `?`
     vs `&` joining for URLs with/without an existing query.
   - Refresh path: reach the loader via
     `layer->findChild<camp::map_tiles::CachedTileLoader*>()` (it is parented to
     `MapTiles`, like the existing `findChild<QTimer*>()` pattern); after
     `enableCacheBusting()`, two `invokeRefresh()` cycles strictly increase
     `cacheBustToken()`.
   - Opt-in: a layer that never calls `setRefreshInterval` keeps
     `cacheBustToken() == 0` (static layers unaffected). No live-network test.

## Files to Change

| File | Change |
|------|--------|
| `src/camp2/map_tiles/cached_tile_loader.h` | Add `cache_bust_`, `enableCacheBusting()`, `bumpCacheBust()`, `cacheBustToken()`, static `withCacheBust()` |
| `src/camp2/map_tiles/cached_tile_loader.cpp` | Implement the above; apply buster in `load()` |
| `src/camp2/map_tiles/map_tiles.cpp` | `setRefreshInterval` → `enableCacheBusting`; `onRefreshTimer` → `bumpCacheBust`; correct freshness comment (`:260`) |
| `docs/decisions/0004-weather-radar-tile-provider.md` | Consequences addendum: CDN caching + cache-buster mitigation |
| `test/test_map_tiles_refresh.cpp` | Add `withCacheBust` + token-bump + opt-in tests |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Enforcement over documentation | Regression test asserts per-refresh URL/token distinctness — the enforceable invariant — without a live-network test |
| Capture decisions, not just implementations | ADR-0004 Consequences addendum records that the static-URL + disk-invalidation reasoning was incomplete (CDN caching) |
| A change includes its consequences | Same PR updates code, the `onRefreshTimer` comment, ADR-0004, and the test |
| Only what's needed | Minimal per-layer cache-buster, not a general `Cache-Control`/HTTP-cache layer. Opt-in via `setRefreshInterval`, so non-radar layers are untouched |
| Test what breaks | Invariant = "a refresh produces a request the CDN cannot satisfy from cache"; asserted via strict per-cycle token monotonicity |
| Safety / operational awareness (project, inferred) | Stale radar can mislead more than blank radar; a visible-staleness safeguard is noted as possible follow-up (out of scope here) |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| camp ADR-0004 (radar tile provider) | Yes (directly) | Consequences addendum records CDN caching + cache-buster; URL contract amended in the same PR (permitted by ADR-0012 cross-reference addendum) |
| camp ADR-0002 (Web-Mercator scene/layer) | No | Radar stays in pure-Qt `libcamp_map`; no scene/layer-model change |
| workspace ADR-0008 (ROS 2 conventions) | No | camp is a Qt/C++ app; no ROS package/launch/`package.xml` change |
| workspace ADR-0001 (adopt ADRs) | Yes (via 0004) | Corrected reasoning captured in the ADR, not just the commit |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Radar request URL (add cache-buster) | ADR-0004 Consequences (lines 100-105) | Yes (step 5) |
| Refresh freshness semantics | `onRefreshTimer` comment (`map_tiles.cpp:260`) | Yes (step 4) |
| `CachedTileLoader` / refresh path | `test/test_map_tiles_refresh.cpp` | Yes (step 6) |

## Open Questions

- **Empirical confirmation before coding (review-issue recommendation):** confirm
  cause #1 by comparing the served tile bytes / reply-headers across two refresh
  cycles via the `.json` sidecar `CachedFileLoader` already writes
  (`cached_file_loader.cpp:102`). If the served tile already varies cycle-to-cycle
  without a buster, the mechanism is elsewhere.
- **Does IEM `tile.py` accept and ignore an unknown `?t=` query param** (return
  the tile normally)? Standard for static tile servers, but verify against the
  live endpoint so the buster doesn't trigger 4xx/blank tiles.

## Estimated Scope

Single PR. One subsystem (`map_tiles`), one mechanism (per-refresh cache-buster),
plus the ADR/comment/test consequences. ~80-120 lines incl. tests.
