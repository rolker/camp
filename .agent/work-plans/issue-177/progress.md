---
issue: 177
---

# Issue #177 — WMS ServiceExceptionReport cache poisoning

## Issue Review
**Status**: complete
**When**: 2026-07-28 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #177
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Actions
- [ ] Commit to a decode-gate strategy in plan-task: Content-Type header check vs. image decode — both are referenced in the issue body; pick one and record the rationale (consider an ADR addendum if this introduces a general cache-validation policy).
- [ ] Add test coverage for the decode gate: a unit test that verifies `CachedFileLoader` does NOT write a cache file when an HTTP 200 response carries a non-image body (mock a WMS XML `ServiceExceptionReport` 200 response).
- [ ] Confirm `&EXCEPTIONS=BLANK` (Option 1) is intended as complementary hardening or is out of scope — the issue says "the two can be combined" but plan-task should state the PR scope explicitly.
- [ ] Verify that the decode gate does not regress OSM or radar tile loading — `CachedFileLoader` is shared by all sources; any content-type or image-decode check must tolerate the bodies those sources return.

## Plan Authored
**Status**: complete
**When**: 2026-07-28 12:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-177/plan.md` at `3c92151`
**Branch**: feature/issue-177 at `3c92151`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-07-28 14:30 +00:00
**By**: Claude Code Agent (Claude Opus)
<!-- Independence: the `## Plan Authored` By agent-name ("Claude Code Agent") equals
     $AGENT_NAME, but this workspace shares one agent name across all Claude agents,
     distinguished by model. Plan authored by Sonnet; this review is a fresh-context
     Opus dispatch — genuinely independent, so the self-review annotation is omitted. -->

**Plan**: `.agent/work-plans/issue-177/plan.md` at `3c92151`
**PR**: PR-less (--issue / file-path mode; feature/issue-177)
**Verdict**: changes-requested

### Findings
- [ ] (must-fix) Blanket gate in shared `downloadFinished` will regress WMTS capabilities — `capabilities.cpp:26` loads `wmts/<label>/capabilities.xml` through the SAME `CachedFileLoader::instance()->load()`, and `Capabilities::dataLoaded` parses it as XML (`QDomDocument`). Gating the `emit client->dataLoaded` (and disk write) on `isAcceptableImageBody` suppresses that legitimate non-image body → WMTS layer discovery breaks. Make the gate caller-aware (e.g. an `expects_image` flag on `CachedFileClient` set by tile loaders) instead of unconditional. — `plan.md:41`, `plan.md:88`
- [ ] (must-fix) Consequences table is factually wrong — `plan.md:88` claims WMTS capabilities "use separate loaders". They do not; they share `CachedFileLoader`. Correct the table and add the resulting file target. — `plan.md:88`
- [ ] (suggestion) File-targeting gap consequent to the above — a caller-aware design will require touching `src/camp_map/map_tiles/cached_tile_loader.cpp` (to opt tile loads into image validation); it is not in the Files-to-Change table. — `plan.md:59`
- [ ] (suggestion) Content-Type source unspecified — step 2 calls `isAcceptableImageBody(data, content_type)` but never says how `content_type` is obtained from the reply (`reply->header(QNetworkRequest::ContentTypeHeader)` / `reply->rawHeader("Content-Type")`). Spell it out. — `plan.md:41`
- [ ] (suggestion) Tests are unit-only on the static helper — review-issue action #2 asked for a test that `CachedFileLoader` does NOT write a cache file on a non-image 200. The plan tests the decision function only, not that `downloadFinished` skips the write/emit. Acceptable given network-mock cost, but once the gate is caller-aware add a regression case asserting capabilities XML still emits. — `plan.md:48`

### Affirmed
- Line references accurate (`data` read at cpp:85, disk-write block 87–113, emit at 115).
- CMake registration mirrors existing `test_wms_url_generation` (`camp_map Qt5::Core Qt5::Gui`).
- `&EXCEPTIONS=BLANK` correctly excluded with rationale (blank image would decode and cache as a "valid" tile).
- Decode-as-authority + header-as-early-out is a sound, testable structure.

## Implementation
**Status**: blocked
**When**: 2026-07-28 14:07 +00:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-177 at `39a9eb8`
**Addressed**: none — no qualifying review entry exists to address

### Actions
- (no-op) `address-findings` found no `## Integrated Review` (post-PR) or `## Local Review (Pre-Push)` (pre-push) entry — the shared parser (`progress_read.py --type "Integrated Review" --type "Local Review (Pre-Push)"`) returns an empty `entries[]`. Per the skill's Step 2, this is the "no qualifying entry" case: report and exit without addressing anything. The skill must **not** fall back to the `## Plan Review` entry (a pre-implementation review it does not act on).

### Why this is a no-op
`address-findings` sits downstream of **implement → review-code**. The latest
review here is a `## Plan Review` with verdict `changes-requested` — the
lifecycle has only reached **review-plan**, and no implementation commits exist
on `feature/issue-177` (`git log main..feature/issue-177` is empty). There is no
post-implementation review whose findings could be addressed.

### Next step
Revise the plan to resolve the two **must-fix** Plan Review findings (the blanket
gate in shared `downloadFinished` regressing WMTS capabilities loading, and the
factually-wrong consequences table claiming WMTS uses separate loaders), then
re-run **review-plan**. Only once the plan is approved and **implement →
review-code** have run will there be a review entry for `address-findings` to
consume.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-28 19:08 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-177 at `c6cb142`
**Mode**: pre-push
**Depth**: Standard (reason: medium change touching the shared singleton `CachedFileLoader` used by all tile sources + WMTS capabilities — cross-source regression risk; no security/concurrency/cross-repo signal for Deep)
**Must-fix**: 0 | **Suggestions**: 2
**Round**: 1 | **Ship**: recommended — no must-fix; both must-fix items from the earlier Plan Review are resolved by the `expects_image` design; only two low-severity suggestions remain.

### Findings
- [ ] (suggestion) Format-authority asymmetry: gate `isAcceptableImageBody` accepts any QImage-decodable format, but consumer forces `pixmap.loadFromData(data,"png")` — a valid non-PNG image would pass the gate, cache, then render blank. Not exploitable today (all sources PNG). — `src/camp_map/map_tiles/cached_tile_loader.cpp:143`
- [ ] (suggestion) Redundant full-image decode per disk-cache hit: gate `QImage` decode + consumer `QPixmap` decode on every `file://` re-read. Negligible per 256px tile; optional. — `src/camp_map/util/cached_file_loader.cpp:108`

### Notes
- Static analysis: cppcheck clean (only the known Qt `slots`-macro config limitation); camp does not enforce ament_cpplint (no `ament_lint_auto` in CMakeLists, sibling tests carry no copyright header, custom include guards) — the 25 cpplint items are repo-convention noise, not actionable.
- Adversarial: 2 disjoint-lens Claude passes (Lens A logic, Lens B systemic/safety) both returned clean; the format-authority point surfaced in Lens A and was echoed by Lens B. Local Adversarial skipped (Ollama not running).
- Governance: both `CachedFileClient` construction sites verified (`cached_tile_loader.cpp:55` → `true`; `wmts/capabilities.cpp:24` → default `false`, ungated). Consequences map satisfied.
- ADR-0018: full `colcon build` could not reconfigure in this shell (lower-layer `marine_ais_msgs` not on prefix path in this worktree). Gate logic was independently compiled & run headless against all 7 test-case inputs — all matched expectations, confirming the unit test needs no QApplication. Run `./build.sh camp && ./test.sh camp` on a fully-sourced env before push.

### Next step
Approved pre-push review. Address the two suggestions (optional) or track them, then push / open PR and hand off to **triage-reviews** in a fresh-context sub-agent. Per ADR-0018, complete a full local `build + test` on a properly-sourced environment before pushing.

## Integrated Review
**Status**: complete
**When**: 2026-07-28 15:47 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #179 at `bea5b55`
**Sources**: 3 (Copilot R1 @ `bea5b55`, Local Review (Pre-Push) @ `c6cb142`, CI rollup @ `bea5b55`)
**Cross-source confirmations**: 0
**CI**: all-pass (`build-and-test` success, `copilot-pull-request-reviewer` success)

### Findings
- [ ] (must-fix, Copilot) Existing poisoned cache entries never self-heal: `load()` prefers the on-disk `file://` copy, so a pre-fix `<z>/<x>/<y>.png` holding a `ServiceExceptionReport` is re-read, rejected by the new gate, and returned from without deleting — the tile stays blank forever (no refetch) for non-refreshing layers until LRU eviction (#98). Field caches are already poisoned; that is what #177 was filed for. Fix: in the rejection branch, when `reply->request().url().isLocalFile()`, delete the cache file and its `.json` sidecar, then re-issue the fetch against the original network URL (stash it as a client property in `load()` alongside `cache_local_path`). No retry loop is possible — the retry is a non-local request, and a second rejection returns without re-entering. — `src/camp_map/util/cached_file_loader.cpp:108`
- [ ] (suggestion, Local Review) Format-authority asymmetry: the gate accepts any `QImage`-decodable body, but the consumer forces `pixmap.loadFromData(data, "png")`, so a valid non-PNG image would pass the gate, be cached, and render blank permanently — the same persistent-silent-blank failure class this PR exists to close. Not reachable today (WMS forces `FORMAT=image/png`; XYZ sources are PNG). One-line fix: drop the `"png"` format hint and let Qt sniff. — `src/camp_map/map_tiles/cached_tile_loader.cpp:143`
- [ ] (suggestion, Local Review) Redundant full-image decode per disk-cache hit (gate `QImage` decode + consumer `QPixmap` decode on every `file://` re-read). Negligible per 256 px tile; optional. — `src/camp_map/util/cached_file_loader.cpp:108`

### False positives
- None. The single Copilot comment is valid and is the highest-severity finding of this round.

### Next step
Hand off to `address-findings` for the must-fix (self-healing of poisoned cache entries) plus the one-line format-hint suggestion; the decode-cost suggestion may be closed as won't-fix. A re-review (`review-code`) and another triage round follow before merge.
