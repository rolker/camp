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
- [x] (must-fix, Copilot) Existing poisoned cache entries never self-heal: `load()` prefers the on-disk `file://` copy, so a pre-fix `<z>/<x>/<y>.png` holding a `ServiceExceptionReport` is re-read, rejected by the new gate, and returned from without deleting — the tile stays blank forever (no refetch) for non-refreshing layers until LRU eviction (#98). Field caches are already poisoned; that is what #177 was filed for. Fix: in the rejection branch, when `reply->request().url().isLocalFile()`, delete the cache file and its `.json` sidecar, then re-issue the fetch against the original network URL (stash it as a client property in `load()` alongside `cache_local_path`). No retry loop is possible — the retry is a non-local request, and a second rejection returns without re-entering. — `src/camp_map/util/cached_file_loader.cpp:108`
- [x] (suggestion, Local Review) Format-authority asymmetry: the gate accepts any `QImage`-decodable body, but the consumer forces `pixmap.loadFromData(data, "png")`, so a valid non-PNG image would pass the gate, be cached, and render blank permanently — the same persistent-silent-blank failure class this PR exists to close. Not reachable today (WMS forces `FORMAT=image/png`; XYZ sources are PNG). One-line fix: drop the `"png"` format hint and let Qt sniff. — `src/camp_map/map_tiles/cached_tile_loader.cpp:143`
- [x] (suggestion, Local Review) Redundant full-image decode per disk-cache hit (gate `QImage` decode + consumer `QPixmap` decode on every `file://` re-read). Negligible per 256 px tile; optional. — `src/camp_map/util/cached_file_loader.cpp:108` (deferred: declined as won't-fix by operator decision 2026-07-31 — negligible per-tile cost)

### False positives
- None. The single Copilot comment is valid and is the highest-severity finding of this round.

### Next step
Hand off to `address-findings` for the must-fix (self-healing of poisoned cache entries) plus the one-line format-hint suggestion; the decode-cost suggestion may be closed as won't-fix. A re-review (`review-code`) and another triage round follow before merge.

**Operator decision (2026-07-31, run-issue checkpoint)**: fix finding 1 (must-fix,
self-heal) and finding 2 (format-hint drop); finding 3 (redundant double decode)
is declined as won't-fix — do not implement it, note it as closed by operator
decision.

## Implementation
**Status**: complete
**When**: 2026-07-31 15:13 +00:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-177 at `6be1f4d`
**Addressed**: `## Integrated Review` (2026-07-28 15:47 -04:00, PR #179 @ `bea5b55`)
**Commits**: `c925cbd` (finding 1), `6be1f4d` (finding 2)

### Actions
- [x] (must-fix, Copilot) Self-heal poisoned cache entries — on a local-file rejection in `downloadFinished`, delete the poisoned cache file and its `.json` sidecar and re-issue the fetch against the original network URL (now stashed as a `network_url` client property in `load()`). The retry is a network request, so a second rejection falls through to the non-local return — no retry loop. — `src/camp_map/util/cached_file_loader.cpp:84,116` (`c925cbd`)
- [x] (suggestion, Local Review) Format-authority asymmetry — dropped the `"png"` format hint in the tile consumer so Qt sniffs the format; a valid non-PNG image that passed the loader's decode gate no longer renders blank. — `src/camp_map/map_tiles/cached_tile_loader.cpp:143` (`6be1f4d`)
- [x] (suggestion, Local Review) Redundant full-image decode per disk-cache hit — `src/camp_map/util/cached_file_loader.cpp:108` (deferred: declined as won't-fix by operator decision 2026-07-31 — negligible per-256px-tile cost)

### Checks
- Both changed translation units syntax-check clean (`g++ -fsyntax-only -std=c++17` with Qt5 Core/Gui/Network/Widgets includes). The self-heal branch reuses `QFile`/`QFileInfo`/`load()` patterns already present in the same function; `isAcceptableImageBody` (the unit-tested gate) is unchanged, so the existing `test_cached_file_loader_validation` suite still applies. No new unit test added: the self-heal logic lives in `downloadFinished` (network + `QApplication` + file I/O), not cheaply unit-testable without a network mock — consistent with the established unit-only test scope, and finding 1 did not request a test.
- Full-workspace `colcon build`/`test` did not complete in this shell (multi-layer build times out; lower-layer prefix path unavailable — same limitation the Local Review recorded). Per ADR-0018, run `./build.sh camp && ./test.sh camp` on a properly-sourced environment before push.

### Next step
Lifecycle: **Implementation** → **review-code** (re-review the fixes). Hand off to a fresh-context sub-agent:

    .agent/scripts/dispatch_subagent.sh --mode in-process --issue 177 --skill review-code

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-31 15:26 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-177 at `d6284d7`
**Mode**: pre-push
**Depth**: Standard (reason: change to shared singleton `CachedFileLoader` used by all tile sources + WMTS capabilities — cross-source regression risk; no security/cross-repo signal for Deep)
**Must-fix**: 1 | **Suggestions**: 0
**Round**: 2 | **Ship**: continue — one genuine correctness must-fix newly introduced by the self-heal code (unbounded loop if `QFile::remove` fails, plus a false "no retry loop is possible" comment); mechanical one-line guard, warrants fix + a fast re-confirm. Round 1 had 0 must-fix, so this is a new edge in the self-heal code, not a rising trend.

### Findings
- [ ] (must-fix) Self-heal can busy-loop if `QFile::remove()` fails: the re-issued `load()` re-selects the `file://` path while the poisoned file still exists (cpp:75), re-reading it → decode-reject → `isLocalFile()` → remove-fails-again → re-issue, an unbounded async loop via the QNAM `finished` signal. The "no retry loop is possible" comment holds only when removal succeeds. Guard the network re-issue on successful removal (or `!poisoned.exists()`); else fall through to `deleteLater(); return;`. — `src/camp_map/util/cached_file_loader.cpp:126`

### Notes
- Round-2 re-review focused on the two new commits `c925cbd` (self-heal) + `6be1f4d` (format-hint drop), in the context of the full branch diff. The declined redundant-double-decode item (operator won't-fix 2026-07-31) was not re-raised, per instruction.
- Static analysis: cppcheck clean except the known Qt `slots`-macro config limitation (unknownMacro) — repo-convention noise, not actionable (same as Round 1).
- Adversarial: 2 disjoint-lens Claude passes. Lens A (logic) surfaced the loop-on-remove-failure must-fix (independently lead-confirmed). Lens B (systemic) raised an "over-broad deletion / #99 regression" must-fix that the lead **rejected** on evaluation: self-heal fires only on `NoError` + a decode-failing local read; valid tiles decode and are untouched, deleted entries were already unrenderable (same Qt plugins as the consumer), and network errors bypass the gate via the `#99` else-branch — no user-visible regression. Lens A's empty-`network_url` and path-symmetry suggestions are unreachable for image-expecting clients (the only ones reaching self-heal) and were dropped. Local Adversarial off (`--no-local`, standing opt-out per workspace#590).
- Governance: WMTS capabilities (`expects_image=false`) fully bypasses the gate+self-heal block (verified); `#99` network-error degradation path unchanged. Consequences map satisfied.
- ADR-0018: full `colcon build`/`test` not completed in this worktree (multi-layer prefix path unavailable — same limitation as Round 1). Run `./build.sh camp && ./test.sh camp` on a fully-sourced env before push.

### Next step
Changes-requested. Host (`/run-issue`) dispatches **address-findings** to work the single must-fix (guard the self-heal re-issue against `QFile::remove` failure + correct the comment), then re-dispatches **review-code** for Round 3. Diff is not pushed until a pre-push review returns **approved**.
