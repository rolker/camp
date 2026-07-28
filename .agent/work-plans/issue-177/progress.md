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
