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
