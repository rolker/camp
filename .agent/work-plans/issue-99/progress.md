---
issue: 99
---

# Issue #99 — Add auto-refreshing weather radar tile overlay (stacked layer on the map system)

## Issue Review
**Status**: complete
**When**: 2026-06-18 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet 4.6)

**Issue**: #99
**Comment**: https://github.com/rolker/camp/issues/99#issuecomment-4742140067
**Scope verdict**: well-scoped

### Actions
- [ ] Add unit/integration tests for Phase 2 refresh timer: verify interval fires, cache is invalidated, and memory stays bounded across N refresh cycles (addresses #98 risk).
- [ ] Document tile provider choice (RainViewer vs. NOAA nowCOAST) in the PR with ToS/offline/coverage rationale; consider an ADR if introducing a long-lived external dependency.
- [ ] Verify `CachedTileLoader` network-error path degrades gracefully (no crash/block) when radar tile server is unreachable (field ops concern).
- [ ] Phase 2 PR should be explicitly coordinated with / blocked on #98 (OOM crash) fix — periodic refresh shares the tile/raster code path implicated in the crash.

## Plan Authored
**Status**: complete
**When**: 2026-06-18 14:30 +00:00
**By**: Claude Code Agent (Claude Sonnet 4.6)

**Plan**: `.agent/work-plans/issue-99/plan.md` at `48c63c5`
**Branch**: feature/issue-99 at `48c63c5`
**Phases**: single PR (Phase 1 = NOAA radar WMTS layer; Phase 2 = refresh timer + eviction — both in same PR)

### Open questions
- [ ] Confirm NOAA nowCOAST WMTS endpoint URL before hardcoding — `https://nowcoast.noaa.gov/arcgis/rest/services/nowcoast/radar_meteo_imagery_nexrad_time/MapServer/WMTS` needs live verification at implementation time.
- [ ] Create ADR-0004 for NOAA nowCOAST external dependency — recommend including in the same PR as Phase 1.
- [ ] 5-minute refresh cadence: confirm this is appropriate or make it configurable (hardcoded default for now, follow-up issue to expose as user setting).
