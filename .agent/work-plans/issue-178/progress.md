---
issue: 178
---

# Issue #178 — BlueTopo WMTS preset: QNetworkReply::ProtocolInvalidOperationError (bare zoom index in TileMatrix URL)

## Issue Review
**Status**: complete
**When**: 2026-07-31 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #178
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Only what's needed | OK | Fix is 1–2 lines in `tile_layout.cpp:36` — no new infrastructure, no speculative changes |
| Improve incrementally | OK | Self-contained bug fix; no other scope should be pulled in |
| A change includes its consequences | Watch | The issue calls out that bare-numeric servers must keep working (fallback); this invariant should be verified, ideally with a unit test for `TileLayout::getUrl()` covering both bare-numeric and `<gridset>:<z>` identifiers |
| Test what breaks | Watch | No tests for URL generation exist in the visible test directory; the fix should add at least a minimal case that would have caught this regression |
| Human control and transparency | OK | Root cause, fix path, and backward-compat constraint are all clearly stated in the issue |
| Workspace vs. project separation | OK | Change belongs in `camp` (project repo); no workspace infra is affected |
| Capture decisions | OK | No new design decision warranted; the fix follows naturally from how `TileMatrix::id` is already parsed |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0001 (Adopt ADRs) | No | Bug fix, not a design decision requiring an ADR |
| ADR-0002 (Worktree isolation) | No | Already in worktree on `feature/issue-178` |
| ADR-0008 (ROS 2 conventions) | No | C++ code change only; no package/launch/license impact |

### Consequences

- **Fallback path**: `zoom_levels[address.zoomLevel()].id` should be used when non-empty; when empty (e.g., bare-numeric servers), fall back to `std::to_string(address.zoomLevel())`. Both BlueTopo WMTS presets (`bluetopo:bathymetry`, `bluetopo:hillshade`) are affected and should be verified in the fix.
- **Related #177**: HTTP 400 responses are not cached (Qt reports them as errors), so #177's poisoning scenario does not compound this bug — no cross-issue fix needed.
- **No interface changes** — `TileLayout::ZoomLevel::id` already exists and is already populated from the WMTS capabilities XML; only the `getUrl()` lookup needs updating.

### Actions
- [ ] Verify fallback: when `zoom_levels[address.zoomLevel()].id` is empty, `getUrl()` should still emit the numeric index so bare-numeric servers keep working.
- [ ] Add a unit test for `TileLayout::getUrl()` covering both a bare-numeric and a `<gridset>:<z>` TileMatrix identifier, so this class of regression is caught locally.
