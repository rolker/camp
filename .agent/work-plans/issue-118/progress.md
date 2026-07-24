---
issue: 118
---

# Issue #118 — WMS GetMap layer support + nowCOAST radar migration

## Issue Review
**Status**: complete
**When**: 2026-07-24 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #118
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Actions
- [ ] Settle the WMS layer granularity design choice (per-tile bbox `GetMap` vs single full-viewport `GetMap`) before implementation — these branch significantly in architecture; record the decision in a new camp ADR.
- [ ] Clarify time-dimension handling for nowCOAST MRMS: fetching available times (e.g. via `GetCapabilities` or MRMS time service) is more complex than a URL parameter and must be scoped explicitly in plan-task.
- [ ] Extend `TileLayerPreset` / persistence schema (ADR-0003 addendum) for WMS-specific fields (at minimum `wms_version`, `crs`; confirm `layer_id` serialization path for WMS type in `persistTileLayer`).
- [ ] Add test coverage for the new WMS layer type: time-dimension fetching, graceful degradation (blank tile on network failure), and the nowCOAST migration path.
- [ ] Confirm the nowCOAST `base_reflectivity_mosaic` runtime field dependency and graceful-degradation story matches the existing `CachedFileLoader` blank-tile fallback contract before plan-task closes scope.
