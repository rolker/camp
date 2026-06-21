---
issue: 104
---

# Issue #104 — Stores layer redesign: store-tree browser → flat selectable display layers (band + colormap + order)

## Issue Review
**Status**: complete
**When**: 2026-06-21 01:06 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Issue**: #104
**Comment**: https://github.com/rolker/camp/issues/104#issuecomment-4760974444
**Scope verdict**: needs-splitting

### Actions
- [ ] Split into ~3 PRs: (3a) browser/manager + flat selectable layers, (3b) band-select + per-band colormap, (3c) GggsTileLayer compositing test — or at minimum land band-selection as its own commit-set behind the structural change. (Principle: Improve incrementally; Only what's needed)
- [ ] Write an ADR (or extend ADR-0003 §4 split-persistence) recording the browse-vs-compose split AND the persistence-format change (store roots → selected flat layers + band + colormap + order). This reverses camp#90's "store folders straight into the layers tree" design — ADR-0001 + ADR-0003 trigger. (Principle: Capture decisions; ADR-0001/0003 Action needed)
- [ ] Reframe item (4) "draw order untested": the Map model reorder path is ALREADY tested (test_map_model.cpp: ReorderKeepsModelValid, DropReordersToTargetRow, #84). What's untested is GggsTileLayer COMPOSITING — stack-order + opacity through the offscreen-FBO QPainter::drawImage paint path (gggs_tile_layer.cpp:499). Target the new test there (e.g. sidescan over bathy). (Principle: Test what breaks — Action needed)
- [ ] Band selection (item 3): GggsTile/loadPixels hardwire GetRasterBand(1) (gggs_tile.cpp:34,77); add a band index param plumbed through loadPixels + auto-range; reuse the existing camp#90 colormap LUT (setColormap). Add coverage rendering a non-1 band of a ≥2-band GeoTIFF. (Principle: Test what breaks)
- [ ] Reuse, don't rebuild: keep each flat layer a GggsTileLayer so camp#90 colormap + camp#102 lazy/async/default-off/visibility-persist carry over; add only the band-index parameter.
- [ ] State the persistence migration explicitly in the PR: old QSettings GggsStores/roots (+ per-layer visible) honored vs reset; update background_manager.cpp createDefaultLayers restore path (lines 77-86) and the onRemovedFromMap drop-on-remove analogue for selected layers; update .agents/README.md persistence notes. (Principle: A change includes its consequences)
