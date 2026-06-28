---
issue: 108
---

# Issue #108 — GeoTIFF band picker for store display layers

## Issue Review
**Status**: complete
**When**: 2026-06-27 23:56 +0000
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #108
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Actions
- [ ] Confirm #104's flat-selectable-layer work is merged/rebased before starting — the flat display layer is the anchoring UI surface for the band picker.
- [ ] Per-band NoData: re-query `GetNoDataValue()` for the selected band (not band 1's cached value) when the operator switches bands (`gggs_tile.cpp:48`).
- [ ] Reset auto-range (`data_min_`/`data_max_`) and reload pixels on band switch — range is band-specific, reusing band 1's range for another band would give wrong colormap scaling.
- [ ] Single-band graceful handling: disable or hide the band-picker context menu item when `GetRasterCount() == 1` — nothing to pick.
- [ ] Update doc comments that will be stale after the change: `gggs_tile.h:36` ("Read band 1"), `gggs_tile_layer.h:41-42` ("Slice 1: single-band…  Band-select + colormap are [follow-up]").
- [ ] Persist the selected band index via QSettings alongside colormap, consistent with how colormap is persisted in `GggsTileLayer`.

## Plan Authored
**Status**: complete
**When**: 2026-06-28 00:10 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-108/plan.md` at `4d40d16`
**Branch**: feature/issue-108 at `4d40d16`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.
