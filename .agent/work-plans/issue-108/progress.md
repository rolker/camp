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

## Plan Review
**Status**: complete
**When**: 2026-06-28 00:06 +00:00
**By**: Claude Code Agent (Claude Opus) (in-context — author self-review)

**Plan**: `.agent/work-plans/issue-108/plan.md` at `4d40d16`
**PR**: PR-less (--issue mode)
**Verdict**: changes-requested

### Findings
- [ ] (must-fix) New `test/test_gggs_band_select.cpp` not registered in `CMakeLists.txt` (`ament_add_gtest` + include/link, mirror `test_gggs_render` at `CMakeLists.txt:562`); without it the test never builds — `plan.md:57-65`
- [ ] (must-fix) Per-band NoData not re-queried — `has_nodata_`/`nodata_` are read once for band 1 (`gggs_tile.cpp:47-49`, used in range filter at `:93`); `setBand` must refresh them so a switched band ranges correctly (review-issue finding) — `plan.md:21-24`
- [ ] (suggestion) Stale doc comments at `gggs_tile.h:36` ("Read band 1") and `gggs_tile_layer.h:41-43` ("Slice 1: single-band… Band-select [follow-up]") not flagged for update (review-issue finding) — `plan.md:60-63`
- [ ] (suggestion) Fragment shader discards `v <= 0.0` and auto-ranges (`gggs_tile_layer.cpp:71-73`) — a second band with 0/negative valid samples renders blank/mis-ranged; acknowledge in Open Questions — `plan.md:92-95`
- [ ] (suggestion) Layer `setBand` must guard null `gl_context_` (not-yet-painted) before makeCurrent/texture release, as `releaseGL()` already does (`gggs_tile_layer.cpp:571`) — `plan.md:27-33`

**Note**: independence annotation applied per the skill's name-based rule (the `## Plan Authored` entry shares the `Claude Code Agent` agent-name); this review was nonetheless a fresh-context dispatch on a different model (Sonnet authored, Opus reviewed).
