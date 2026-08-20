---
issue: 171
---

# Issue #171 — Live-coverage display collapses to low resolution during eviction (folding frees almost no memory)

## Issue Review
**Status**: complete
**When**: 2026-08-20 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #171 (and companion #172, reviewed together per operator directive)
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Summary

Issues #171 and #172 are the two halves of world-store LOD step 4: the live
coverage cache adopts the shared fold/pyramid machinery. Steps 1–3 already landed
(camp PR#173, uma PR#287, camp PR#183 / ADR-0013). The eventual PR closes both.

**#171 root cause** (verified in source at `foldIntoParent()` line 589):
`SonarLiveTile(parent_index, fine.width(), fine.height())` allocates each
overview tile at the same pixel count as the fine tile (960×960, ~11 MB). The
full chain to level 0 creates ~8 overview tiles per evicted fine tile. With 45
accumulated overview tiles at ~11 MB each (~500 MB) against a 512 MiB budget,
phase-1 eviction gains no headroom — each fine-tile eviction generates a same-size
overview — and runs until all 38 fine tiles are gone.

**#172 root cause** (ADR-0010 D2 / ADR-0013 §"The camp#172 hook"):
Evicted fine tiles remain in reconciler possession (`markHave`) so re-request is
never triggered, and `warmLoad()` is the only disk-reload path (startup only).
ADR-0013 deliberately left the `hasUnloadedVisibleTiles` + snapshot-filtered
worker hook enabled-but-unimplemented.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Capture decisions, not just implementations | Action needed | Camp ADR-0010 D3 defines same-pixel-size overviews; any fix that decimates overview pixel size (direction 1) or discounts overviews in the budget (direction 2) is a design change — ADR-0010 amendment required in the same PR. Camp ADR-0013 §"camp#172 hook" should be updated from "enabled, not implemented" to "implemented" once the reload hook lands. |
| A change includes its consequences | Action needed | Tests needed: (a) regression that eviction at survey scale (45 overview tiles, 38 fine tiles) leaves budget headroom after the fix; (b) regression that an evicted fine tile is reloaded when the viewport re-enters its area (#172). |
| Human control and transparency | Watch | Budget default (512 MiB) was designed before the overview-accumulation math was understood to be same-pixel-size. After the fix the parameter semantics change; `LiveTileCache/max_vram_bytes` documentation should note the new effective range. |
| Only what's needed | OK | Fix scope is tight: fix #171 in `foldIntoParent()` / `accountedBytes()`, implement the #172 hook. No unrelated rework. |
| Improve incrementally | OK | Staying within the established step-4 shape (uma shared fold engine + ADR-0013 reload seam); no redesign needed. |
| Test what breaks | Watch | The eviction collapse is a field-observed failure at realistic survey scale; a test that seeds realistic tile counts and confirms budget headroom after eviction is the highest-value regression. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| Camp ADR-0010 (bounded eviction / overview pyramid) | Yes — directly | D3 chose match-resolution overview tiles. Fix direction 1 (decimate pixel size) amends D3. Fix direction 2 (discount overviews in budget) amends D1. Either way an amendment is mandatory in the same PR. D2 limitation (no on-demand reload) is resolved by #172; amendment should reflect that too. |
| Camp ADR-0013 (LOD level selection / demand-driven load) | Yes — #172 hook | §"The camp#172 hook" is explicitly "enabled, not implemented"; implementing it updates the status of that clause. ADR-0013 should get a note once implemented. |
| Uma ADR-0011 (overview pyramids — sidecar layout / fold policy) | Yes — step-4 adoption | The shared `overview_builder.hpp` is the step-4 fold engine; camp's `foldIntoParent()` should converge to it. Uma ADR-0011 defines the sidecar layout (flat `overviews/<level>_<row>_<col>.tif`) that camp already follows — but the fold policy (MEAN for imagery) and pixel-size semantics (same pixel count per tile at every level) should be confirmed aligned or noted as a divergence. |
| Uma ADR-0010 (geospatial world model) | Watch | Referenced as context for D1 budget accounting; no direct change needed, but the nightly-regen anti-clobber / catalog prune propagation mentioned in the operator's locked decisions should be confirmed in scope. |
| Camp ADR-0006 (live tile cache persistence) | Watch | ADR-0010 extended ADR-0006; eviction and reload semantics built on ADR-0006 guarantees. No amendment needed if the fix doesn't touch the persistence contract. |
| Workspace ADR-0001 (adopt ADRs) | Yes | Design decision (which fix direction) must be recorded before or alongside implementation. |

### Consequences

Per the consequences map and the code:

- **If `foldIntoParent()` is changed** (pixel-size decimation): rendering in `items()` emits overviews first (coarse→fine) — smaller overview tiles still paint correctly because the renderer scales to scene coordinates, not pixel-for-pixel. `recomputeBounds()` and `foldAutoRange()` union both maps — no change needed there. Write-through path (`scheduleWriteThrough`) writes to `overviews/` — file sizes will change; on-disk warm-load (startup) must handle the new sizes. Tests that assert overview tile dimensions must be updated.
- **If budget accounting changes**: `accountedBytes()` changes — tests that assert budget tracking behavior need updating.
- **If the #172 reload hook is implemented**: `evictIfOverBudget()` and the reload hook must not ping-pong; a hysteresis margin (e.g., reload only when under some fraction of budget) or a cool-down must be explicit. ADR-0010 D4 (GUI-thread-only mutation) constrains how the reload can be dispatched — same constraint as the existing demand-driven worker in `GggsTileLayer` (ADR-0013 §"demand-driven loading").
- **Nightly-regen anti-clobber → catalog prune**: the operator's locked decisions include propagating nightly-regen anti-clobber to the operator cache via catalog prune; confirm whether this is in scope for the step-4 PR or a follow-up.

### Recommendations

- Fix direction 1 (decimate overview pixel size) is more faithful to the standard pyramid model (uma ADR-0011's per-tile-same-pixel-count but each level covers 4× more area means 1/4 the spatial resolution per pixel — still same pixel count). The memory math: a chain of 8 levels = 11 × (1 + 1/4 + 1/16 + …) ≈ 14.7 MB total per new fold chain vs. 88 MB with same-size tiles. This is the natural alignment with the shared fold engine.
- The reload-hysteresis requirement for #172 (don't ping-pong with eviction) should be codified in the ADR-0010 amendment, not just in a code comment.
- The budget default (512 MiB) may be appropriate to revisit in the amendment — document what fraction overview tiles should occupy at the chosen pixel-size.

### Actions
- [ ] Amend camp ADR-0010 to record the chosen fix direction (decimate overview pixel size or adjust budget accounting) and update D3 / D2 accordingly.
- [ ] Update camp ADR-0013 §"camp#172 hook" once the reload path is implemented.
- [ ] Add regression test: eviction at realistic survey scale (e.g., 38 fine tiles + 45 overview tiles) leaves budget headroom after the fix rather than shedding all fine tiles.
- [ ] Add regression test: an evicted fine tile is reloaded on-demand when the viewport re-enters its area (hysteresis / budget-margin must prevent immediate re-eviction).
- [ ] Confirm nightly-regen anti-clobber / catalog-prune propagation scope (in this PR or a tracked follow-up).
