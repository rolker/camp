---
issue: 121
---

# Issue #121 — Unified raster render abstraction + CAMP live tile cache (anti-entropy)

## Issue Review
**Status**: complete
**When**: 2026-06-28 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #121
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: needs-splitting

### Actions
- [ ] Split into two PRs: Part A (RasterFieldSource interface + adapters) and Part B (live tile cache) — Part B depends on #230 (anti-entropy transport) and #44/#68 (discover/opt-in); verify those are merged before starting Part B.
- [ ] Verify that the `RasterFieldSource` interface doesn't merely rename existing `GggsTileLayer` internals — the abstraction should emerge from needing a second implementation (live cache), not be introduced speculatively ahead of it. Consider whether the interface belongs in the same PR as Part A or only in Part B's PR.
- [ ] Write a camp project ADR for Part B's persistence contract: in-memory render + write-through to disk + anti-entropy reconcile + "display-grade preview vs. data of record" distinction. This is a significant design decision for the project.
- [ ] Clarify how the live cache layer presents to the operator: does it appear in the Layers tree? How is its "live/preview" status visually distinguished? Align with ADR-0005's browse/compose split.
- [ ] For Part B acceptance: add a test or documented scenario for the simulated-downtime gap (request + prune) that can run without a live boat connection.

## Plan Authored
**Status**: complete
**When**: 2026-06-28 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-121/plan.md` at `edf9946`
**Branch**: feature/issue-121 at `edf9946`
**Phases**: single

### Open questions
- [ ] Discover vs. opt-in: camp#44/#68 not yet implemented; auto-spawn (GridManager pattern) proposed — confirm with operator before implementing.
- [ ] Multi-band write-through: which bands to persist (all received vs. depth-only)? Propose: all received bands.
- [ ] Cache size limits: no built-in eviction beyond prune-on-absence; flag if operator expects a bounded cache.

## Plan Review
**Status**: complete
**When**: 2026-06-28 17:55 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-121/plan.md` at `edf9946`
**PR**: PR-less (--issue mode)
**Verdict**: approve-with-suggestions

The plan is well-grounded in the actual codebase (reconciler API, `ros::Layer`/`Node`,
`GggsTile::loadPixels`, manager wiring, QoS choices all verified) and addresses every
review-issue finding. The one must-fix is a completeness gap: the Files-to-Change table
omits two files (`package.xml`, `item_types.h`) that the plan's own Consequences table
says must change. Suggestions concern threading clarity, the discover-vs-opt-in blocker,
and minor convention/version-semantics points.

### Findings
- [ ] (must-fix) Files-to-Change table omits `package.xml` (needs `<depend>marine_tiled_raster_store</depend>` — confirmed absent) and `item_types.h` (needs a new `SonarLiveCacheLayerType` in its `enum ItemType` — the Consequences row says "in sonar_live_cache_layer.h" but the enum lives in `src/camp_map/map/item_types.h`). Reconcile the two tables. — `plan.md:214-227, 252-257`
- [ ] (suggestion) Consequences row "node.h forward-declares the manager" is unnecessary: existing `MarkersManager`/`GridManager` are created in `node.cpp` (`new XManager(this)`) and are not forward-declared in `node.h`. Drop the row or confirm it's actually needed. — `plan.md:256`
- [ ] (suggestion) Resolve Open Question #1 (auto-spawn vs. operator opt-in) with the operator *before* implementation — it determines `SonarLiveCacheManager`'s spawn behavior, a structural decision, not a detail to defer. Also note this bypasses ADR-0005 §2's generic `CatalogSource` seam (which explicitly anticipated topic-discovery consumers like camp#44/#68); the deferral is defensible but should be stated as a conscious ADR-0005 tension in ADR-0006. — `plan.md:182, 261`
- [ ] (suggestion) `TileCatalogReconciler` is explicitly **not thread-safe** (tile_catalog.hpp:46). The plan should state that `reconcile()`/`markHave()`/`drop()`/`applyPatch()` all run on a single thread (warm-load constructor + ROS callbacks marshalled to the GUI thread via queued signal per ADR-0001), so the ROS callback must only copy+emit, never touch `reconciler_` directly. — `plan.md:122-135, 245`
- [ ] (suggestion) Specify version semantics consistency: newest-wins compares `header.stamp` against `versionOf()` (`TileVersion` = int64), and prune is gated on catalog `generation_time` (also int64). State the single conversion (e.g. stamp→nanoseconds) used for `markHave`, the stale check, and the timestamp gate so they're comparable. — `plan.md:128-130, 211-213`
- [ ] (suggestion) New manager file placement: existing managers live in `ros/` subdirectories (`ros/grids/`, `ros/markers/`, `ros/geometry/`); the plan puts `sonar_live_cache_manager.{h,cpp}` directly in `ros/`. Consider a subdir for consistency (cosmetic). — `plan.md:161, 223-224`
