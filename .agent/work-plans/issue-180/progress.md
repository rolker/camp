---
issue: 180
---

# Issue #180 — Depth-at-cursor for GGGS store layers

## Issue Review
**Status**: complete
**When**: 2026-07-31 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #180
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Summary

Issue requests wiring GGGS store layers (Stores tab, `GggsTileLayer`) into the
`getDepth()` query path so hovering over an open bathy store reports depth in the
status bar. Currently `m_depthRasters` is populated only by legacy `DepthRaster`
objects (one per loaded chart that carries a depth band); GGGS store layers never
register, so the readout silently returns NaN over store data.

The issue defers datum conversion explicitly, which is correct scope hygiene.
The multi-level / post-#103 consideration is flagged as future work with a clear
rationale. The "label as ellipsoidal up-positive" note is helpful context.

---

### Scope Assessment

**Well-scoped?** Yes — single PR. The proposed change has three parts (point-query
method, registration/deregistration lifecycle, label text) all in the render layer
and the depth-provider list. Datum conversion is explicitly deferred to the
follow-on datum service (#288). No splitting needed.

**Right repo?** Yes — `camp` owns `GggsTileLayer`, `GggsStoreSource`,
`AutonomousVehicleProject::getDepth()`, and `ProjectView`. All changes are inside
this repo.

**Dependencies**: The S-102 reference store (rolker/unh_marine_autonomy#278) is
the suggested test target — it must be importable to verify the readout, but the
feature itself does not depend on that PR landing. Issue #103 (LOD selection) is
flagged as a future consideration; the issue correctly scopes the query to the
finest available tile regardless of rendered LOD, which requires no dependency on
#103.

---

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Only what's needed | OK | Datum conversion and LOD interaction explicitly deferred; narrow scope |
| Improve incrementally | OK | One concrete gap (no store depth readout) closed without redesigning the depth model |
| A change includes its consequences | Watch | Deregister-on-close must be handled symmetrically with registration; lifecycle should be tested or reviewed carefully |
| Capture decisions, not just implementations | Watch | The precedence choice (stores first, load-order within) and the point-query strategy (finest tile regardless of LOD) are design decisions worth recording in the PR description or an ADR addendum |
| Human control and transparency | OK | Labeling the value as ellipsoidal up-positive (not chart datum depth) is the right transparency measure until the datum service exists |
| Test what breaks | Watch | No test mention in the issue; `getDepth()` is queried on mouse move so a unit test or integration note would reduce regression risk |

---

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| camp ADR-0003 (Depth-layer tree) | Yes | `m_depthRasters` is the current depth-provider list; ADR-0003 §2 specifies `getDepth()` walks enabled depth layers in tree order. The AVP header comment reads "stage 4 will front it with a tree." Extending `m_depthRasters` with GGGS stores is consistent with the interim model, but the plan should note how the extension fits (or will be subsumed by) the eventual depth-tree. |
| camp ADR-0005 (Catalog browser + flat layers) | Yes | `GggsTileLayer` is a flat display layer produced by `GggsStoreSource`. Registration in the depth-provider walk must respect the display-layer lifecycle (created by catalog selection, removed by the Layers tab Remove action). The deregister path for `GggsStoreSource`-spawned layers may need the same care as the `GggsStores/roots` persistence path. |
| camp ADR-0007 (RasterFieldSource) | Partially | `GggsTile` already holds geographic extent + pixel data per ADR-0007. A point-query reuses that data without changing the render abstraction, so ADR-0007's contract is untouched. Worth noting if the point-query is added as a method on `GggsTileLayer` vs. directly on `GggsTile`. |
| camp ADR-0001 (TopicBridge/executor) | No | No ROS callbacks involved |

---

### Consequences

- `AutonomousVehicleProject::getDepth()` / `m_depthRasters` will need a parallel
  mechanism (or extension) for store-layer depth providers. If they share
  `m_depthRasters`, `DepthRaster` (which wraps chart rasters) and the GGGS
  store provider need a common interface — or a separate list with the same walk.
- The label "Depth:" in `ProjectView::mouseMoveEvent` may need to distinguish
  store-reported values from chart-reported values (ellipsoidal height vs. chart
  datum depth), at least in a tooltip or secondary label, until the datum service
  exists.
- Remove / close action on the Stores tab must deregister the depth provider;
  this should be tested to avoid a dangling pointer or stale readout.

### Actions
- [ ] In the plan, clarify whether GGGS store providers share `m_depthRasters` with `DepthRaster` objects or use a separate list; document the precedence rule (stores first).
- [ ] Confirm deregister-on-close is handled in both the Layers-tab Remove path and any app-shutdown teardown.
- [ ] Consider an ADR addendum or PR-description note on the precedence decision and the "finest tile regardless of LOD" query strategy, to satisfy "Capture decisions."
- [ ] Identify an automated test or a manual test protocol for the readout (unit test on `getDepth()` with a mock GGGS tile, or a test note in the PR).

## Plan Authored
**Status**: complete
**When**: 2026-07-31 10:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-180/plan.md` at `4b4af74`
**Branch**: feature/issue-180 at `4b4af74`
**Phases**: single

### Open questions
- [ ] `sampleAt()` opens the GDAL file per call; on network-mounted stores this may lag. Add a debounce or cursor-stop trigger now, or treat as follow-on if latency is observed?
- [ ] When both a GGGS store and a chart raster cover the cursor, the plan shows both labels ("Elev: X (ellipsoid)" and "Depth: Y"). Preferred UX: show both, or suppress chart depth when a store value is present?

## Plan Review
**Status**: complete
**When**: 2026-07-31 18:05 +00:00
**By**: Claude Code Agent (Claude Opus) (in-context — author self-review)

**Plan**: `.agent/work-plans/issue-180/plan.md` at `4b4af74`
**PR**: PR-less (`--issue` / file-path mode; no draft PR — `gh` unauthenticated in this environment)
**Verdict**: approve-with-suggestions

### Findings
- [ ] (must-fix) `sampleAt()` "apply the tile's NoData mask" relies on `has_nodata_`/`nodata_`, which are populated only by `loadPixels()` (constructor defers them — `gggs_tile.h:30`; confirmed `test_gggs_tile.cpp:208`). Since `getElevation()` queries the finest-level tile regardless of rendered LOD, it may sample a tile that was never painted → `has_nodata_==false` → the NoData sentinel is returned as a real elevation. Query the band's NoData directly via GDAL in `sampleAt()`, and add a NoData-on-unloaded-tile test — `plan.md:29`
- [ ] (suggestion) Test coverage stops at `GggsTile::sampleAt()`. The riskier new logic — `getElevation()`'s extent filter, `<level>_<row>_<col>` basename parse, descending-level sort, and `getStoreElevation()`'s dynamic-cast walk — is untested. Add a multi-tile `getElevation()` test (overlapping levels: finest covering tile wins; out-of-extent → NaN), or at least a manual protocol in the PR — `plan.md:51`
- [ ] (suggestion) `getStoreElevation()` walks all `GggsTileLayer`s regardless of the operator's Layers-tab enable/disable, whereas ADR-0003 §2 specifies `getDepth` walks *enabled* depth layers. (Current `getDepth`/`m_depthRasters` also ignores visibility — `autonomousvehicleproject.cpp:381` checks only `depthValid()` — so this matches today's behavior.) State the decision explicitly: should a hidden store still report elevation at the cursor? — `plan.md:39`
- [ ] (suggestion) `sampleAt()` opens the GeoTIFF per call and `mouseMoveEvent` runs on the GUI thread → a synchronous GDAL open per cursor move (the "safe on any thread" property isn't exploited; the call chain is GUI-thread). Deferring a debounce/cache is fine, but commit to defer-and-revisit-if-observed rather than leaving Open Question #1 fully open — it directly affects the operator UX this issue targets — `plan.md:99`

Non-blocking: `gggs_tile_layer.h` will need a `<QGeoCoordinate>` include for the new `getElevation(const QGeoCoordinate&)` signature (trivial implementation detail).

Strengths worth noting: file targeting is accurate (all 8 files verified to exist with the described members; integration point confirmed exact at `projectview.cpp:212`). The dynamic `topLevelLayers()->childMapItems()` walk (no stored pointer list) elegantly resolves the deregister/dangling-pointer lifecycle concern the Issue Review flagged, addressing its Actions 1–2. The separate `getStoreElevation()` path plus the distinct "Elev: (ellipsoid)" label is a defensible interim reading of ADR-0003 §2 that keeps ellipsoidal height and chart-datum depth semantically distinct until the datum service (#288).
