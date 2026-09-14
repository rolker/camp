---
issue: 22
---

# Issue #22 — Import generic vector data

## Issue Review
**Status**: complete
**When**: 2026-09-14 10:38 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #22
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: needs-more-detail

### Actions
- [ ] Reconcile with the existing `VectorDataset` / `camp::vector::parseVectorLayers` code (`src/camp/vector/vectordataset.{h,cpp}`, `src/camp/vector/vector_parse.{h,cpp}`): this already opens arbitrary OGR vector files (GeoJSON, shapefile, etc.), RAII-closes the GDAL handle (issue #152), and builds Point/LineString/Polygon items — but into the editable MissionItem/Group tree, not a `map::Layer`. The issue's Related section omits it entirely. State explicitly whether #22 (a) reuses/extends `parseVectorLayers` for the new read-only layer (recommended — avoids re-deriving OGR iteration; "only what's needed"), (b) supersedes `VectorDataset`, or (c) is a deliberately separate concept (editable-import vs. read-only-display) — and why.
- [ ] `ParsedGeometry`/`ParsedLayer` (`vector_parse.h`) currently carry **no attribute fields at all** — attributes are dropped during parsing. The issue's "Must have" list (attribute-driven styling, click-for-attributes) requires extending this struct/parse path; scope that work explicitly in the plan rather than assuming "just wire up styling to existing data."
- [ ] `VectorDataset::write()` persists the source filename but `VectorDataset::read()` is an empty stub — the persisted file is never restored on project reload today. The new layer's "Must have: persist by path + style" is exactly the feature quietly broken in the sibling code; note this so a reviewer doesn't assume the existing persistence path can be copied as-is.
- [ ] The issue asks to both "mirror RasterLayer" (a single async-loaded Layer that paints one composited surface, per ADR-0007) and support "click a feature → its attributes." RasterLayer has no per-feature hit-testing; `VectorDataset`'s approach (one QGraphicsItem child per feature) gets Qt's native item-picking for free but doesn't fit a single async field-source layer. Pick and state the architecture (custom hit-testing in one Layer vs. per-feature child items) during plan-task — the issue is silent on this fork and it drives the whole implementation shape.
- [ ] The "reusable colormap facility (#63)" reference is stale: `camp::map::ColorMap` (the facility #63 originally scoped) was deleted in camp#141/PR#144 (ADR-0008) and replaced by the external `marine_colormap` library, already integrated in `RasterLayer` and `ros/grids/grid_map.cpp` (`marine_colormap::find_palette` + `palette->sample()`). This is good news, not a blocker — the real facility already exists and is in active use — but #22 should cite `marine_colormap` / ADR-0008 directly, and #63 (still open) looks like a stale tracking issue worth closing as superseded in a follow-up.
- [ ] No tests are mentioned in the issue's scope, despite this area's strong precedent (`test_raster_layer_gdal_cleanup.cpp`, `test_color_map.cpp`, `test_catalog_source.cpp`) and the "Test what breaks" principle. Plan-task should scope: attribute-parse extension tests, the thread-safe-teardown test mirroring `ros/geometry/polygon.h`'s #213 pattern (join-worker-in-destructor), and colour/size-by-field mapping tests.
- [ ] Consider phasing: load+render+persist+teardown as an MVP slice, attribute-driven styling + click-popup as a follow-on — the combined "Must have" list is large for one PR (six distinct capabilities). Not blocking; a recommendation for plan-task.
- [ ] ADR-0003's depth-tree / Z-order model and ADR-0011's viewport-clip convention should be checked against the new layer's paint path once the architecture fork above is resolved — flag if per-feature rendering needs viewport clipping for larger vector sets (the two cited test datasets are tiny — 33 and 7 features — but the issue frames this as general-purpose, and future consumers like survey-index footprints could be much larger).


## Plan Authored
**Status**: complete
**When**: 2026-09-14 10:48 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-22/plan.md` at `75182fc`
**Branch**: feature/issue-22 at `75182fc`
**Phases**: single

### Open questions
- [ ] Persistence mechanism: plan resolves "persist in the project file" (issue wording) to ADR-0003 §4's app-state (QSettings) pattern like RasterLayer/backgrounds, since VectorLayer is a Layers-tab layer, not a mission item — the operator's six decisions didn't explicitly settle this; confirm at review-plan.
- [ ] Menu/action wiring location ("Open vector layer") to be confirmed against the existing "Open background" action during implementation.
- [ ] `ParsedGeometry::attributes` container type (QMap<QString, QVariant> vs. ordered vector-of-pairs) — defaulted to QMap; revisit only if display order matters.
