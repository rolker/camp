# Plan: BlueTopo WMTS preset: QNetworkReply::ProtocolInvalidOperationError (bare zoom index in TileMatrix URL)

## Issue

https://github.com/rolker/camp/issues/178

## Context

`TileLayout::getUrl()` (`src/camp_map/map_tiles/tile_layout.cpp:36`) substitutes the bare
integer zoom index (`std::to_string(address.zoomLevel())`) for `{TileMatrix}`. GeoServer GWC
(nowCOAST) names its tile matrices `<gridset>:<z>` (e.g., `EPSG:3857:1`), so every tile
request returns HTTP 400 with `InvalidParameterValue: Unknown TILEMATRIX 1`.

The fix is already latent: `TileLayout::ZoomLevel::id` holds the correct identifier string from
the WMTS capabilities XML (set in `wmts/capabilities.cpp:109`). For OSM/XYZ layers
`level.id` is set to `std::to_string(zoom_level)` (`map_tiles/osm.cpp:27`), so bare-numeric
servers keep working unchanged after the fix.

## Approach

1. **Fix `getUrl()` TileMatrix substitution** — replace the bare numeric lookup with the
   declared zoom-level identifier string, falling back to the numeric index if `id` is empty.
2. **Add a unit test** — add a new test file covering `TileLayout::getUrl()` with both a
   bare-numeric `id` (OSM/NOAA ArcGIS) and a `<gridset>:<z>` `id` (GeoServer GWC/BlueTopo)
   to prevent this class of regression.
3. **Register the test in CMakeLists.txt**.

## Files to Change

| File | Change |
|------|--------|
| `src/camp_map/map_tiles/tile_layout.cpp` | Line 36: use `zoom_levels[address.zoomLevel()].id` for TileMatrix, fallback to numeric index if `id` is empty |
| `test/test_tile_matrix_url.cpp` | New: two test cases — bare-numeric id and `EPSG:3857:N` id |
| `CMakeLists.txt` | Register `test_tile_matrix_url` with `ament_add_gtest` |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | One-line fix plus one focused test; no new infrastructure |
| A change includes its consequences | Test covers both bare-numeric and GeoServer id paths; both BlueTopo presets covered by the same code path |
| Test what breaks | New test directly catches the regression; named after the failure mode |
| Improve incrementally | Self-contained single-PR fix |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0001 (Adopt ADRs) | No | Bug fix, not a design decision |
| ADR-0008 (ROS 2 conventions) | No | C++ only, no package/launch/license changes |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `getUrl()` TileMatrix key | Test for both id shapes | Yes — step 2 |
| `getUrl()` TileMatrix key | NOAA charts preset (bare-numeric) still passes | Yes — covered by fallback + test |

## Open Questions

- [ ] No open questions — plan is review-plan-ready.

## Estimated Scope

Single PR.
