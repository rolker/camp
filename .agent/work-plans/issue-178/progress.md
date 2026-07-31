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

## Plan Authored
**Status**: complete
**When**: 2026-07-31 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-178/plan.md` at `d2d15a6`
**Branch**: feature/issue-178 at `d2d15a6`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-07-31 17:23 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-178/plan.md` at `d2d15a6`
**PR**: PR-less (dispatched via issue #178 in layer worktree)
**Verdict**: approve-with-suggestions

Verified all code claims against source: `tile_layout.cpp:36` uses the bare
numeric zoom index for `TileMatrix` (the bug); `capabilities.cpp:109` already
populates `ZoomLevel::id` from the WMTS `<gridset>:<z>` identifier; `osm.cpp:27`
sets `id` to the numeric string so bare-numeric servers stay correct after the
fix. The `zoom_levels[address.zoomLevel()].id` lookup matches the existing
WMS_BBOX branch pattern (`tile_layout.cpp:47`) and won't regress the WMS path
(which never uses the `TileMatrix` key).

### Findings
- [ ] (suggestion) Existing `test/test_wms_url_generation.cpp` already unit-tests `TileLayout::getUrl()`; plan step 2 and the review-issue premise ("no URL generation tests exist") missed it — reconcile the new TileMatrix cases with that file (extend it, or justify a separate `test_tile_matrix_url.cpp`) — `plan.md:24`, `plan.md:33`

Independence note: dispatched as a fresh-context Opus sub-agent; the `## Plan
Authored` entry was written by a Sonnet instance. Treated as an independent
review (the mechanical name-match with the shared "Claude Code Agent" identity
would false-positive as self-review; the differing model and fresh context make
this a genuine second opinion), so no author-self-review annotation applied.

## Implementation
**Status**: complete
**When**: 2026-07-31 00:00 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-178/plan.md` (synced this phase; original at `d2d15a6`)
**Branch**: feature/issue-178
**Commits**:
- `503d4af` fix(map_tiles): use declared TileMatrix id for WMTS tile URLs
- `016c50f` test(map_tiles): cover WMTS TileMatrix id substitution
- `f9ede27` docs(plan): sync #178 test step with existing URL-generation suite

### What changed
- **Fix** — `src/camp_map/map_tiles/tile_layout.cpp`: the `{TileMatrix}`
  substitution in `getUrl()` now emits `zoom_levels[address.zoomLevel()].id`
  (the identifier parsed from the WMTS `<ows:Identifier>` at
  `capabilities.cpp:109`), falling back to `std::to_string(address.zoomLevel())`
  when `id` is empty. GeoServer GWC (BlueTopo/nowCOAST) names its tile matrices
  `<gridset>:<z>` (e.g. `EPSG:3857:3`), so the previous bare numeric produced
  HTTP 400 (`Unknown TILEMATRIX`). Bare-numeric servers (OSM/XYZ via
  `osm.cpp:27`, NOAA ArcGIS) keep working via the id-or-fallback, both of which
  yield the plain index.
- **Tests** — per the plan-checkpoint operator decision, extended the existing
  `test/test_wms_url_generation.cpp` (rather than adding a new file + CMake
  registration) with three `TileMatrix` cases: `TileMatrixUsesBareNumericIdForOsm`
  (backward-compat, byte-for-byte), `TileMatrixUsesGridsetPrefixedId` (the #178
  fix, `EPSG:3857:<z>`), and `TileMatrixFallsBackToNumericWhenIdEmpty` (empty-id
  fallback). No CMake change — the target `test_wms_url_generation` is already
  registered (`CMakeLists.txt:573`).
- **Plan sync** — corrected plan step 2 and the Files-to-Change table to reflect
  extending the existing suite, and noted the review-issue "no URL generation
  tests exist" premise was wrong (the file predates this work).

### Verification
- Multi-layer camp build/test is **unavailable in this container** (the lower
  layers — `underlay_ws`/`core_ws`/`platforms_ws`/etc. — have no `install/`
  symlink targets, so `./ui_ws/build.sh camp` cannot resolve camp's ROS
  dependencies). The host runs `./ui_ws/build.sh camp && ./ui_ws/test.sh camp`
  before push.
- As a standalone check the URL-generation suite (pure C++/Qt5, no ROS) was
  compiled and run directly (`g++ -std=c++17` against `tile_layout.cpp`,
  `tile_address.cpp`, `osm.cpp`, `wms.cpp`, `web_mercator.cpp` + Qt5
  Core/Gui/Positioning + gtest): **all 7 tests pass** (4 pre-existing + 3 new).
- **Counterfactual** — recompiling the new tests against the pre-fix
  `tile_layout.cpp` (`503d4af~1`) fails exactly `TileMatrixUsesGridsetPrefixedId`
  (got `.../3/...`, expected `.../EPSG:3857:3/...`) while the two backward-compat
  cases still pass, confirming the test catches the regression and the fix
  preserves bare-numeric behavior.

### Notes for next phase
- No push / PR / GitHub actions performed (per handoff contract).
- Suggested next step: `review-code` on the pre-push diff, then the host build/test.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-31 17:41 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-178 at `b0e2f2d`
**Mode**: pre-push
**Depth**: Standard (reason: small diff, promoted from Light — networked URL path + backward-compat invariant + work plan present)
**Must-fix**: 0 | **Suggestions**: 2
**Round**: 1 | **Ship**: recommended — no must-fix findings; fix is correct, minimal, and counterfactual-verified, backward-compat invariant holds

Specialists: Static Analysis (cppcheck; cpplint unavailable), Governance, Plan Drift,
Claude Adversarial ×2 (Lens A logic + Lens B systemic). Local Adversarial off
(--no-local, workspace#590). Copilot off (default). No plan drift; diff matches
plan file-for-file. Two independent adversarial passes agree: no must-fix.

### Findings
- [x] (suggestion) Server-controlled TileMatrix `id` (raw WMTS `<Identifier>`) concatenated into URL without percent-encoding — widens injection surface; low severity, Qt-mitigated, consistent with existing `id`/`Style`/`TileMatrixSet` insertion — `src/camp_map/map_tiles/tile_layout.cpp:42` (fixed in `getUrl()`; siblings are upstream — see Implementation note)
- [x] (suggestion) New tests use the OSM stand-in (id==index) and don't exercise the real WMTS parse path where id≠index; a mismatched-id case would assert id-over-index more directly — `test/test_wms_url_generation.cpp:106`

**Operator decision (2026-07-31, run-issue publish checkpoint)**: fix BOTH
suggestions before publishing. Suggestion 1: percent-encode the
server-controlled WMTS substitution values in `getUrl()` — the TileMatrix id
and its sibling insertions (layer id / Style / TileMatrixSet) for consistency,
since the finding notes the same pattern on all of them; do not expand beyond
`getUrl()`. Suggestion 2: add a mismatched-id test case (id ≠ zoom index) that
asserts id-over-index directly. Then a fast re-confirm review round precedes
the publish.

## Implementation
**Status**: complete
**When**: 2026-07-31 17:53 +00:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-178 at `3d813e5`
**Addressed**: `## Local Review (Pre-Push)` (2026-07-31 17:41 +00:00, branch `feature/issue-178` at `b0e2f2d`), per the operator decision on its R1 suggestions
**Commits**:
- `b8f941c` fix(map_tiles): percent-encode server-controlled WMTS TileMatrix id
- `3d813e5` test(map_tiles): assert id-over-index and TileMatrix id encoding

### Actions
- [x] (suggestion 1) Percent-encode the server-controlled TileMatrix `id` in `getUrl()` — `src/camp_map/map_tiles/tile_layout.cpp:39-51`
- [x] (suggestion 2) Add a mismatched-id test (id ≠ zoom index) asserting id-over-index directly — `test/test_wms_url_generation.cpp:136`

### What changed
- **Fix** — `tile_layout.cpp`: the `{TileMatrix}` substitution now emits
  `QUrl::toPercentEncoding(id, ":")` instead of the raw `id`. `id` is a raw
  server-controlled string (the WMTS `<ows:Identifier>`), so a hostile
  capabilities document can no longer inject path/query/fragment delimiters
  (`/`, `?`, `#`, whitespace, …) into the tile URL. `':'` is **excluded** from
  encoding because it is a legal path-segment character and is load-bearing for
  GeoServer GWC ids (`EPSG:3857:<z>`) — encoding it would re-break the #178 fix.
  The bare-numeric fallback (`std::to_string(zoomLevel())`) is digits-only, so it
  passes through unchanged (byte-for-byte backward compatible). `TileRow`/
  `TileCol` are locally-computed integers, not server data, so they are left
  as-is (encoding digits is a no-op anyway).
- **Tests** — `test_wms_url_generation.cpp`: two new cases the OSM stand-in
  (id == index) could not exercise. `TileMatrixPrefersIdOverIndexWhenMismatched`
  sets `id = "EPSG:3857:99"` at zoom 3 and asserts the URL carries `…:99…` (not
  the bare index `3`), proving id-over-index directly. `TileMatrixIdIsPercentEncoded`
  sets `id = "EPSG:3857:3/x"` and asserts the `/` becomes `%2F` while the colons
  survive, covering the new encoding.

### Scope note — sibling insertions (layer id / Style / TileMatrixSet)
The operator decision named these siblings "for consistency" but also scoped the
change to `getUrl()` ("do not expand beyond `getUrl()`"). On inspection these two
constraints only partly overlap: **`getUrl()` substitutes exactly one
server-controlled value — the TileMatrix `id`** (that is what finding 1 cites,
`tile_layout.cpp:42`). The layer id, Style, and TileMatrixSet are **not**
substituted in `getUrl()`; they are baked into `url_static_parts` upstream in
`wmts::Capabilities::getLayout()` (`src/camp_map/wmts/capabilities.cpp:77-99`) and
reach `getUrl()` already fused with the literal URL template, so `getUrl()` cannot
selectively encode them without either (a) touching `capabilities.cpp` — outside
the `getUrl()` scope the operator set — or (b) percent-encoding whole static parts,
which would corrupt the template delimiters (`?`, `&`, `=`, `/`). I therefore
encoded the one value in `getUrl()`'s remit (the TileMatrix id) and did **not**
expand into `capabilities.cpp`. **If the operator wants the sibling values encoded
too, that is a separate, one-line-each change at their insertion point in
`getLayout()`** — flagged here for the re-review / next checkpoint to decide.

### Verification
- Same constraint as the prior phase: the multi-layer camp ROS build/test is
  **unavailable in this container** (lower layers have no `install/` targets), so
  `./ui_ws/build.sh camp` cannot resolve camp's ROS deps. The host runs the full
  `build.sh camp && test.sh camp` before push.
- Standalone check (as before): compiled the URL-generation suite directly
  (`g++ -std=c++17` against `tile_layout.cpp`, `tile_address.cpp`, `osm.cpp`,
  `wms.cpp`, `web_mercator.cpp` + Qt5 Core/Gui/Positioning + gtest — `QUrl` is in
  QtCore, already linked): **all 9 tests pass** (7 prior + 2 new).
- Counterfactual: the new `TileMatrixIdIsPercentEncoded` case is a genuine guard —
  without the encoding the raw `/` splices through (`…/EPSG:3857:3/x/…`), so the
  test fails against the pre-fix `getUrl()`; the backward-compat cases
  (`…BareNumericIdForOsm`, `…FallsBackToNumericWhenIdEmpty`) stay byte-for-byte
  identical, confirming the encoding is transparent to legitimate ids.

### Notes for next phase
- No push / PR / GitHub actions performed (per handoff contract).
- Next: `review-code` (re-review) on the pre-push diff to confirm the two R1
  findings are genuinely resolved, then the host build/test. The re-reviewer
  should also weigh in on the sibling-encoding scope note above.
