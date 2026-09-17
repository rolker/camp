# Plan: Import generic vector data

## Issue

https://github.com/rolker/camp/issues/22

## Revision history

**Rev 18** (2026-09-17) — **round-6 PR triage fixes (Copilot R7).** No design
change; four should-fix, two suggestions and one nit, all new ground.

- **The startup restore decides its whole state before opening anything, and
  persists once.** `openVectorLayer()` ends in `persistVectorLayers()`, which
  rewrites `vectorLayers/files` from the restored order, the unavailable list and
  the tracked layers — so while those lists were built INSIDE the open loop, each
  per-layer write put a list truncated at the current entry on disk, and an exit
  or a worker-side crash mid restore (the opens are asynchronous parses that
  outlive the loop) silently forgot every later layer: camp#90/#117 from the
  other direction. The classification moves into
  `camp::vector::planVectorLayerRestore()` — a harness-visible seam that opens
  nothing and writes nothing — and per-layer persistence is suppressed for the
  duration of the loop.
- **A standalone point the parse drops is reported as a skipped item.** The
  round-17 fix reached lines and polygons; the point branch was the one case it
  did not, so a file whose points all fall outside their projection's inverse
  domain still produced the bare "(no items)" — an empty-FILE verdict.
  `ParseDiagnostics::point_geometries_dropped` is its own counter because
  `points_dropped` also counts bad VERTICES of lines and polygons that were drawn
  in full.
- **The parse result is read by reference, and the future is released.** Qt
  5.15's `QFutureWatcher::result()` returns by value, so `loadFinished()` built a
  second full copy of every coordinate and attribute map while the future's store
  still held the first — and the watcher, a never-reset member, RETAINED that
  first copy for the layer's whole lifetime, undercutting the memory bound
  `kMaxFeatureItems` is documented to give. Read through the future's const
  iterator and clear the watcher's future at the end of the slot (guarded against
  the re-entrant `finished()` that clearing can deliver).
- **The persisted colormap name is case-folded and registry-validated**, falling
  back to grayscale — the `RasterLayer::readSettings()` rule (camp#141) this layer
  did not share. An unknown name RENDERED as grayscale while `colormap_` kept the
  invalid spelling, so no palette showed as checked and `writeSettings()` wrote
  the bad value back every session.
- **Size by offers point-carried numeric fields only.** `applyStyle()` folds the
  size range over points alone, so a field only lines or polygons carry left every
  marker at the default radius while the action showed as checked and the choice
  persisted. Color by keeps the full list; a set-but-unusable size field is
  annotated `(no numbers on any point)`.
- **The geometry cap is not charged for geometry with no placeable coordinate.** A
  layer with NO spatial reference takes the untransformed branch, so projected
  metres arrive as a well-formed coordinate that is nowhere on earth: nothing
  failed, nothing was empty, and the geometry was emitted, charged, and rejected
  one step later — on a multi-layer container an SRS-less layer could exhaust the
  cap before the good layer behind it was read. The parser now applies the display
  layer's own `hasPlaceableCoordinate()` test (moved beside the parse, so one rule
  serves both) and drops-and-counts instead. Consequence, recorded in ADR-0016: a
  `.prj`-less file is now read to its last feature rather than stopping at the
  cap. The cap `qWarning()` no longer overstates what is shown.
- **`kMaxFeatureItems` names the limit it does not bound**, pointing at the
  ADR-0016 D11 deferral (a single ring of millions of vertices) rather than
  leaving the claim to read as unqualified. The deferral itself stands.
- Tests: the restore plan is complete (and writes nothing) before the first open;
  a point-only file outside its projection domain does not report the empty-file
  verdict; a bogus and a miscased persisted palette round-trip to grayscale and to
  the registry spelling; Size by omits a line-only numeric field and annotates one
  that is set; and the round-4 capped-but-empty test is turned around to assert
  what the cap now does NOT charge.

**Rev 17** (2026-09-17) — **round-8 pre-push review fixes.** No design change;
one must-fix and eight suggestions, all consequences of the rev-16 fixes.

- **A geometry the PARSE drops is reported as a skipped item.** Rev 16 stopped
  emitting a line or polygon whose exterior holds no usable vertex — and with it
  went the only operator-visible report of that class, because such a geometry
  never reaches `VectorLayer` to be counted there: the Layers tab printed the bare
  "(no items)", the verdict an EMPTY FILE gets, for a file whose features exist
  and fall outside their projection's inverse domain. The skips have their own
  `ParseDiagnostics` counter, logged per layer like its siblings, and
  `loadFinished()` folds it — and `polygons_without_exterior_ring`, which had the
  same gap — into the skipped count the status reports.
- **`aborted` and `geometry_cap_reached` are never both set.** The rev-16 abort
  re-check inside the cap return could raise the abort on a result already flagged
  cap-reached, the one combination `ParseDiagnostics` says cannot happen. The cap
  flag is cleared there; the header now says how much an aborted result holds is
  unspecified (the caller discards it whole), and the abort-poll sweep asserts the
  combination never appears.
- **The cap path's abort window does not log the per-layer summaries**, which the
  lambda's own contract reserves for a result the caller keeps.
- **The promotion gate compares the purged list's CONTENTS**, not its length — the
  length form was sound only while `withoutVectorLayerFile()` does not
  de-duplicate, unlike both its siblings.
- **`withVectorLayerFilePromoted()` stops resolving at the entry it rewrites.** It
  walked the whole restored order with a GUI-thread stat per entry, at the one
  moment those paths are most likely dead mounts; the exact-string duplicate
  collapse is unchanged, and the cost is now stated for this function rather than
  borrowed from its smaller sibling.
- **What the cap stopped bounding is recorded.** Only a drawable geometry spends
  the cap, so a file whose geometries all drop out is read to its end: memory and
  abortability are unaffected, the residue is wall time on a file that shows
  nothing, and the operator is told the drop count. Written down on
  `ParseOptions::max_geometries`, `kMaxFeatureItems` and in ADR-0016's
  consequences, with why charging the cap for dropped geometry is worse.
- **The no-mouse-button guarantee is stated for the ITEM SUBTREE** in ADR-0016 D5
  and `.agents/README.md` — the item-only reading is what let the child hover
  label keep Qt's default.
- Units: the four residual "feature" spellings of the quantity rev 16 renamed to
  items (the skipped-count log line, the empty-layer status `(no items)`, and two
  ADR sentences).
- Tests: a layer-level regression that a file of parse-dropped geometries is not
  reported as empty, the new counter asserted on the all-dropped fixture, the
  undrawable polygon's hole moved OUT of domain so "the holes are never read" is
  actually proved (7 dropped points, not 11), the abort/cap exclusivity swept at
  every poll position, and the promotion's duplicate collapse.

**Rev 16** (2026-09-17) — **round-5 PR triage fixes.** No design change; seven
review findings, all in the mechanisms rev 14 and rev 15 put in place.

- **A reopened once-unavailable layer keeps its persisted SLOT.** Rev 14 purged
  the unavailable list by canonical-equivalent identity, but the restored ORDER
  kept the raw spelling a dangling symlink was remembered under while the
  reopened layer is tracked under its resolved target, so the rebuild's exact
  string match skipped the slot and the trailing append loop moved the layer to
  the END of the operator's stacking order. `withVectorLayerFilePromoted()`
  rewrites the entry in place, on the same promotion that purges the list.
- **A line or polygon whose exterior loses every vertex is not emitted and does
  not spend a geometry-cap slot.** The point branch already counted-and-dropped
  without spending; the other two pushed an empty geometry and spent anyway, so
  out-of-domain features burned cap slots later valid features needed. The
  polygon returns before walking the holes of a shape it cannot draw.
- **The cap path reports the per-layer diagnostic summaries.** The cap return sat
  above the three `qWarning` blocks, so on the one path where the operator is
  already told the read was partial, `points_dropped` and `geometries_unhandled`
  — reported nowhere else — vanished. Both consumed exits now call one lambda.
- **An abort that rises inside the cap lookahead is reported as an abort**, which
  the header always promised. The lookahead also answers "nothing remains" once
  an abort is requested, so a genuinely capped parse could otherwise have claimed
  it was read in full.
- **The hover label answers no mouse button either.** The item's
  `Qt::NoButton` guarantee (ADR-0016 D5 / camp#225) is only true by construction
  if it holds for the whole subtree, and the label is placed AT the cursor for a
  line or polygon with the item lifted above its siblings.
- **The cap is reported in ITEMS**, the quantity it counts: `max_geometries` is
  spent per emitted geometry part and one item is built per part, so a multi-part
  feature accounts for several. Header, status line, log and ADR-0016 now agree;
  wording only.
- Tests: the canonical-aware restored order (and the promotion's blast radius),
  the per-layer style QSettings round trip (a mistyped key would have lost every
  persisted style with the suite green), an all-dropped exterior under the cap,
  the cap path's summaries, an abort at EVERY poll position of a capped parse,
  and the label's no-button contract at item level plus a press over the label
  through a real view.

**Rev 15** (2026-09-15) — **round-7 pre-push review fixes.** No design change.

- **The unhandled-type NAME in the per-layer summary line is per-layer**, like
  the count beside it. `ParseDiagnostics::first_unhandled_geometry_type`
  accumulates across the whole parse, so layer 2 of a mixed dataset logged layer
  1's type. The parser now scopes the field to the layer body (an RAII guard, so
  the two early returns cannot leak the layer-scoped value) and restores the
  parse-wide first type afterwards; the field keeps its documented whole-parse
  meaning for callers.
- **The cap lookahead polls the abort predicate.** It sits on the path the
  destructor's GUI-thread join waits for, and without the poll an aborted worker
  still ran one unabortable `GetNextFeature()` per remaining layer. The answer is
  cosmetic once aborted — the result is discarded whole.
- Tests: the once-per-layer test now uses a **two-layer** fixture of two curve
  types (it fails without the scope guard), and a new two-layer cap fixture
  covers the lookahead's previously untested multi-layer branch (trailing layer
  empty vs. holding one feature).
- Docs: ADR-0016 D11 now records the "cap flag means input was ACTUALLY left
  unread" rule and its bounded lookahead, and `withoutVectorLayerFile()`'s
  declaration records the accepted cost of its per-entry GUI-thread stat.

**Rev 14** (2026-09-15) — **round-3 PR triage fixes.** No design change: the
hover label, the pan-mode arrow cursor and numeric-only ramp fields all stand as
rev 12 settled them. The plan-level behaviour changes are:

- **The unavailable-at-startup list is purged by canonical-equivalent identity**,
  not by exact string. An entry is only canonical when its file resolved at the
  time it was written, so a dangling symlink is remembered under its RAW
  spelling; once the target appears, the path in hand is the resolved target and
  `removeAll()` missed the entry, which the rebuild then wrote back forever — the
  camp#90/#117 class the mechanism exists to avoid.
  `camp::vector::withoutVectorLayerFile()` (with `canonicalVectorLayerPath()`
  moved beside it so both can be tested) does the exact and the equivalent drop.
- **A polygon must be anchored by an EXTERIOR vertex.** The anchor search fell
  back to interior rings, so a polygon whose exterior was entirely unplaceable
  but whose hole held a valid vertex was built from the hole alone — which
  `Qt::OddEvenFill` paints as solid fill, a hole drawn as a feature. It is now
  skipped and counted in the layer's `skipped` tally.
- **`geometry_cap_reached` means input was actually left unread.** It used to be
  set the moment the total reached `max_geometries`, so a file holding exactly
  that many geometries claimed "rest of file not read" about a file read in full.
  A bounded lookahead (the current feature's unread parts, one feature on this
  layer, one per remaining layer) establishes the remainder first. The capped
  note is also now said on the **no-features** status path, which a `.prj`-less
  national shapefile lands on.
- **Unhandled geometry types are logged once per layer**, with the count and the
  first type seen, instead of once per geometry — an unhandled geometry does not
  spend the geometry budget, so the cap bounded nothing on that path.
- A caller-supplied `ParseDiagnostics` is **reset** before the parser binds to
  it, so one object reused across parses cannot carry a flag or a counter
  forward. Doc corrections: a line's or polygon's label is placed where the
  cursor ENTERED and does not track it (header, call site, test name, ADR-0016
  D5), and ADR-0016 D13 now records that a dropped mid-ring vertex leaves a
  fabricated straight segment.
- New tests: the dangling-symlink purge lifecycle,
  `WithoutVectorLayerFileKeepsUnrelatedEntries` (the purge drops the file's own
  spellings and NOTHING else — the guard on resolving every entry), the
  polygon-with-a-valid-hole rejection, a capped-but-empty layer's status,
  cap-at-exactly-the-file-size (and its mid-feature counterpart),
  one-warning-per-layer for unhandled types, the diagnostics reset, and a
  projected-CRS (EPSG:32619, UTM 19N) reprojection fixture — the transform path
  was previously only ever exercised WGS84 -> WGS84.

**Rev 13** (2026-09-15) — **round-5 pre-push review fixes.** No design change:
the hover popup, the in-scene label, the pan-mode arrow and numeric-only ramp
fields all stand as rev 12 settled them.

- `VectorFeatureItem::shape()` returns a **cached** hit shape, rebuilt in the
  constructor and at every `prepareGeometryChange()` site, instead of re-stroking
  the whole path on every call — the scene calls it per mouse-move now that
  inspection is on hover.
- A **persisted colour/size field the file no longer offers** is shown in the
  Color by / Size by submenus, marked `field (no numbers)`, with `(none)` present
  to clear it. The submenus used to be built only from `numericFields()`, so such
  a setting was invisible and unclearable while being re-persisted.
- The point hover label's offset is recomputed on `setRadius()`, the hovered item
  is raised above its siblings so its label cannot be painted over, and the
  release-side pan-cursor reset is scoped to the **left** button.
- Doc corrections: `fields()` and the `test_vector_feature_item` CMake comment no
  longer describe a click-to-inspect popup or a tooltip; ADR-0016 D5 and the
  `projectview.cpp` row below no longer claim the file is byte-identical to
  `jazzy` (D16's cursor change is in it); the `.agents/README.md` cursor paragraph
  was moved out of the middle of the vector-layer paragraph.
- Deferred (pre-push suggestions, with reasons in the progress.md Implementation
  entry): caching `numericFields()`, a hover-during-layer-removal test, and the
  cppcheck `useStlAlgorithm` style hits.

**Rev 12** (2026-09-15) — **the hover popup becomes an in-scene LABEL, and pan
mode gets an ARROW cursor CAMP-wide.** Both are operator decisions taken at a
run-issue checkpoint after testing the rev-11 build in the GUI.

- Rev 11's popup was the item's ordinary Qt tooltip. Tested, the operator's
  verdict was "similar to what was existing, but not the same": CAMP's own items
  answer the cursor **instantly**. `VectorFeatureItem` now replicates
  `GeoGraphicsItem`'s mechanism — `setAcceptHoverEvents(true)`, a child
  `QGraphicsSimpleTextItem` carrying the same flag/font/brush/pen as
  `geographicsitem.cpp:16-25`, filled in `hoverEnterEvent()` and emptied in
  `hoverLeaveEvent()` — rather than inheriting it, because `camp_map` cannot
  depend on the `camp` executable. `setToolTip()` is gone, so there is no second,
  delayed popup. The label item is created on the **first hover**, not one per
  feature at load (up to 50 000 of them). ADR-0016 D5 rewritten.
- `ProjectView` shows `Qt::ArrowCursor` on the **viewport** in pan mode — after
  `setDragMode(ScrollHandDrag)` and again after `QGraphicsView::mouseReleaseEvent()`
  returns, the two places Qt installs its open hand. The closed hand during a real
  drag stays; the add-\* modes keep `Qt::CrossCursor`. Rev 11 recorded this as a
  CAMP-wide follow-on; the operator decided it instead, because the open hand's
  invisible hotspot makes every hover-answering item in CAMP hard to aim at. New
  ADR-0016 **D16**; the follow-on line is removed. `projectview.h` is unchanged,
  and this is the branch's only ProjectView change (rev 11's revert stands).
- Tests: the two tooltip-based tests are replaced by label-based ones —
  `VectorFeatureItem.HoverShowsAnInSceneLabelWithTheAttributes` (hover-enter fills
  the child label, hover-leave clears it, and there is no tooltip) and
  `VectorLayerInteraction.HoverThroughARealViewShowsTheAttributes` (rewritten
  around a synthesized buttonless `QMouseEvent(MouseMove)` on the viewport of a
  real y-flipped ScrollHandDrag view, at centre / 3 px off / clear of any
  feature). The no-mouse-button and press-falls-through tests are unchanged.
  **The cursor change has no test**: `ProjectView` is not constructible in a
  harness (the `test_mission_insertion` precedent). It is verified in the GUI.

**Rev 11** (2026-09-15) — **inspection moved from CLICK to HOVER**, an operator
decision taken at a run-issue checkpoint after the rev-10 fixes were tested in the
GUI and the layer worked ("It works!"). The remaining oddity was that the vector
layer was the only thing in CAMP that inspects on a click: `Platform` and
`AISContact` show their label on hover, `GeoGraphicsMissionItem` brightens on
hover, and nothing inspects on click. The popup is now the item's ordinary Qt
tooltip (`setToolTip()`, shown by `QGraphicsScene::helpEvent()`), and a
`VectorFeatureItem` accepts **no mouse button at all**.

What that deleted rather than added:

- `mousePressEvent`/`mouseReleaseEvent`, `viewInPanMode()` and the click/drag
  slop constant — all of which existed only to tell a click from a pan and an
  inspection from a mission-item placement.
- The `ProjectView` change this branch made for the gate (`e2a56cc`, deferring
  the switch back to pan mode until after the press was dispatched). With no gate
  it has no purpose, and it touched the press path of every add-\* mode, so
  `src/camp/projectview.cpp` is reverted and is byte-identical to `jazzy`.
- **camp#225** (a pan gesture starting on a feature does not pan) is fixed by
  construction: the press always reaches the view's ScrollHandDrag now. The host
  can close it at merge.

The hit slack stays (ADR-0016 D15) — `shape()` is what `QGraphicsScene::helpEvent()`
hit-tests too, so the tolerance serves hover for the same reason it served the
click; the constants are renamed `kHoverWidth` / `kPointHoverSlackPixels`. Tests
follow: the tooltip text, the no-mouse-button contract, a synthesized
`QHelpEvent(QEvent::ToolTip)` through a real y-flipped view at centre / 3 px /
7 px off / clear of any feature, and a press through that view leaving the scene
with no mouse grabber. ADR-0016 D5 is rewritten; D15 and the camp#225 consequence
are updated; "click to pin the popup open" and "the pan cursor is a CAMP-wide
choice" are recorded as follow-ons.

**Rev 10** (2026-09-15) — the first OPERATOR GUI TEST of the branch, on 7 real
magnetic-anomaly candidates
(`massabesic_joint_candidates.geojson`). Two of the three "must have"
behaviours did not work in the operator's hands, and neither was visible to any
test written so far because every click test bypassed the view and no test looked
at what a marker actually paints:

- **Colouring by a free-text field made the features disappear.** No feature has
  a number for `assessment`, so the range was invalid, every feature was flagged
  no-data, and the hollow no-data marker was stroked with a width-0 hairline —
  one dashed device pixel of mid grey, no fill behind it, over a chart. The
  styling menus now offer `VectorLayer::numericFields()` only, the hollow marker
  is stroked at width 2 like every other outline, and an invalid colour range
  falls back to UNSTYLED rather than marking the whole layer no-data (ADR-0016
  D6, D14). **Categorical styling is explicitly a follow-on**, not an MVP gap
  being papered over: a colour per class with a legend is a different mapping,
  not a degenerate ramp.
- **The point click target was the drawn 5 px marker**, aimed at under an
  open-hand pan cursor whose hotspot is not visible, so no tooltip was ever
  obtained. `shape()` now carries 4 px of slack (ADR-0016 D15), as a line's
  shape already carries `kClickWidth`.
- **The regression test goes through a real `QGraphicsView`** — y-flipped,
  ScrollHandDrag, synthesized `QMouseEvent` on the viewport — because everything
  between the mouse and the item is precisely what the existing tests skipped.
  All three new tests were run against the pre-fix behaviour and fail there.

**Rev 9** (2026-09-15) — the round-4 pre-push review's must-fix. Rev 8's
vertex-level poll TRUNCATES a ring, and the only thing keeping a truncated ring
off the screen is `ParseDiagnostics::aborted`; the per-feature cap check ran
first and returned from its own branch, so a feature that both crossed
`max_geometries` and carried a truncated ring reported `geometry_cap_reached`
with `aborted` unset. The abort is now checked BEFORE the cap — an aborted
result is reported as aborted and is not trimmed to exactly `max_geometries`,
being partial by construction — and the interior-ring loop breaks on the abort
flag (never on the cap, which would drop the holes of a polygon drawn in full).
"Abort latency bounded by a poll interval" is unchanged; what changes is which
flag a parse that hits both reports.

**Rev 8** (2026-09-15) — the round-4 fix pass addressed both should-fix findings
of the 2026-09-15 Integrated Review (round 2) on PR #226. The plan-level
consequences:

- **The abort poll reaches INSIDE a single ring.** Rev 7 bounded abort latency by
  one geometry; one geometry can be a ring of millions of vertices, so
  `readRing()` now takes the `ParseBudget` and polls every 1024 vertices (and
  once per ring, for a polygon of very many short interior rings). Abort latency
  is therefore bounded by a poll interval, not by the largest ring in the file
  (ADR-0016 D11).
- **`normalizedValue()` stays monotonic across an overflowing span.** A range of
  -DBL_MAX to DBL_MAX made the span +inf while `(value - min) / inf` stayed
  finite at 0.0, so the existing `!isfinite` fallback never fired and the ramp
  ordered features wrongly rather than merely compressing them. The ratio is now
  computed on halved operands when the span is not finite; the order-based
  fallback stays last.
- **Bounding the VERTEX count (as opposed to abort latency) remains deferred** to
  the ADR-0016 D11 per-attribute-bound decision — truncating a ring draws a wrong
  shape, so it is a decision, not a mechanical edit.

**Rev 7** (2026-09-15) — the round-3 fix pass (ten commits, `e2a56cc` through `71e56c0`)
addressed all ten findings of the 2026-09-14 Integrated Review on PR #226. The
plan-level consequences:

- **`ProjectView` IS touched after all**, reversing rev 4's Open Question 2 and
  the struck-out Files-to-Change row. Not for a `mouseMode` accessor — the
  feature item still reads pan mode from the view's `dragMode()` — but because
  the gating did not work: `mousePressEvent()` switched back to pan mode
  *before* forwarding the press, so the item read "pan" mid-placement and the
  attribute popup fired while the operator was placing a mission item. The
  switch is now DEFERRED until after the forward, for all six add-* modes and
  the right-button cancel alike. The coupling is recorded from both sides
  (`vector_feature_item.cpp`'s gate comment and ADR-0016 D5) because it is
  invisible from `camp_map`. Follow-up camp#225 (an item accepting the left
  press takes it from the view's pan gesture) is unchanged by this.
- **No-data is a SECOND CHANNEL, not a colour.** Step 5's "a fixed neutral
  colour distinct from any position on the active palette" was not achievable:
  grayscale is a palette the operator can pick *and* the unknown-name fallback,
  and its midpoint is that same grey. A no-data feature is now drawn with a
  DASHED outline and a HATCHED fill (a hollow marker for a point) — channels no
  palette touches, which survive colour-blind vision and a monochrome printout.
  `camp::vector::isNoData()` names the state as exactly the branch
  `colorForValue()` answers with `noDataColor()`, so the two cannot drift
  (ADR-0016 D6).
- **The cap and the abort predicate are threaded INTO the geometry recursion**
  as a `ParseBudget`, one step beyond rev 6's "carried into the parse". Rev 6
  stopped the parse between features; a single `MultiPolygon` or nested
  `GeometryCollection` could still materialise unboundedly and delay the
  destructor's join. The budget is checked before every part and spent on every
  emission, so abort latency is bounded by one geometry rather than one feature —
  and, since rev 8, by a 1024-vertex poll interval inside one ring rather than by
  the whole ring (ADR-0016 D11).
- **Polar latitudes are CLAMPED, not dropped** (operator decision at the
  round-3 checkpoint). Every conversion in `vector_feature_item.cpp` — ring
  vertices *and* the item's anchor position — goes through `placeableToMap()`,
  which clamps to `web_mercator::maximum_latitude` (85.0511°) before projecting.
  The anchor mattered as much as the vertices: clamping only the path would have
  left the item's position 2.4e8 m out behind an innocent-looking local path
  (ADR-0016 D13).
- **One item deferred to a follow-up**: persisted layer ORDER does not follow a
  Layers-tab drag. It is not the small change it looks: the live order lives in
  the map's child list while an unavailable-at-startup entry has no map item and
  holds its slot only from the restored order, so the two sequences must be
  merged — and `persistBackgrounds()` has the identical limitation, which is why
  one issue covering both charts and vector layers is the honest remedy. Issue
  text is drafted in the 2026-09-15 `## Implementation` entry in `progress.md`
  for the host to file.
- Round 3 itself returned *ship: recommended* with two documentation must-fixes
  (this revision, and a wrong line citation in `mission_insertion.h`) and no
  surviving code defect.

**Rev 6** (2026-09-14) — pre-push review round 2 returned *ship: recommended*
with two must-fixes and nine suggestions. The plan-level consequences:

- **The driver allowlist was never the thing that stopped a network fetch**, so
  the plan's "pinned driver set" protection is restated: GDAL resolves
  `/vsicurl/`, `/vsizip/`, `/vsis3/` in its virtual file system *before* driver
  selection, so an allowed GeoJSON driver would happily read a downloaded file.
  The gate is now a refusal of any `/vsi` PATH — in `VectorLayer`'s constructor
  and on the project's open *and restore* paths, the restore path being the one
  that reopens every persisted entry at startup unattended. The allowlist keeps
  the job it can do: excluding non-file drivers (`PG:`, `WFS:`, …). A KML
  NetworkLink remains a documented limitation (ADR-0016 D12).
- **The feature cap is carried INTO the parse** (`ParseOptions::max_geometries`)
  rather than applied to its result. Rev 3 scoped the cap as a bound on
  GUI-thread item construction; that leaves the worker materialising every
  geometry and attribute map of the whole file first, which is the OOM the cap
  is documented as protecting against. The parse now stops at the cap and the
  rest of the file is never read (ADR-0016 D11).
- **One follow-up filed**: camp#225 — an item that accepts the left press to
  tell a click from a drag takes that press away from the view's pan gesture.
  The fix belongs in `ProjectView`; shipped as-is, recorded in the code and in
  ADR-0016's consequences.

**Rev 5** (2026-09-14) — pre-push review round 1 returned changes-requested with
ten must-fixes; these are the plan-level consequences. The code-level fixes are
in the branch's commits and the `## Implementation` entry in `progress.md`.

- **Rev 4's "no `onRemovedFromMap()` override" deferral is REVERSED** — it rested
  on an inaccurate premise and was must-fix 1. Rev 4 argued the override would
  signal "nothing the model does not already signal". The model signals something
  *different*: `Map::setMapItemParent()` implements a drag-REORDER as
  `beginRemoveRows` + `beginInsertRows` (`map.cpp:285-307`), so
  `rowsAboutToBeRemoved` fires for a reorder too and cannot be told apart from a
  removal. Dragging a vector layer up or down the Layers tab therefore erased its
  file from `vectorLayers/files`, and it did not come back on the next launch.
  `VectorLayer::onRemovedFromMap()` now exists and emits `removedFromMap()`;
  `AutonomousVehicleProject` connects to that per layer instead of to the model.
  The "one owner for the key" decision (Plan Review round 2) is UNCHANGED —
  `persistVectorLayers()` is still the single writer; the override writes nothing.
  What changed is *which signal* the owner reacts to.
- **A new ADR** (`docs/decisions/0016-read-only-vector-file-layer.md`, must-fix
  10): the layer family's design decisions and the persisted schema. Rev 4 had no
  ADR step; every predecessor layer family has one.
- **Two new bounds, both reported** (must-fixes 6 and 7):
  `parseVectorLayers()` takes a `ParseOptions::aborted` predicate polled per
  feature, so the destructor's join is bounded by the abort rather than by the
  size of the file; and item construction is capped at
  `VectorLayer::kMaxFeatureItems`, since it can only run on the GUI thread.
- **New test file** `test/test_vector_feature_item.cpp` — line hit-testing,
  coordinate placeability, and the click/drag gating.

**Rev 4** (2026-09-14) — implementation notes, edited inline as the work landed
(plan-first workflow). Scope is unchanged; these are the points where the
implementation differs from rev 3's letter, each with its reason:

- ~~**No `VectorLayer::onRemovedFromMap()` override**~~ — **reversed in rev 5
  above; the premise was wrong.** (Kept here so the reasoning that failed stays
  legible: the argument was that an override would be dead code because
  `AutonomousVehicleProject::onVectorLayerRemoved`, connected to the Map model's
  `rowsAboutToBeRemoved`, already covered removal. It did not: that signal also
  fires for a drag-reorder.)
- **No `ProjectView` change** (step 4; Open Question 2 resolved). The feature
  item reads pan mode from the view's own `dragMode()`
  (`QGraphicsView::ScrollHandDrag`), which `ProjectView::setPanMode()` sets
  (`projectview.cpp:335`) and all six add-* modes clear to `NoDrag`
  (`projectview.cpp:273-314`). No accessor is needed — and a camp_map item could
  not call into `ProjectView` (executable) anyway, the same layering rule that
  moved the parser. `src/camp/projectview.{h,cpp}` drops out of Files to Change.
- **New `src/camp_map/vector/vector_style.{h,cpp}`** — the colour/size mapping
  as free functions, which is what step 5's tests require ("extract the
  normalize+sample logic into small free functions"). Added to Files to Change.
- **The lat/lon-order fix landed in this PR** (step 1; Open Question 3 resolved).
  Confirmed only the UNTRANSFORMED branch was wrong (`readRing` built
  `QGeoCoordinate(getX(), getY())` where the Point path built `(getY(), getX())`);
  both now go through one `toWgs84()` helper. One helper in a file this step
  already rewrote, so no follow-up issue.
- **`boundingRect()` override IS needed** (step 3's "confirm during
  implementation"): `MapItem::boundingRect()` returns an empty rect
  (`map_item.cpp:34-37`), so the layer returns `childrenBoundingRect()`.
- **Persistence-test harness** (step 8): the `vectorLayers/files` mechanism
  (key, read/write, add-dedup, remove) lives in `camp_map` so the rules are
  testable — `AutonomousVehicleProject` is no more constructible in a test than
  it was for step 2's seam. The project stays the single *owner*, rebuilding the
  whole list from `m_vectorLayers` on both paths; its two call sites are verified
  by reading the source, as step 2's are.
- **Step 9 acceptance, partially automated**: both real datasets were driven
  through the shipped parse + styling path off-GUI (33 and 7 features; the
  58.07 nT/m peak normalizes to 1.0; candidate C reads back at 307792 E /
  4762878 N UTM 19N — confirmed with `gdaltransform` — carrying its `assessment`
  text). What remains genuinely manual is the GUI half: rendering, the click
  popup, and the remove-then-restart check.

**Rev 3** (2026-09-14) — responds to Plan Review round 2 (verdict
changes-requested, 1 must-fix + 2 suggestions; all 9 round-1 findings
verified resolved). Operator chose "patch the plan, then implement" — no third
review round; the pre-push code review gates the code.
- Must-fix: the step-2 regression test had no harness (no test constructs
  `AutonomousVehicleProject`). Step 2 now extracts a `resolveInsertionParent`
  seam into its own TU with a narrow gtest, and names the fallback.
- Suggestion: `vectorLayers/files` has one writer (`persistVectorLayers()`).
- Suggestion: `openGeometry`'s parent is a `nullptr` sentinel, not a default
  argument, and the `RowInserter` is routed through the same resolved parent.


**Rev 2** (2026-09-14, this revision) — responds to the Plan Review
(`.agent/work-plans/issue-22/progress.md`, verdict changes-requested, 9
must-fix + 3 suggestion findings). Every finding was re-verified against the
current source before being folded in:

- Step 2 ("fix `VectorDataset::read()`") is **replaced**. The premise was
  false: a persisted `VectorDataset` node **is** restored today —
  `MissionItem::readChildren` (`missionitem.cpp:185`) special-cases
  `object["type"] == "VectorDataset"` and dispatches to
  `AutonomousVehicleProject::openGeometry(fname)`
  (`autonomousvehicleproject.cpp:200-214`), which opens the file. `read()` is
  never called on that path, so restoring it would be dead code (and risk a
  double-load if a `VectorDataset` item were ever also constructed there).
  The real defect, confirmed by reading both functions: `openGeometry`
  always inserts the new `VectorDataset` under `m_currentGroup` (a
  project-global "current group" pointer), not under the `MissionItem` whose
  `readChildren` is doing the dispatching — so a `VectorDataset` nested
  inside a `Group` is restored at the top level (or wherever
  `m_currentGroup` happens to point), not back where the user put it. The
  operator confirmed this re-scoping (decision 1 on the issue, 2026-09-14):
  fix `openGeometry`'s insertion target instead, with a regression test, and
  drop all `read()`-stub work.
- The consequences-table row claiming "previously-empty `VectorDataset`
  nodes now populate on reload" is removed — it described the same false
  premise and must not reach the PR description.
- `vector_parse.{h,cpp}` now moves into `camp_map` (new
  `src/camp_map/vector/` directory) instead of staying in the executable's
  sources, so `VectorLayer` (which lives in `camp_map`) can call it — per
  `.agents/README.md`'s "a library cannot call into the executable that
  links it" rule (the `camp_crash` lesson, #217; verified at
  `CMakeLists.txt:135` where `vector_parse.cpp` currently compiles only into
  `CCOMAutonomousMissionPlanner`'s `SOURCES`, and `CMakeLists.txt:295-339`
  where `camp_map` is a separate `SHARED` library with no link back to the
  executable).
- Step 3 (now step 2) adds the mandatory
  `camp_crash::install_thread_alt_stack()` first statement in the load
  worker, per the `check_worker_alt_stacks` CTest guard
  (`CMakeLists.txt:554-557`, pattern confirmed at `raster_layer.cpp:189`).
- Persistence (step 6, now step 7) adds the removal half: an
  `onRemovedFromMap()` override plus `Map`-model `rowsAboutToBeRemoved`
  bookkeeping on `AutonomousVehicleProject`, mirroring
  `RasterLayer::onRemovedFromMap()` (`raster_layer.cpp:749-757`) and
  `AutonomousVehicleProject::onChartLayerRemoved`
  (`autonomousvehicleproject.cpp:322+`) — confirmed these exist and are
  wired to the Map model precisely so a Layers-tab removal survives a
  restart (camp#90/#117). Persistence mechanism is now settled (operator
  decision 2): QSettings app state, the same `persistBackgrounds()` /
  `restorePersistedBackgrounds()` pattern, not the mission project file —
  the prior Open Question is resolved, not deferred to review-plan.
- Styling (step 5) now specifies degenerate/absent-value handling: a
  same-value field, a missing field, and a non-numeric/NaN value each get a
  defined behavior, mirroring `grid_map.cpp:198-206`'s
  `min_value == max_value` guard.
- The parser (step 1) now explicitly handles multi-part geometries
  (`wkbMultiPoint`/`wkbMultiLineString`/`wkbMultiPolygon`) and every 25D/ZM
  variant via `wkbFlatten()`, instead of silently dropping them — confirmed
  `vector_parse.cpp:86-140` currently `switch`es on the raw
  `getGeometryType()` with only `wkbPoint`/`wkbLineString`/`wkbPolygon`
  cases and a `default: break` that drops everything else, including a
  GeoJSON point with elevation (`wkbPoint25D`).
- Click-to-inspect (step 4) is now specified against `ProjectView`'s actual
  mouse-mode state machine (`projectview.cpp:57-196`), confirmed to run
  waypoint/trackline/survey placement on left-press in its add-* modes and
  set `ScrollHandDrag` (`projectview.cpp:335`) in pan mode, unconditionally
  forwarding to `QGraphicsView::mousePressEvent()` afterward (so child
  `QGraphicsItem`s under the cursor still receive press events regardless
  of mode).
- Files to Change now lists `item_types.h` (new `VectorLayerType` enum
  value — confirmed every `map::Layer` descendant declares one at
  `item_types.h:22-45`), the `mainwindow.cpp:161` startup restore call
  (confirmed `restorePersistedBackgrounds()` is called there and
  `restorePersistedVectorLayers()` needs the same site), and an
  `.agents/README.md` note.
- Suggestions folded in: per-layer settings key now overrides
  `settingsKey()` rather than relying on the `itemID()` default (camp#126
  precedent, confirmed at `gggs_tile_layer.h:72` /
  `gggs_tile_layer.cpp:1829-1852`, since `itemID()` is basename-derived —
  `map_item.cpp:53-61` — and two same-named files in different directories
  would otherwise collide); the pre-existing lat/lon-order inconsistency
  between the Point path (`op->getY(), op->getX()`,
  `vector_parse.cpp:96-104`) and `readRing` (`getX(), getY()`,
  `vector_parse.cpp:46`) is flagged for a one-line fix-or-confirm while
  step 1 is already editing that file, since the acceptance test is a
  coordinate check; persistence mechanism (previously an Open Question) is
  now a decided item, not a question.

**Rev 1** (2026-09-14) — initial plan, superseded above.

## Context

`VectorDataset` (`src/camp/vector/vectordataset.{h,cpp}`) already opens any
OGR-readable file via `camp::vector::parseVectorLayers` and builds an
editable `Group`/`Point`/`LineString`/`Polygon` tree in the **mission**
model. That parser is leak-clean (#152) and unit-tested
(`test/test_vector_dataset_cleanup.cpp`), but `ParsedGeometry`/`ParsedLayer`
carry no attributes today, and — the real, verified defect —
`AutonomousVehicleProject::openGeometry()` always inserts the reopened
`VectorDataset` under `m_currentGroup` rather than under the `MissionItem`
whose `readChildren` is restoring it, so a nested `VectorDataset` moves on
reload.

This issue asks for a second, **read-only** way to view the same kind of
file: a `map::Layer` in the Layers tab (like `RasterLayer`/backgrounds,
ADR-0002/ADR-0003) that renders points/lines/polygons with attribute-driven
styling (colour-by-field via `marine_colormap`, size-by-field) and
inspection of a feature's attributes (rev 12: on hover, as an in-scene label;
rev 11: on hover, as a Qt tooltip; click-to-inspect through rev 10).

The issue review raised six open items; the operator answered all six in a
follow-up comment on the issue (2026-09-14), and — after Plan Review rev 1
found the `read()` premise false — made a further decision (2026-09-14,
below) re-scoping the persistence-defect fix. This plan implements both
rounds of answers directly:

1. Extend `vector_parse`'s `ParsedGeometry`/`ParsedLayer` with feature
   attributes and reuse it — do not re-derive OGR iteration, and do not
   touch `VectorDataset`'s existing editable-import behavior.
2. Attributes are in scope, carried by the extended parser.
3. **(Revised per Plan Review + operator decision, 2026-09-14)** Fix the
   real defect — `AutonomousVehicleProject::openGeometry()` inserts a
   restored `VectorDataset` under `m_currentGroup` instead of under its
   original parent node — with a regression test. The `read()` stub is
   unrelated dead code on this path and is left alone.
4. Architecture: per-feature `QGraphicsItem` children under the new layer
   (for free hit-testing / hover-to-inspect), transformed to scene
   coordinates once at load — not a single painted surface like
   `RasterLayer`.
5. Colour-by-field uses `marine_colormap` (already integrated in
   `RasterLayer` and `ros/grids/grid_map.cpp`), not the deleted
   `camp::map::ColorMap` / stale #63.
6. MVP scope for this PR: OGR load (GeoJSON first), points/lines/polygons
   (including multi-part and 25D variants — see step 1) with a default
   style, colour-by-field, size-by-field, hover-to-inspect attributes,
   save/restore in the project (QSettings app state, per operator decision
   2 below), unit tests for the parser attributes and the styling mapping.
   Label-by-field and a style-editing UI are follow-on (new issues, not this
   PR).

**Operator decision (2026-09-14, responding to Plan Review rev 1):**

1. Replace step 2 with the `openGeometry()` insertion-target fix described
   above; drop the `read()` work and the wrong consequences row.
2. Persistence of the new layer's file list uses the app-state QSettings
   pattern that background rasters use (ADR-0003 §4,
   `persistBackgrounds`/`restorePersistedBackgrounds`, restored from
   `mainwindow.cpp` at startup), **including the removal half**
   (`onRemovedFromMap()` + model `rowsAboutToBeRemoved` bookkeeping, camp#90
   /#117 precedent) — not the mission file.

## Approach

1. **Move `vector_parse` into `camp_map`, and extend it with attributes and
   full geometry coverage.**
   - Relocate `src/camp/vector/vector_parse.{h,cpp}` to a new
     `src/camp_map/vector/vector_parse.{h,cpp}` (pure Qt/GDAL, no ROS — fits
     `camp_map`'s existing ROS-free boundary, confirmed at
     `CMakeLists.txt:295-339`). Update `VectorDataset`'s include and the
     `CMakeLists.txt` source lists (remove from the executable's `SOURCES`,
     add to `CAMP_MAP_SOURCES`) and the existing
     `test_vector_dataset_cleanup.cpp` include path. This is the only way
     the new `VectorLayer` (which must live in `camp_map` — see step 2) can
     call `parseVectorLayers`, per `.agents/README.md`'s
     library-cannot-call-executable rule (#217).
   - Add `QMap<QString, QVariant> attributes` to `ParsedGeometry`. In
     `parseVectorLayers`, read every field from `feature`'s
     `OGRFeatureDefn` (`GetFieldCount()`/`GetFieldDefnRef(i)`/
     `GetFieldAsString`/`GetFieldAsDouble`/`GetFieldAsInteger` keyed by
     `OGRFieldType`) into that map before `OGRFeature::DestroyFeature
     (feature)`. `VectorDataset::buildItems` ignores the new field (source
     compatible; no behavior change for the editable import path).
   - Replace the `switch(geometry->getGeometryType())` in
     `parseVectorLayers` (`vector_parse.cpp:86-140`) with a switch on
     `wkbFlatten(geometry->getGeometryType())` so every 25D/ZM/M variant
     (`wkbPoint25D`, `wkbLineStringZM`, etc.) reaches the same case as its
     2D form. Add `wkbMultiPoint`/`wkbMultiLineString`/`wkbMultiPolygon`
     cases that iterate the multi-geometry's parts (`OGRGeometryCollection::
     getGeometryRef(i)`) and emit one `ParsedGeometry` per part, reusing the
     existing Point/LineString/Polygon extraction logic (factor each into a
     small free function so the multi-* cases can call it per part without
     duplicating the transform/ring logic). Anything still unhandled
     (the curve types) stays a documented, logged skip — not a silent one —
     since the issue's format claims (shapefile/GeoPackage/KML) are then
     honestly met for the geometry types those formats actually emit.
     (Superseded in detail by later revisions, recorded here so the Approach
     section is not read as current: `wkbGeometryCollection` is HANDLED, and
     the skip is no longer a warning per geometry. An unhandled geometry does
     not spend the geometry budget, so a file of curve types emitted unbounded
     log I/O under a cap that bounded nothing there; rev 14 made it ONE summary
     line per layer carrying that layer's count and its own first type name —
     see rev 14's bullet and rev 15's correction of the type's scope.)
   - **Bound the parse itself, per geometry** (rev 6/7): `ParseOptions` carries
     `max_geometries` (the layer's feature cap) and an `aborted` predicate, and
     both travel into `appendGeometry()`'s collection recursion as a
     `ParseBudget` — checked before every part and spent on every emission. A
     cap applied to the parse's RESULT would leave the worker materialising the
     whole file first (the OOM the cap exists to prevent), and a check made only
     between features would let one huge `MultiPolygon` or nested
     `GeometryCollection` run on after the destructor asked the worker to stop.
     Abort latency is therefore bounded by one geometry — and, since rev 8, by a
     1024-vertex poll interval inside one ring, since a single ring of millions
     of vertices is itself one geometry — which is what makes the destructor's
     join on the GUI thread safe (ADR-0016 D11).
   - While in this file: confirm and, if confirmed, fix the pre-existing
     lat/lon-order inconsistency between the Point path (`op->getY(),
     op->getX()` at `vector_parse.cpp:96-104`) and `readRing` (`getX(),
     getY()` at `vector_parse.cpp:46`) — untransformed line/polygon vertices
     currently come out lat/lon-swapped relative to points. Fix in this PR
     (small, in a file this step already touches) or file a dedicated
     follow-up issue and note the decision here; do not leave it
     unacknowledged, since the acceptance test in step 8 is a coordinate
     check.

2. **Fix the real `openGeometry` persistence defect.**
   `AutonomousVehicleProject::openGeometry(fname, label)`
   (`autonomousvehicleproject.cpp:200-214`) always does
   `vd = new VectorDataset(m_currentGroup)`, ignoring which `MissionItem`
   node originally contained it. `MissionItem::readChildren`
   (`missionitem.cpp:185`) calls `project->openGeometry(...)` for a
   `type == "VectorDataset"` child without passing `this` as the intended
   parent. Fix, in three parts so the regression test has a seam that
   builds (Plan Review round 2 must-fix — no existing test constructs
   `AutonomousVehicleProject`, whose TU pulls in the whole mission tree):
   - **Seam**: a free function
     `MissionItem* resolveInsertionParent(MissionItem* requested, MissionItem* currentGroup)`
     in a small new TU `src/camp/mission_insertion.{h,cpp}` — returns
     `requested` when non-null, else `currentGroup`. Header depends only on a
     forward-declared `MissionItem`, so a narrow gtest can compile it
     without the project class.
   - **Call sites**: `openGeometry` gains a `MissionItem* parent = nullptr`
     parameter (a `nullptr` sentinel — "default to `m_currentGroup`" is not
     expressible as a default argument since `m_currentGroup` is a member).
     It resolves `parent` through the seam **once** and routes both the
     `RowInserter` and the `new VectorDataset(...)` through that resolved
     parent. `readChildren` passes `this`; the existing "Import" menu-action
     caller passes nothing and keeps its behaviour.
   - **Regression test** `test/test_mission_insertion.cpp`: (a) unit-test the
     seam — `requested` wins when set, `currentGroup` when it is null —
     compiling only `mission_insertion.cpp` + stubs (same narrow-source-set
     pattern as the other `ament_add_gtest` blocks); (b) a parse-level check
     that `MissionItem::readChildren`'s `VectorDataset` branch is the only
     `openGeometry` call that passes a parent is covered by review, not a
     test — the project class is not constructible in the test harness.
   Fallback, if the seam proves unnecessary during implementation (e.g. the
   resolution collapses to one expression): keep the narrow test anyway; the
   operator required a regression test for this fix.
   This lands in this PR as the real defect fix (operator decision 1); no
   `read()`/`write()` change is needed since that path is dead code for the
   persisted case.

3. **Add a `camp::vector::VectorLayer` class** under
   `src/camp_map/vector/` (mirrors `src/camp_map/raster/` for
   `RasterLayer`), deriving from `map::Layer`:
   - Constructor takes `(map::MapItem* parent, const QString& filename)`,
     mirrors `RasterLayer`'s shape: open the OGR dataset off the GUI thread
     (`QFutureWatcher`, abort flag + mutex — the `RasterLayer`/#213
     join-in-destructor pattern) via `parseVectorLayers`, reproject each
     `ParsedGeometry`'s WGS84 coordinates to the Web-Mercator scene **once**
     on load completion, and build one child `QGraphicsItem` per feature.
   - The load worker's **first statement** must be
     `camp_crash::install_thread_alt_stack()`, matching
     `raster_layer.cpp:189` — required by the `check_worker_alt_stacks`
     CTest guard (`CMakeLists.txt:554-557`), which fails CI for any
     `QtConcurrent::run()` entry point that omits it.
   - `int type() const override` returns a new `VectorLayerType` value
     added to `camp::map::ItemType` (`item_types.h:22-45` — every
     `map::Layer` descendant declares one; see Files to Change).
   - `boundingRect()` covers the union of child extents (confirm during
     implementation whether an explicit override is needed given
     `MapItem::boundingRect()`'s current implementation, which already
     unions children for hit-testing/painting purposes).
   - Destructor joins the load worker before teardown (the #213 pattern
     already used by `ros/geometry/polygon.h` and `OccupancyGrid`).
   - `settingsKey()` override: do **not** rely on the `itemID()` default
     (basename-derived, `map_item.cpp:53-61`) — two vector files with the
     same basename in different directories would share one settings
     group. Follow the `GggsTileLayer::settingsKey()` precedent
     (`gggs_tile_layer.cpp:1829-1852`, camp#126) and key on the full
     filename path instead.
   - `writeSettings()`/`readSettings()` overrides (QSettings, keyed by
     `settingsKey()`) persist per-layer style (`colorField`, `sizeField`,
     `colormap` name) — see step 7 for the file-list half.
   - `onRemovedFromMap()` override (rev 5): emit `removedFromMap()` so the
     project can drop this file from the persisted list. It writes no key
     itself — `persistVectorLayers()` is the single writer — but it is the
     only reorder-SAFE removal signal: the Map model's `rowsAboutToBeRemoved`
     fires for a drag-reorder too. See step 7 and ADR-0016 D9.

4. **Add feature child-item classes** (new file,
   `src/camp_map/vector/vector_feature_item.{h,cpp}`): lightweight
   `QGraphicsItem` (not `MapItem`/`QGraphicsObject` — no need for the tree
   model, settings, or signal/slot machinery per feature) for Point,
   LineString, and Polygon geometry (including multi-part instances from
   step 1, each rendered as its own child item), each holding a
   `const ParsedGeometry*` (or a copy) and its resolved paint color/size.
   Distinct from `Point`/`LineString`/`Polygon` (`src/camp/*.h`) — those are
   `MissionItem` subclasses with editing/drag/waypoint-linking baggage a
   read-only layer must not inherit.
   - Default style: point = filled circle (fixed radius unless
     size-by-field is set), line = stroked path, polygon = filled+stroked
     path with exterior/interior rings (even-odd fill rule for holes).
   - **Hover-to-inspect** (rev 12, operator decision; rev 10's click-to-inspect
     and rev 11's tooltip are both gone). The feature item accepts **no mouse
     button at all** (`setAcceptedMouseButtons(Qt::NoButton)`), so every press
     over a feature falls straight through to `QGraphicsView` — which is what
     makes a pan that starts on a feature pan (camp#225, fixed by construction)
     and keeps `ProjectView`'s add-\* placement clicks out of the item's hands.
     No pan-mode gate is needed, and therefore no `ProjectView` `mouseMode`
     accessor: rev 10 specified one, rev 11 deleted the gate that wanted it.
     Inspection is instead `setAcceptHoverEvents(true)` with
     `hoverEnterEvent()`/`hoverLeaveEvent()` filling and emptying a lazily
     created child `QGraphicsSimpleTextItem` — `GeoGraphicsItem`'s own mechanism
     (`geographicsitem.cpp:16-25`), replicated because `camp_map` cannot depend
     on the `camp` executable. It appears INSTANTLY; `QToolTip::showText()` was
     tried in rev 11 and rejected in the 2026-09-15 GUI test because it waits out
     Qt's delay and nothing else in CAMP does.
   - **Latitudes beyond the Mercator limit are CLAMPED, not dropped** (rev 7,
     operator decision): every conversion — ring vertices and the item's ANCHOR
     position alike — goes through `placeableToMap()`, which clamps to
     `web_mercator::maximum_latitude` (85.0511°) before projecting. Mercator
     sends 90° to infinity, and clamping only the path would leave the item's
     position 2.4e8 m from the data behind a locally-sane-looking path
     (ADR-0016 D13). Coordinates that are not placeable at all (NaN, a
     shapefile read without its `.prj` so northings arrive as degrees) are still
     rejected rather than clamped.
   - `boundingRect()`/`shape()` from the transformed geometry, in the
     parent layer's local (scene-mercator) coordinates.

5. **Attribute-driven styling on `VectorLayer`.**
   - `setColorField(const QString& field)` / `colorField()`: when set,
     resolve each feature's numeric value for that field, compute the
     field's min/max **only over features that have a present, numeric,
     non-NaN value for it** — mirroring `grid_map.cpp:198-206`'s
     `min_value == max_value` guard — at load, and sample a
     `marine_colormap::Palette` (`find_palette(name)`, default `"viridis"`,
     `palette->sample(normalized_value)` → `Rgba8` → `to_rgba8`/`QColor`)
     per feature. Degenerate/absent-value handling, specified explicitly
     (missing from rev 1):
     - **All present values equal** (`min == max`): treat as `grid_map.cpp`
       does — offset `min` by a small epsilon so every feature normalizes
       to 1.0 (top of the ramp) instead of dividing by zero, rather than
       falling back to the default style.
     - **Field missing on a feature, or present but non-numeric/NaN**:
       say so in a channel the PALETTE DOES NOT USE (rev 7). Rev 1's "a
       colour distinct from any position on the active palette" is not
       achievable — grayscale is both a selectable palette and the
       unknown-name fallback, and its midpoint is `noDataColor()`'s grey. The
       feature keeps that neutral grey *and* is drawn with a dashed outline
       and a hatched fill (a hollow marker for a point), which no palette can
       imitate and which survive colour-blind vision and a monochrome
       printout. `camp::vector::isNoData()` names the state as exactly the
       branch `colorForValue()` answers with `noDataColor()`, so the colour
       and the outline cannot drift apart. Never silently reuses palette
       index 0.
     - No field set → the existing per-layer default style color.
   - `setSizeField(const QString& field)`: same min/max normalization and
     the same degenerate/missing/non-numeric handling as colour-by-field
     (equal-value epsilon; missing/non-numeric → a fixed default radius,
     not a computed one), linear interpolation between a fixed min/max
     marker radius (e.g. 3–15 px) for points; lines/polygons ignore
     size-by-field (documented in the header comment, not silently
     dropped).
   - `setColormap(const std::string& name)`: mirrors
     `RasterLayer::setColormap` for consistency, applies to colour-by-field
     only.
   - Recompute per-feature paint properties when a style setter changes
     (single pass over already-loaded features — no re-parse, no re-load).

6. **Menu/action wiring.** Add an `AutonomousVehicleProject::
   openVectorLayer(fname)` entry point (parallel to `addBackgroundLayer`),
   wired from a new "Open vector layer" action alongside the existing "Open
   background" action (confirm the exact wiring site in `mainwindow.cpp`
   during implementation — same file as the step-7 startup-restore call).

7. **Persistence (operator decision 2 — QSettings app state, both halves).**
   Per ADR-0003 §4 and the operator's explicit confirmation, `VectorLayer`
   persists exactly like `RasterLayer`/backgrounds — app state, not the
   mission project file:
   - **Restore half**: a new `m_vectorLayers` list +
     `persistVectorLayers()`/`restorePersistedVectorLayers()` pair on
     `AutonomousVehicleProject`, mirroring `persistBackgrounds()`/
     `restorePersistedBackgrounds()` (`autonomousvehicleproject.cpp:
     291-320`) — QSettings key `vectorLayers/files`, de-dup by filename,
     self-heal on restore (rev 5: re-persist unconditionally, which collapses
     duplicates and normalises path spellings; a file that is not reachable
     right now is REMEMBERED and carried forward rather than dropped — an
     unmounted share is not the operator asking for a removal). Add the
     `project->restorePersistedVectorLayers();` call in `mainwindow.cpp`
     immediately alongside the existing `restorePersistedBackgrounds()`
     call at `mainwindow.cpp:161`.
   - **Removal half** (missing from rev 1 — confirmed
     `RasterLayer::onRemovedFromMap()` at `raster_layer.cpp:749-757` and
     `AutonomousVehicleProject::onChartLayerRemoved` at
     `autonomousvehicleproject.cpp:322+`, wired to the `Map` model's
     `rowsAboutToBeRemoved`, exist for exactly this reason, camp#90/#117):
     add `VectorLayer::onRemovedFromMap()` (signals removal; it does **not**
     write `vectorLayers/files` itself — Plan Review round 2 suggestion:
     the raster precedent has two writers over *two different* keys, so the
     vector key gets **one owner**, `AutonomousVehicleProject::persistVectorLayers()`,
     called from both the add and the remove paths) and an
     `AutonomousVehicleProject::onVectorLayerRemoved` slot connected
     **(rev 5) to that per-layer signal, NOT to the Map model's
     `rowsAboutToBeRemoved`** — which `Map::setMapItemParent()` also fires for a
     drag-reorder, so reacting to it un-persisted a layer the operator only
     moved. (Drop the matching `m_vectorLayers` entry,
     re-persist). Without this, a vector layer removed from the Layers tab
     would silently reappear on next launch.
   - Per-layer style (`colorField`, `sizeField`, `colormap` name) persists
     via `VectorLayer`'s own `writeSettings()`/`readSettings()` override
     (step 3), keyed by the overridden `settingsKey()` — no new persistence
     plumbing needed beyond new keys under that group.

8. **Tests** (new `test/` files, wired into `CMakeLists.txt` next to the
   existing `ament_add_gtest` blocks for `test_vector_dataset_cleanup` /
   `test_raster_layer_gdal_cleanup` / `test_background_persistence`):
   - `test_vector_parse_attributes.cpp`: extend the existing
     GeoPackage-writer pattern from `test_vector_dataset_cleanup.cpp` with
     typed fields (string, int, real) on each feature; assert
     `ParsedGeometry::attributes` round trips the values and types. Also
     write a `MultiPolygon` (and one 25D `Point`) feature and assert each
     part/variant is parsed into a `ParsedGeometry`, not silently dropped.
     Keep the leak-check coverage from the existing test intact.
   - `test_vector_layer_styling.cpp`: headless unit test of the
     colour-by-field and size-by-field mapping functions (extract the
     normalize+sample logic into small free functions or static methods
     that don't need a `QApplication`/GUI thread, matching
     `test_color_map.cpp`'s headless style) — assert min/max feature values
     map to the palette's first/last LUT entries, mid-range values
     interpolate, an all-equal field maps every feature to the top of the
     ramp (not a divide-by-zero/NaN), and a feature missing the field (or
     holding a non-numeric/NaN value) gets the documented neutral color
     rather than palette index 0.
   - `test_vector_layer_teardown.cpp`: confirm the load worker is joined in
     the destructor before the dataset/parsed data it captured is freed —
     the #213 pattern, analogous to
     `test_raster_layer_gdal_cleanup.cpp`'s abort-on-destroy coverage.
   - `test_mission_insertion.cpp`: the step-2 seam test — narrow source
     set (`mission_insertion.cpp` only), asserts `resolveInsertionParent`
     returns the requested parent when given and the current group when
     not. (Rev 2's `test_open_geometry_nested_group.cpp` round-trip is
     dropped: nothing in `test/` can construct `AutonomousVehicleProject`.)
   - `test_vector_layer_persistence.cpp` (or extend
     `test_background_persistence.cpp`'s pattern in a new file scoped to
     vector layers): mirror its `RestoreExistingNoReseedAndDedup` and
     `RemoveDePersistsAndSticks` coverage for `vectorLayers/files` — add,
     restore, remove, confirm it stays removed across a second restore.
   - The `check_worker_alt_stacks` CTest guard (`CMakeLists.txt:554-557`)
     already runs against every `QtConcurrent::run()` entry point in the
     tree; no new test needed for step 3's alt-stack call, but confirm it
     passes locally before pushing.

9. **Manual acceptance** against the issue's two test datasets
   (`~/data/logs/analysis/2026-09-14_massabesic_mag/
   massabesic_mag_peaks.geojson`, `massabesic_joint_candidates.geojson`) —
   not automatable in this PR (real files outside the repo, GUI rendering),
   but recorded here so `review-code`/the PR description can check it off
   explicitly: load both, confirm colour-by-field on
   `analytic_signal_nT_per_m` puts the 58 nT/m peak at the top of the ramp,
   confirm candidate C's popup identifies the point at 307792 E /
   4762878 N (UTM 19N) with its `assessment` text, and confirm a vector
   layer removed via the Layers tab does **not** reappear after closing and
   reopening CAMP.

## Files to Change

| File | Change |
|------|--------|
| `src/camp_map/vector/vector_parse.h` (moved from `src/camp/vector/`) | Add `attributes` field to `ParsedGeometry`; document type-mapping rules |
| `src/camp_map/vector/vector_parse.cpp` (moved from `src/camp/vector/`) | Read OGR field values into `attributes`; `wkbFlatten` + multi-part geometry handling |
| `src/camp/vector/vectordataset.h`, `.cpp` | Update include path for the moved `vector_parse.h`; no behavior change |
| `src/camp/autonomousvehicleproject.h` | `openGeometry()` gains a parent-group parameter (defaulted); `m_vectorLayers`, `openVectorLayer()`, `persistVectorLayers()`/`restorePersistedVectorLayers()`, `onVectorLayerRemoved` declarations |
| `src/camp/autonomousvehicleproject.cpp` | `openGeometry()` inserts under the passed parent instead of always `m_currentGroup`; new vector-layer persistence + removal implementations mirroring `addBackgroundLayer`/`persistBackgrounds`/`restorePersistedBackgrounds`/`onChartLayerRemoved` |
| `src/camp/missionitem.cpp` | `readChildren`'s `VectorDataset` case passes `this` as the restore parent |
| `src/camp_map/map/item_types.h` | New `VectorLayerType` enum value |
| `src/camp_map/vector/vector_layer.h` (new) | `VectorLayer : public map::Layer` — async OGR load (with alt-stack install), style setters, `settingsKey()` override, `onRemovedFromMap()`, persistence hooks |
| `src/camp_map/vector/vector_layer.cpp` (new) | Load worker, scene-coordinate transform, style recompute (with degenerate/missing-value handling), readSettings/writeSettings |
| `src/camp_map/vector/vector_feature_item.h` (new) | Read-only per-feature `QGraphicsItem` (point/line/polygon); rev 12: hover-to-inspect via a lazily created child `QGraphicsSimpleTextItem` label (the `GeoGraphicsItem` mechanism, replicated), and `setAcceptedMouseButtons(Qt::NoButton)` |
| `src/camp_map/vector/vector_feature_item.cpp` (new) | Paint + hit-test (rev 5: line shapes are STROKED, or Qt's fill-area test never picks them) + coordinate placeability + (rev 12) `setAcceptHoverEvents(true)` with `hoverEnterEvent()`/`hoverLeaveEvent()` driving the label, replacing rev 11's `setToolTip()` and rev 10's mouse handlers |
| `docs/decisions/0016-read-only-vector-file-layer.md` (new, rev 5) | ADR for the layer family and the persisted schema |
| `test/test_vector_feature_item.cpp` (new, rev 5) | Line hit-testing, coordinate placeability; rev 12: the hover LABEL (text, settings, cleared on leave, none before the first hover) and the no-mouse-button contract |
| `src/camp_map/vector/vector_style.h`/`.cpp` (new) | Colour/size-by-field mapping as free functions — the headless-testable seam step 5's tests need |
| `src/camp/projectview.cpp` (rev 7, REVERTED rev 11, rev 12 cursor) | Rev 12 sets `Qt::ArrowCursor` on the viewport in pan mode (ADR-0016 D16) — the only ProjectView change this branch carries, and it is a real one: the file is **not** byte-identical to `jazzy`. Rev 7 deferred `mousePressEvent()`'s switch back to pan mode until after the press was forwarded, so the item's pan-mode gate read the mode the operator clicked in. Rev 11 removed the gate (hover, no mouse buttons), leaving that change without a purpose, so the **press path** is reverted to `jazzy`; rev 13 scopes the release-side cursor reset to the left button, the only button `ScrollHandDrag` pans with |
| `src/camp/mainwindow.cpp` | "Open vector layer" action wiring; `restorePersistedVectorLayers()` call alongside `restorePersistedBackgrounds()` at `mainwindow.cpp:161` |
| `CMakeLists.txt` | Move `vector_parse.cpp` from the executable's `SOURCES` to `CAMP_MAP_SOURCES`; add `vector_layer.cpp`/`vector_feature_item.cpp` to `CAMP_MAP_SOURCES`; 5 new `ament_add_gtest` blocks |
| `test/test_vector_parse_attributes.cpp` (new) | Attribute parse round trip + multi-part/25D geometry coverage |
| `test/test_vector_layer_styling.cpp` (new) | Colour/size-by-field mapping, including degenerate/missing-value cases |
| `test/test_vector_layer_teardown.cpp` (new) | Thread-safe teardown (#213 pattern) |
| `src/camp/mission_insertion.{h,cpp}` (new) | `resolveInsertionParent` seam (step 2) |
| `test/test_mission_insertion.cpp` (new) | step-2 seam regression test, narrow source set |
| `test/test_vector_layer_persistence.cpp` (new) | Add/restore/remove/stays-removed round trip for `vectorLayers/files` |
| `.agents/README.md` | Note `VectorLayer` (read-only display, `camp_map`) vs. `VectorDataset` (editable import, mission tree) as the two vector-file entry points |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Capture decisions, not just implementations | The architecture forks (reuse `vector_parse`; per-feature items vs. single surface; persistence mechanism) are operator-decided (issue comment + Plan Review response, 2026-09-14) and restated in Context/decision blocks above. No new ADR proposed — this plan applies existing ADR-0002/0003/0007/0008 precedent, not a new pattern. |
| A change includes its consequences | The real `openGeometry` defect (step 2) lands in this PR with a regression test. Persistence's removal half (step 7) is no longer missing. Library layering (step 1's move) is resolved before implementation, not discovered at link time. |
| Only what's needed | Label-by-field and style-editing UI stay deferred to follow-on issues (operator's MVP scope). Multi-part/25D geometry support is added because dropping it silently would misrepresent the issue's own format claims — not scope creep. |
| Test what breaks | Five new test files: attribute+geometry-coverage parsing, styling math (including degenerate cases), GDAL/thread teardown, the nested-parent persistence regression, and the layer-list persistence round trip (add/remove/stays-removed). |
| Improve incrementally | `VectorDataset` (editable import) is untouched behaviorally except the `openGeometry` parent fix; the new read-only layer is fully additive. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0002 (Web-Mercator scene/layer model) | Yes | Coordinates transformed to scene Web-Mercator once at load (step 3), matching the ADR's "transform once, let Qt scale" convention already used by `RasterLayer` and mission items. |
| ADR-0003 (backgrounds as layers/depth tree) | Yes | `VectorLayer` is an ordinary Layers-tab layer (§1 pattern); §4's split persistence (app-state for Layers-tab layers, project-file for mission items) is the confirmed basis for step 7's persistence design, now including the removal half. |
| ADR-0007 (RasterFieldSource render abstraction) | No | Not applicable — `VectorLayer` does not implement `RasterFieldSource`; per-feature `QGraphicsItem` children are the chosen shape (operator decision), not `RasterLayer`'s single-texture pattern. |
| ADR-0008 (marine_colormap LUT bake) | Yes | Colour-by-field uses `marine_colormap::find_palette`/`sample()` directly (step 5), the same facility `RasterLayer` and `grid_map.cpp` already use — not the deleted `camp::map::ColorMap`. |
| ADR-0011 (viewport-clip render convention) | No (for this PR) | The two acceptance datasets are 33 and 7 features; no viewport-scoped culling in this MVP. Noted as a follow-on if vector layers grow to survey-index-footprint scale. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `vector_parse.{h,cpp}` moves from the executable into `camp_map` | `VectorDataset`'s include path; `CMakeLists.txt` source lists (both the executable's `SOURCES` and `CAMP_MAP_SOURCES`); `test_vector_dataset_cleanup.cpp`'s include | Yes — step 1 and Files to Change |
| `ParsedGeometry`/`ParsedLayer` gain an `attributes` field and multi-part/25D handling | `VectorDataset::buildItems` (source-compatible, ignores the new field; unaffected by the geometry-type widening since it already only handles Point/LineString/Polygon) | Yes — verified no behavior change, noted in step 1 |
| A new `VectorLayer` type is added to the Layers tab | `item_types.h`'s `ItemType` enum; any place that maps a persisted layer "type" string to a class | Yes — `VectorLayer` persists via QSettings app state (step 7), not the mission JSON's type-string dispatch in `missionitem.cpp`, so no new case is needed there for the layer itself |
| `AutonomousVehicleProject::openGeometry()` gains a `MissionItem* parent = nullptr` parameter | The existing "Import" menu-action call site passes nothing and resolves to `m_currentGroup` through the seam, so its behavior is unchanged | Yes — step 2, nullptr sentinel + `resolveInsertionParent` |
| A vector layer can be removed from the Layers tab | Its entry in `vectorLayers/files` must be dropped, not just the in-memory item | Yes — step 7's removal half, previously missing |
| A new menu action opens vector layers | `.agents/README.md` if it documents the menu structure | Yes — `.agents/README.md` note added in this PR (Files to Change), since the plan itself proposes the distinction it should record |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `.agents/README.md` gains a short
  note distinguishing `VectorLayer` (read-only display, `camp_map`) from
  `VectorDataset` (editable import, mission tree) — the two now look
  confusingly similar by name, and this PR is the moment that distinction
  is introduced, so it is a stale-docs item (the change itself creates the
  gap), not a proposal. See Files to Change.
- **Agent-instruction candidates** (proposals only): None beyond the above
  — the library-layering rule (#217) and the persistence pattern (ADR-0003
  §4) are both already documented; this PR is an application of existing
  guidance, not a new pattern needing its own instruction entry.

## Open Questions

All three were settled during implementation (rev 4):

- [x] `ParsedGeometry::attributes` container type — **`QMap<QString, QVariant>`**.
      It also gives the attribute popup a stable alphabetical field order,
      so the ordered-vector alternative bought nothing.
- [x] Whether `ProjectView` needs a read-only `mouseMode` accessor — **no**.
      Pan mode is `QGraphicsView::ScrollHandDrag`, which the feature item reads
      from the view directly; every add-* mode sets `NoDrag`, so the layering
      rule is kept without an accessor. **`ProjectView` is NOT untouched,
      however** (rev 7): reading `dragMode()` only works if the view has not
      already switched back to pan mode by the time the press reaches the item,
      and it had. `mousePressEvent()` now defers that switch until after the
      forward. The accessor question stands answered; the "untouched" claim
      does not.
- [x] Whether the Point-vs-`readRing` lat/lon-order inconsistency is fixed here
      or filed as a follow-up — **fixed in this PR**. Only the untransformed
      branch was wrong, and both branches now share one `toWgs84()` helper.

## Estimated Scope

Single PR (MVP per operator decision). Label-by-field and a style-editing
UI are explicitly out of scope, to be filed as follow-on issues after this
PR lands.
