# Parity: markers (visualization_msgs/Marker)

**camp**: `markers/markers.{h,cpp}`, `markers/markers_converter.h`, `markers/markers.ui`
**camp2**: `ros/markers/{marker,markers,marker_namespace}.{h,cpp}`

## Architectural difference

**camp** is a single monolithic `Markers` `GeoGraphicsItem`. It owns one nested
`std::map<ns, std::map<id, payload>>` (`markers.h:52`) and renders everything
in one `paint()` override (`markers.cpp:59-86`), building a `QPainterPath` per
marker. TF gating + conversion is offloaded to a `TfDispatcher`
(`markers.cpp:189-194`); the converter is a testable free function
`convertMarker` (`markers_converter.h:50`). Positioning is geographic
(ECEF/lat-long).

**camp2** is a three-level QGraphicsItem *tree*: `Markers` (subscription/
dispatch) → `MarkerNamespace` (one per ns) → `Marker` (one per id). Each
`Marker` is its own `Layer` building *child* graphics items
(`QGraphicsRectItem`/`EllipseItem`/`LineItem`/`SimpleTextItem`,
`marker.cpp:44-96`) rather than painting a path. ns/id lookup is implicit in
the child list via `qgraphicsitem_cast`. Positioning is Web Mercator
(`marker.cpp:35`). Each `Marker` runs its own 1 Hz expiry `QTimer`
(`marker.cpp:20-23`).

> **Consequence to note up front:** camp2's `Marker::updateMarker`
> *unconditionally deletes all child graphics at the top of every update*
> (`marker.cpp:27-32`) and rebuilds only for `ADD`. So a DELETE clears the
> *visuals*, but the empty `Marker`/`MarkerNamespace` *objects* are never
> removed from the scene graph.

## Feature matrix

| Feature | camp (file:line) | camp2 (file:line) | Delta | Resolution |
|---|---|---|---|---|
| SPHERE | `markers.cpp:96-101` | `marker.cpp:53-59` | parity | keep camp2 |
| LINE_STRIP | `markers.cpp:102-118` | `marker.cpp:60-87` | parity | keep camp2 |
| LINE_LIST | `markers.cpp:119-144` | `marker.cpp:60-87` | parity | keep camp2 |
| TEXT_VIEW_FACING | `markers.cpp:145-154` (font scaled by `scale.z`) | `marker.cpp:88-93` (no scale) | differs | font sizing tracked with the TEXT-rendering work in **#76 / PR #78** (same code path); not in #70 |
| **CUBE** | not handled (`markers.cpp:155-157` warn) | `marker.cpp:46-52` | camp2-only (keep) | inherit |
| CYLINDER / CUBE_LIST / SPHERE_LIST / POINTS / MESH / TRIANGLE_LIST / ARROW | not handled | not handled | parity (gap) | out of scope unless needed |
| Action ADD/MODIFY (0) | `markers.cpp:251-264` | `marker.cpp:33-97` | parity | keep camp2 |
| **Action DELETE (2)** | `markers.cpp:265-271` (erases id) | visuals cleared (`marker.cpp:27-32`) + `checkExpired` action!=0 (`marker.cpp:110`); **`Marker`/`MarkerNamespace` objects leaked** | differs | ✅ **#70**: `MarkerNamespace::updateMarker` deletes the `Marker` object; `Markers::pruneEmptyNamespaces` drops the emptied namespace |
| **Action DELETEALL (3)** | `markers.cpp:272-276` (clears **all** ns) | `marker_namespace.cpp:19-24` clears **one** ns only | differs (**regression**) | ✅ **#70 / PR #75**: fans out to every namespace; this PR also deletes the `Marker` objects + prunes namespaces |
| Unknown action handling | warns (`markers.cpp:277-280`) | silent | camp-only (port) | ✅ **#70**: throttled warn in `Markers::updateMarker` |
| Lifetime expiry | one-shot per ADD + `purgeExpiredMarkers` (`markers.cpp:254-263,287-311`) | per-Marker 1 Hz poll (`marker.cpp:106-118`) | differs | both work; keep camp2 |
| Already-expired drop at ingest | `markers_converter.h:61-68` | none | camp-only (port) | ✅ **#70**: pre-drop in `addMarkers` (non-zero-stamp guard mirrors `checkExpired`) |
| Empty frame_id drop | `markers_converter.h:69-75` | none | camp-only (port) | ✅ **#70**: explicit guard in `addMarkers` |
| Namespace+id keying | nested `std::map` (`markers.h:52`) | scene-graph tree (`markers.cpp:74-83`, `marker_namespace.cpp:26-44`) | differs | keep camp2 tree |
| Color (RGBA) | pen a=a, brush a=a/2 (half-alpha fill) (`markers.cpp:70,79`) | pen+brush full a=a (`marker.cpp:38-42`) | differs (UX) | **DECIDED 2026-06-02: make fill alpha a config option** (don't hardcode either). Wire a fill-opacity setting during the markers migration; pick the default at PR5 |
| Pose / yaw | `markers_converter.h:106` + path rotate | `markers.cpp:62` + `setRotation` (`marker.cpp:36`) | parity | keep camp2 |
| Frame / TF handling | TF→earth, MessageFilter-gated, buffers 50 / 1 s drop (`markers.cpp:191`, `markers_converter.h:77-105`) | `transformToWebMercator` → `Layer::lookupEarthTransform` caches the last good earth←frame transform and reuses it within a 1 s staleness budget (`ros/transform_cache.cpp`); only cold-start / over-budget gaps drop (`markers.cpp:61,66-70`) | partial parity | **PR5: TF-gap robustness via transform cache (not message buffering).** camp buffered *messages* until TF arrived; camp2 caches the *transform* and holds the last placement across a brief gap — covers the common transient-lag case (the white-flicker / dropped-map symptom #67 fixed for grids) without the MessageFilter. Buffering early-arriving markers (held before their first-ever TF) is still not ported; revisit if pre-TF marker arrival shows up in the field |
| Visibility toggle | `displayCheckBox` (`markers.ui:18`, `markers.cpp:313-318`) | via `Layer` base | differs | verify `Layer` provides equivalent |
| QoS | `QoS(1)` (`markers.cpp:202`) | `QoS(10).durability_volatile()` (`markers.cpp:22-23`) | differs | keep camp2 |
| Dispatcher retarget (thread-safe swap) | mutex snapshot (`markers.h:63-64`) | n/a (topic fixed at ctor) | camp-only | not needed unless retarget feature added |
| Testable converter | `convertMarker` free fn (`markers_converter.h:50`) | inline in `addMarkers` | camp-only | optional: factor for unit tests |

## Direct answer: do we lose DELETE/DELETEALL, and does expiry subsume them?

> **Resolved (#70).** DELETEALL fan-out landed in PR #75; DELETE object removal,
> namespace pruning, the two ingest drops, and the unknown-action warn landed in
> the #70 follow-up PR. Bullets below are the original analysis, kept for context.

- **DELETE (2):** Not a hard *visual* regression (camp2 clears children), **but**
  camp2 leaks empty `Marker`/`MarkerNamespace` objects camp does not. **Port:
  actually remove the `Marker` item + prune empty namespaces on DELETE.**
  → *Done:* `MarkerNamespace::updateMarker` now deletes the `Marker` and
  `Markers::pruneEmptyNamespaces` drops the emptied namespace.
- **DELETEALL (3):** **Real semantic regression.** camp clears **all**
  namespaces (`markers.cpp:272-275`, per the visualization_msgs spec); camp2
  only clears the message's single namespace (`marker_namespace.cpp:19-24`).
  **Fix before retiring camp's markers.** → *Done in PR #75* (fan-out), extended
  here to delete the `Marker` objects and prune namespaces.
- **Does the expiry timer subsume them?** **No.** The timer handles *lifetime*
  removal; it provides neither the all-namespace `DELETEALL` fan-out nor object
  cleanup. Complementary, not a substitute — `DELETEALL` must be fixed
  independently.

## Open questions

- **camp2 object lifecycle on delete:** confirmed leak in these files; check
  `ros/layer.{h,cpp}` for any base-class GC / scene pruning before calling it a
  true leak.
- **Visibility in camp2:** presumed provided by `Layer` (`displayCheckBox` has
  no analog in markers files) — verify in `layer.h`.
- **`transformToWebMercator` target frame:** warning text says "earth"
  (`markers.cpp:69`) but call is `transformToWebMercator`; confirm internal path.
- **Text legibility at chart scale:** camp2 text has no font sizing
  (`marker.cpp:88-93`); camp sizes by `scale.z`. Verify legibility.
- ~~**Fill alpha intent (UX).**~~ **RESOLVED 2026-06-02:** make fill alpha a
  config option rather than hardcoding camp's half or camp2's full; default
  chosen at PR5 markers migration.
- **markers_manager.\*** (retiring): not read; confirm no unique behavior beyond
  topic discovery before deleting.

## PR3a-i orientation/units note (Web-Mercator scene)

After PR3a-i the scene is Web Mercator and `ProjectView` flips the Y axis (north up).
camp's `Markers::markerPath` builds LINE_STRIP/LINE_LIST/SPHERE/TEXT geometry as
**local offsets** added to the anchor, with a baked-in Y reflection
(`y = x·sinr - y·cosr`, `markers.cpp:108-109,130,134`) and a `pixel_size_`
(chart metres-per-pixel) unit scaling — both correct for the old chart-pixel scene,
both wrong on the flipped Web-Mercator scene. Net effect on-branch: **marker paths
render vertically flipped and mis-scaled.** This is **not** patched in camp (Bucket A
retire-and-replace): camp2's `ros/markers` is written for the Web-Mercator scene and
supersedes this code in PR5. Retirement of `markers.*` therefore also resolves the
flip/units — verify the replacement renders LINE_STRIP/markers correctly oriented and
scaled as part of the PR5 parity gate.
