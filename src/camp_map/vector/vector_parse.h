#ifndef CAMP_VECTOR_PARSE_H
#define CAMP_VECTOR_PARSE_H

#include <functional>
#include <vector>

#include <QGeoCoordinate>
#include <QMap>
#include <QString>
#include <QVariant>

class GDALDataset;

namespace camp::vector
{

// [camp#22] This TU lives in camp_map (not in the CCOMAutonomousMissionPlanner
// executable, where it started under #152) because BOTH vector-file entry points
// have to reach it: VectorDataset (the editable mission-tree import, in the
// executable) and the read-only camp::vector::VectorLayer (a map::Layer, in
// camp_map). A library cannot call into the executable that links it — the
// libcamp_crash lesson from #217 — so the shared parser belongs in the library.
// It stays pure Qt/GDAL with no ROS, which is what camp_map requires (ADR-0002).

// Plain-data geometry parsed from an OGR vector source, coordinates already in
// WGS84. Deliberately free of MissionItem / AutonomousVehicleProject coupling:
// VectorDataset::open historically interleaved the GDAL/OGR resource lifecycle
// (per-layer coordinate transformations + point iterators — the issue #152 leak
// sites) with construction of the project item graph, which made the leak path
// impossible to exercise without standing up the whole application. Splitting
// the pure parse into this standalone TU lets a headless unit test reach it.
struct ParsedGeometry
{
    enum Type { Point, LineString, Polygon };
    Type type;
    // Point: exactly one coordinate. LineString / Polygon exterior ring: the
    // ordered vertices.
    std::vector<QGeoCoordinate> exterior;
    // Polygon only: each interior ring's vertices (empty for Point/LineString).
    std::vector<std::vector<QGeoCoordinate>> interiorRings;
    // [camp#22] The source feature's attribute fields, keyed by field name.
    //
    // Type mapping, applied per OGRFieldDefn::GetType():
    //   OFTInteger        -> int      (OFSTBoolean/OFSTInt16 subtypes included)
    //   OFTInteger64      -> qlonglong
    //   OFTReal           -> double
    //   everything else   -> QString  (GetFieldAsString: OFTString, OFTDate,
    //                                  OFTDateTime, list types, binary, ...)
    // A field that is unset or NULL on a feature is ABSENT from the map rather
    // than present-and-empty, so a consumer can tell "no value" from "empty
    // string" — the styling path paints a missing value in its no-data colour
    // instead of at the bottom of the ramp.
    //
    // One entry per PART of a multi-part geometry: a MultiPolygon feature emits
    // one ParsedGeometry per polygon, each carrying a copy of the same feature
    // attributes, so per-feature styling and hover-to-inspect work uniformly
    // over single- and multi-part sources.
    QMap<QString, QVariant> attributes;
};

struct ParsedLayer
{
    QString name;
    std::vector<ParsedGeometry> geometries;
};

// [camp#22] Caller-supplied controls for one parse.
struct ParseOptions
{
    // Polled once per layer and once per feature — and, so the promise below
    // holds in every window, once more after the bounded lookahead the geometry
    // cap runs. When it returns true the parse stops where it is and returns what
    // it has, with ParseDiagnostics::aborted set. Empty (the default) means
    // "never abort".
    //
    // This exists because VectorLayer parses on a worker thread its DESTRUCTOR
    // joins: without a cancellation hook inside the loops, closing a layer part
    // way through a large file blocks the GUI thread for the rest of the parse.
    // The abort flag is read here, per feature, exactly as RasterLayer re-checks
    // inside its work loops (camp#213).
    std::function<bool()> aborted;

    // [camp#22] Stop after this many ParsedGeometry parts have been emitted in
    // total, across all layers. 0 (the default) means unlimited.
    //
    // The cap has to be applied HERE, not by the caller after the fact: parsing a
    // national coastline shapefile materialises every geometry and every
    // attribute map of the file in the worker before the caller sees anything, so
    // a cap applied afterwards bounds the GUI work and leaves the memory
    // unbounded — the OOM that VectorLayer::kMaxFeatureItems is documented as
    // protecting against. When the cap stops the parse, exactly `max_geometries`
    // geometries are returned and ParseDiagnostics::geometry_cap_reached says so.
    //
    // [camp#22 round-8 suggestion] WHAT THE CAP DOES NOT BOUND, deliberately: a
    // geometry that is dropped as undrawable (an exterior with no usable vertex —
    // ParseDiagnostics::geometries_with_empty_exterior — a polygon with no ring,
    // a standalone point whose coordinate would not transform, or — [round 9] —
    // a geometry NO vertex of which can be placed on the scene,
    // ParseDiagnostics::geometries_without_placeable_vertex) is not charged to it,
    // so a file whose geometries ALL drop out is read to its last feature. The cap's "the rest of the file is never read" holds
    // for a file that produces drawable geometry, which is the case it exists for.
    // Accepted, with the reasoning written down rather than rediscovered:
    //  * MEMORY, the thing the cap is here to bound, is unaffected — a dropped
    //    geometry is never materialised, so such a parse holds nothing;
    //  * the parse stays INTERRUPTIBLE at the same granularity as any other
    //    (`aborted` is polled per feature and every kVertexPollInterval vertices),
    //    so closing the layer still returns the GUI thread promptly;
    //  * the alternative — charging the cap for geometries that were dropped —
    //    is exactly what the round-5 fix removed: it spends the operator's budget
    //    on shapes that draw nothing, and makes a mixed file report the cap
    //    reached having produced fewer items than the cap. [round 9] The
    //    unplaceable class is where that mattered most: a multi-layer container
    //    whose FIRST layer has no spatial reference could exhaust the cap on
    //    shapes that draw nothing and never reach the good layer behind it.
    // What is left is wall time on a pathological file, and the operator is told:
    // the drop count reaches the layer's status (VectorLayer::loadFinished()).
    int max_geometries = 0;
};

// [camp#22] What the parse could NOT do. Every field counts something that was
// deliberately dropped, so a caller can report it instead of presenting a partial
// read as a clean one.
struct ParseDiagnostics
{
    // The parse stopped early because ParseOptions::aborted returned true.
    bool aborted = false;
    // The parse stopped early because ParseOptions::max_geometries was reached
    // AND input was left unread. What is returned is the first `max_geometries`
    // geometries of the file; how many more the file holds is deliberately NOT
    // reported, because reading that far is the cost the cap exists to avoid (and
    // an OGR feature count is not a geometry count — one multi-part feature emits
    // several).
    //
    // [camp#22] "And input was left unread" is load-bearing: a file holding
    // exactly `max_geometries` geometries has been read IN FULL, and this flag is
    // what puts "rest of file not read" in the layer's status. A bounded lookahead
    // (the current feature's own unread parts, one feature on the current layer,
    // one per remaining layer) establishes that something remains before the flag
    // is set.
    //
    // [camp#22] A parse that hits BOTH the cap and the abort reports `aborted`,
    // not this: an abort can truncate a ring mid-way, and `aborted` is the flag
    // on which a caller discards a partial result, so it must never be masked by
    // the cap. The parser clears this flag in that case, so `aborted` and
    // `geometry_cap_reached` are never both set.
    //
    // [camp#22 round-8] How much such a result holds is UNSPECIFIED: an abort
    // raised inside a work loop leaves a result that is partial by construction,
    // while one raised in the window between the cap's trim and its return leaves
    // one trimmed to exactly `max_geometries`. Neither is a promise, because
    // `aborted` is the flag on which the caller discards the result whole — the
    // "exactly `max_geometries`" contract is about the cap-reached return, which
    // is the one a caller consumes.
    bool geometry_cap_reached = false;
    // Layers seen, and layers SKIPPED ENTIRELY because the layer declares a
    // spatial reference but no transformation to WGS84 could be built for it.
    // Such a layer must never fall through to the untransformed branch: its
    // coordinates are projected metres, and reading them as degrees silently
    // places the features about 1e17 metres from where they belong.
    int layers_total = 0;
    int layers_failed = 0;
    // Individual points whose coordinate transformation FAILED. OGR leaves a
    // failed point at HUGE_VAL, so these are dropped rather than carried.
    //
    // This counts VERTICES: a bad vertex of a line or polygon that was drawn
    // perfectly well is in here, and so is the one coordinate of a standalone
    // Point geometry (which is also counted in point_geometries_dropped below).
    // That mixture is why it must never be folded into a caller's "items I could
    // not show" tally — see the field below.
    int points_dropped = 0;
    // [camp#22 round-9 should-fix] STANDALONE Point geometries dropped because
    // their one coordinate would not transform — the subset of points_dropped
    // that cost the caller a whole geometry rather than a vertex.
    //
    // It needs its own field for the same reason geometries_with_empty_exterior
    // does, and it is the one geometry class that fix did not reach: such a point
    // is never emitted, so VectorLayer's own skip loop cannot count it, and
    // points_dropped cannot be folded in as a substitute because it also counts
    // vertices of lines and polygons that ARE on screen. Without this, a file of
    // points that all fall outside their projection's inverse domain produced
    // zero features with every usable counter at zero, and the Layers tab reported
    // the bare "(no items)" — an EMPTY-FILE verdict, whose remedy is a different
    // one entirely. VectorLayer folds this into the count of items it could not
    // place.
    int point_geometries_dropped = 0;
    // Geometries of a type this parser does not handle (the curve types), and the
    // name of the FIRST such type seen — the parser logs one summary line per
    // layer rather than one per geometry (an unhandled geometry does not spend the
    // geometry budget, so a file of curve types would otherwise emit unbounded log
    // I/O), and the type name is the part of that message worth keeping.
    //
    // SCOPE: the count and the name below are PARSE-WIDE (every layer), while the
    // per-layer summary line names that layer's own first unhandled type — the
    // parser scopes the field to the layer for the duration of the layer body and
    // restores the parse-wide first type afterwards.
    int geometries_unhandled = 0;
    QString first_unhandled_geometry_type;
    // Polygons dropped because they carry no exterior ring — there is no outline
    // to draw and no ring to close, so nothing can be emitted for them.
    int polygons_without_exterior_ring = 0;
    // [camp#22 round-8 must-fix] Line strings and polygons dropped because their
    // exterior came back EMPTY: every vertex failed to transform (each one also
    // counted in points_dropped), or the ring held no vertex to begin with. Such a
    // geometry is neither emitted nor charged to the cap — an empty shape draws
    // nothing and must not spend a cap slot a drawable feature needs.
    //
    // It needs its own field because nothing else counts it: the file whose
    // features all fall outside their projection's inverse domain is the motivating
    // case, and without this the caller sees an empty result with every other
    // counter at zero and reports it as an empty FILE. `points_dropped` is not a
    // substitute (a genuinely empty ring has no vertex to count, and a partly
    // dropped ring is still drawn), and `polygons_without_exterior_ring` counts the
    // different case of no ring at all. VectorLayer folds this into the count of
    // items it could not place, so the Layers tab says what happened.
    int geometries_with_empty_exterior = 0;
    // [camp#22 round-9 suggestion] Geometries dropped because NO vertex of their
    // exterior can be placed on the scene (`hasPlaceableCoordinate()`), although
    // every one of them transformed successfully.
    //
    // The case is a layer with no spatial reference at all: no transformation is
    // built, the file's projected metres are read as degrees, and the result is a
    // perfectly well-formed QGeoCoordinate that is nowhere on earth. Nothing above
    // catches it — no vertex FAILED and the exterior is not empty — so such a
    // geometry used to be emitted and to SPEND a cap slot, only for the display
    // layer to reject it on the same test one step later. On a multi-layer
    // container an SRS-less layer could therefore exhaust the cap on shapes that
    // draw nothing and starve a good layer behind it. Dropping it here is the same
    // rule as geometries_with_empty_exterior, applied to the same question asked
    // one step earlier; the count is what keeps the operator's status honest, since
    // the dropped geometry no longer reaches the layer to be counted there.
    int geometries_without_placeable_vertex = 0;
    // [camp#22 round-10 suggestion] Features that carry NO GEOMETRY AT ALL —
    // `OGRFeature::GetGeometryRef()` came back null — and null PARTS of a Multi*
    // collection, which are the same loss reached one level down (OGR offers
    // nothing there that would tell the two apart).
    //
    // This is not a theoretical row either: a CSV opened without usable X/Y (or
    // WKT) columns has a null geometry on EVERY feature, and CSV, GML and DXF are
    // all on this parser's allowed-driver list. It was the last geometry class
    // dropped with no counter behind it — the condition that returns for it used
    // to be shared with the exhausted geometry budget, which reports itself — so
    // such a file arrived at the caller with zero geometries and zero of
    // everything else, and was reported as an EMPTY FILE. The two verdicts have
    // different remedies ("this file has no rows" vs "tell GDAL which columns hold
    // the coordinates"), so the caller reports this with a note of its own.
    int features_without_geometry = 0;
};

// [camp#22] True when @p coordinate can be placed on the Web-Mercator scene: both
// ordinates finite, latitude within +/-90, longitude within +/-180
// (`QGeoCoordinate::isValid()`).
//
// This is not a theoretical guard. A shapefile shipped without its `.prj` sidecar
// has no spatial reference, so the parser reads its projected eastings and
// northings as degrees — a UTM northing of 4 800 000 becomes "latitude 4800000",
// and `geoToMap()` turns that into a position ~1e17 scene metres away. A single
// such feature poisons the layer's `childrenBoundingRect()` (so fit-to-extent
// flies to nowhere) and the scene's spatial index. A NaN ordinate, which a failed
// coordinate transform produces, is worse: every comparison against it is false,
// and the bounding rect becomes permanently invalid.
//
// [camp#22 round-9 suggestion] It lives HERE, beside the parse, rather than in the
// display layer where it started: the parser applies it too, to decide whether a
// geometry is worth emitting and charging to the geometry cap, and a second copy
// of the rule in two files is a rule that drifts.
//
// [camp#22 round-10 suggestion] WHAT THE GUARD ACTUALLY BOUNDS, stated so the
// paragraph above is not read as more than it is. The test applied to a GEOMETRY
// is `hasPlaceableCoordinate()`, which admits it on ANY ONE placeable vertex — so
// a line carrying one good vertex and 999 UTM northings read as degrees IS
// admitted, and those 999 vertices do reach `childrenBoundingRect()` and the
// scene index. What the guard removes is the geometry NONE of whose vertices is a
// place on the earth: the whole-file case (a `.prj`-less shapefile, a layer with
// no spatial reference), which is the one that actually occurs and the one that
// leaves fit-to-extent useless.
//
// The weaker bound is deliberate. A mixed geometry is a corrupt or mis-projected
// FILE rather than a format CAMP is asked to read, dropping a shape whose vertices
// mostly transformed would lose real data, and there is no per-vertex answer that
// is right for every geometry type (dropping vertices reshapes a line; dropping
// the geometry discards a feature that may be exactly where the operator is
// looking). The NaN an ordinate takes from a FAILED TRANSFORM never gets this far
// at all: `toWgs84()` returns nothing for it, so the vertex is dropped (and
// counted) before a geometry is built. What remains is a file that carries a NaN
// literally in a layer with no transformation, on a geometry whose other vertices
// are fine — admitted here, like any other bad vertex of an admitted geometry.
bool isPlaceable(const QGeoCoordinate &coordinate);

// [camp#22 round-12] The Web-Mercator latitude domain, in DEGREES
// (`web_mercator::maximum_latitude`, 85.0511...). Declared here so the ONE
// constant is shared: `placeableToMap()` clamps to it and `isProjectable()`
// below tests against it, and two spellings of the projection's own limit is a
// limit that drifts.
double webMercatorLatitudeLimit();

// [camp#22 round-12 must-fix] True when @p coordinate is placeable AND lands
// inside the Web-Mercator world WITHOUT being clamped there — the rule a
// consumer needs when the coordinate becomes an EDITABLE position.
//
// `isPlaceable()` deliberately admits latitude +/-90: it is a perfectly valid
// WGS84 coordinate, and the DISPLAY path (`placeableToMap()`) handles it by
// CLAMPING to the projection's limit, which draws a polar feature at the edge of
// the Mercator world rather than losing it — a polar survey line is real data
// this program should show. That is why the polar bound cannot simply be folded
// into `isPlaceable()`: doing so would turn the display path's documented clamp
// into a silent drop.
//
// The MISSION path cannot clamp. `VectorDataset` copies each admitted vertex into
// a Point/LineString/Polygon MissionItem, which is draggable, written to the
// mission file and a candidate for transmission to the robot — and whose
// `GeoGraphicsItem::geoToPixel()` calls `web_mercator::geoToMap()` RAW, with no
// clamp anywhere on that path: latitude 90 projects to y ~ 2.425e8 m, about
// twelve times the world half-extent, blowing out the scene index and
// fit-to-extent exactly as a `.prj`-less shapefile's northings do. Clamping is
// not the alternative either — it would move the waypoint, i.e. quietly answer a
// position the file does not state. So this path DROPS the vertex and counts it
// (see `placeableGeometry()`), which is the same trade that function already
// makes for every other unplaceable vertex.
bool isProjectable(const QGeoCoordinate &coordinate);

// The first vertex of @p geometry's EXTERIOR that `isPlaceable()` admits, or
// nullptr when it has none. This is what a display item is positioned at.
//
// [camp#22 round-3 should-fix] Interior rings deliberately do not qualify. A
// polygon whose exterior is entirely unplaceable but whose HOLE has a valid vertex
// would otherwise be admitted, and the item would be built from the hole alone —
// which `Qt::OddEvenFill` paints as solid fill, turning a hole into a feature in a
// file whose coordinates CAMP has already said it cannot place. Points and lines
// have no interior rings, so this reads the same for them.
const QGeoCoordinate *firstPlaceableCoordinate(const ParsedGeometry &geometry);

// True when @p geometry has at least one placeable coordinate in its exterior —
// i.e. when an item built from it would land somewhere real.
bool hasPlaceableCoordinate(const ParsedGeometry &geometry);

// [camp#22 round-11 must-fix] One parsed geometry reduced to the vertices that
// `isProjectable()` admits, for the consumer that CANNOT filter again later.
// (Round 12: `isProjectable()`, not `isPlaceable()` — the editable consumer needs
// the polar bound too, and has no clamp to fall back on. See above.)
struct PlaceableGeometry
{
    // The admitted exterior vertices, in file order. Empty when the geometry
    // contributes nothing (`placeable` is false).
    std::vector<QGeoCoordinate> exterior;
    // Polygon only: the admitted vertices of each interior ring. A ring left with
    // no vertex at all is omitted (and counted in `rings_dropped`) rather than
    // carried as an empty hole.
    std::vector<std::vector<QGeoCoordinate>> interiorRings;
    // Vertices rejected, across the exterior and every interior ring.
    int vertices_dropped = 0;
    // Interior rings omitted because no vertex of theirs is placeable.
    int rings_dropped = 0;
    // False when no exterior vertex survived: nothing should be built at all.
    bool placeable = false;
};

// WHY THIS EXISTS, and why it is not the same question `hasPlaceableCoordinate()`
// answers. That test admits a geometry on ANY ONE placeable vertex, and the
// parser applies it (`geometries_without_placeable_vertex`) — a deliberately weak
// bound, justified on the DISPLAY path because `VectorFeatureItem` filters again
// per vertex before it projects anything into the scene.
//
// The parser has a second production consumer that does not: `VectorDataset`
// (File > Open Geometry) copies every emitted vertex into `Point`/`LineString`/
// `Polygon` MISSION ITEMS, which are draggable, saved into the mission file and
// candidates for transmission to the robot (ADR-0016 Context). So a `.prj`-less
// shapefile whose vertices are mostly UTM northings read as degrees imported one
// good vertex and a trail of waypoints that are nowhere on earth. Documenting the
// weak bound (round 9) did not protect the consumer that has no second filter;
// this is that filter, applied by the consumer.
//
// It lives HERE, beside `isPlaceable()` and for the same reason that test does:
// two copies of the placement rule in two files is a rule that drifts. The PARSER
// still does not apply it — ADR-0016 D4 keeps the parse faithful and leaves
// placement policy to the consumer — this is a helper the consumer calls.
//
// THE TRADE, stated because it differs from the one the parser makes: dropping
// vertices RESHAPES a line or a ring, and the header above gives that as a reason
// the parser does not do it. On this path the alternative is worse: an editable,
// persisted, transmittable waypoint at a coordinate that does not exist. A
// mis-projected file is corrupt input either way; what a mission item must never
// hold is a position no vessel can be sent to. The counts are what keep it
// honest — the caller reports them rather than filtering in silence.
PlaceableGeometry placeableGeometry(const ParsedGeometry &geometry);

// Parse every layer of an already-open OGR dataset into WGS84 plain data.
//
// Geometry coverage: Point / LineString / Polygon, their Multi* collections and
// the heterogeneous wkbGeometryCollection, each matched after wkbFlatten() so
// every 25D/Z/M/ZM variant (a GeoJSON point with an elevation is wkbPoint25D)
// reaches the same case as its 2D form. A geometry type that is still not handled
// (the curve types) is skipped and counted in ParseDiagnostics::geometries_unhandled
// — never silently dropped — and reported in ONE qWarning per layer naming the
// count and the first type seen.
//
// Coordinates: each point is transformed individually and the per-point success
// flag is CHECKED. OGR leaves a point that failed to transform at HUGE_VAL, so an
// unchecked transform quietly emits a coordinate 1.7e308 degrees from anywhere;
// failed points are dropped and counted instead. Coordinates that transform (or
// need no transform) are passed through as the file states them — deciding
// whether a coordinate can be PLACED on the scene belongs to the display layer,
// which is the one that knows what "placeable" means (see
// camp::vector::isPlaceable).
//
// [#152] For each layer this creates an OGRCoordinateTransformation and, per
// geometry, OGRPointIterators — and DESTROYS every one of them before
// returning (the transformation at end-of-layer, each iterator at end-of-use).
// OGRFeatures are freed via DestroyFeature. The caller retains ownership of
// `dataset` (open/close is the caller's responsibility); this function opens no
// dataset of its own.
//
// @param diagnostics  optional; filled in with what was skipped and why. A
//                     supplied object is RESET first, so it describes this parse
//                     alone — reusing one across parses cannot carry a counter or
//                     an `aborted`/`geometry_cap_reached` flag into the next.
std::vector<ParsedLayer> parseVectorLayers(GDALDataset *dataset,
                                           const ParseOptions &options = ParseOptions(),
                                           ParseDiagnostics *diagnostics = nullptr);

}  // namespace camp::vector

#endif  // CAMP_VECTOR_PARSE_H
