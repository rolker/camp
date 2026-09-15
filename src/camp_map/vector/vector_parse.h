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
    // Polled once per layer and once per feature. When it returns true the parse
    // stops where it is and returns what it has, with ParseDiagnostics::aborted
    // set. Empty (the default) means "never abort".
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
    // the cap. Such a result is also not trimmed to exactly `max_geometries` —
    // it is partial by construction.
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
    int points_dropped = 0;
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
};

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
