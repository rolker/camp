#include "vector_parse.h"

#include <functional>
#include <limits>
#include <memory>
#include <optional>

#include <QDebug>

#include <gdal_priv.h>
#include <ogrsf_frmts.h>

namespace camp::vector
{

namespace
{

// RAII deleter for OGRCoordinateTransformation, so the per-layer transformation
// frees on every path (including an exception mid feature loop) — parity with
// the dataset's gdal_closer in VectorDataset::open.
struct OGRCTDeleter
{
    void operator()(OGRCoordinateTransformation *ct) const
    {
        if(ct)
            OGRCoordinateTransformation::DestroyCT(ct);
    }
};

// Convert one OGR (x, y) pair to a WGS84 QGeoCoordinate.
//
// [camp#22] The axis convention is the one the Point path has always used, now
// applied at EVERY site: an UNTRANSFORMED source is read in traditional GIS
// order, so x is longitude and y is latitude; a TRANSFORMED one comes back in
// the target SRS's authority order (the WGS84 target below is built with
// SetWellKnownGeogCS and no traditional-order override, so EPSG:4326 authority
// order applies), where the first ordinate is latitude. readRing used to build
// QGeoCoordinate(getX(), getY()) on BOTH branches, so line and polygon vertices
// from a source with no spatial reference came out lat/lon swapped while points
// from the same file did not.
//
// [camp#22] The per-point transform result is CHECKED. OGRCoordinateTransformation
// leaves a point it could not transform at HUGE_VAL (1.7e308) and reports it in
// pabSuccess; discarding that flag emitted a coordinate 1.7e308 degrees from
// anywhere, which then propagated into the scene extent. A failed point is
// dropped — nullopt — and counted by the caller.
std::optional<QGeoCoordinate> toWgs84(double x, double y,
                                      OGRCoordinateTransformation *unprojectTransformation)
{
    if(unprojectTransformation)
    {
        int succeeded = FALSE;
        unprojectTransformation->Transform(1, &x, &y, nullptr, &succeeded);
        if(!succeeded)
            return std::nullopt;
        return QGeoCoordinate(x, y);   // transformed: (latitude, longitude)
    }
    return QGeoCoordinate(y, x);       // untransformed: x = longitude, y = latitude
}

// [camp#22] What is left of the parse's budget, threaded THROUGH the recursion.
//
// The cap and the abort predicate are polled per FEATURE by the caller's loop,
// but a single feature can be a MultiPolygon or a nested GeometryCollection with
// hundreds of thousands of parts, each one emitting a ParsedGeometry and a copy
// of the feature's attribute map before control ever returns to that loop. The
// per-feature poll therefore bounds neither the memory overshoot (one feature's
// worth) nor — the one that matters — the ABORT LATENCY, and bounding abort
// latency is the whole reason the predicate exists: VectorLayer's destructor
// joins this worker on the GUI thread.
//
// `remaining` counts geometries this parse may still emit (SIZE_MAX when
// ParseOptions::max_geometries is unlimited); `aborted` is ParseOptions::aborted.
//
// `input_remaining` is an OUTPUT: set when a collection's part loop stopped with
// parts still unread, so the caller can tell "the cap fell exactly at the end of
// this feature" from "the cap cut this feature short". It is what keeps
// ParseDiagnostics::geometry_cap_reached from claiming an unread remainder that
// does not exist.
struct ParseBudget
{
    size_t remaining = std::numeric_limits<size_t>::max();
    std::function<bool()> aborted;
    bool input_remaining = false;

    bool exhausted() const { return remaining == 0 || (aborted && aborted()); }
    void spend()
    {
        if(remaining != std::numeric_limits<size_t>::max() && remaining > 0)
            --remaining;
    }
};

// Read one ring's vertices, transforming to WGS84, and destroy the iterator.
// [#152] The OGRPointIterator is always destroyed here — at every call site,
// including the interior-ring loop where the original code reassigned `pi` per
// ring and so leaked all but (at most) one.
//
// [camp#22] The budget is polled INSIDE the vertex loop. Polling it per geometry
// and per collection part bounds abort latency by the number of parts, which says
// nothing about a single LineString or ring carrying millions of vertices — the
// same failure the per-feature poll had one level up, one level down. The
// destructor joins this worker on the GUI thread, so an unbounded stretch here is
// an unbounded freeze there.
//
// Polled every kVertexPollInterval vertices rather than every vertex: the
// predicate takes a mutex (ParseOptions::aborted reads the layer's abort flag),
// so a per-vertex poll would cost more than the parse. 1024 vertices is well
// inside a frame on any machine that can run CAMP, and far below the part counts
// the coarser polls already bound.
//
// A truncated ring is SAFE, not a wrong shape on screen — but only because the
// feature-boundary check below tests the abort predicate BEFORE the geometry cap
// (see the round-4 comment at that site). The abort that truncated the ring is
// still raised at that check, which sets ParseDiagnostics::aborted, and
// VectorLayer::loadFinished() discards the whole parse result on that flag. Had
// the cap been tested first, a feature that both crossed the cap and carried a
// truncated ring would have returned cap-reached with aborted unset, and the
// partial shape would have been drawn. Nothing partial is drawn as long as that
// ordering holds.
constexpr int kVertexPollInterval = 1024;

std::vector<QGeoCoordinate> readRing(const OGRCurve *ring,
                                    OGRCoordinateTransformation *unprojectTransformation,
                                    ParseDiagnostics &diagnostics,
                                    ParseBudget &budget)
{
    std::vector<QGeoCoordinate> points;
    if(!ring)
        return points;
    // One poll per ring as well as one per kVertexPollInterval vertices: a polygon
    // with a very large number of SHORT interior rings never reaches the in-loop
    // poll. The interior-ring loop itself breaks on the ABORT flag only, never on
    // the geometry cap (see the wkbPolygon case) — breaking it on the cap would
    // drop the holes of a polygon that is otherwise drawn in full.
    if(budget.aborted && budget.aborted())
        return points;
    OGRPointIterator *pi = ring->getPointIterator();
    OGRPoint p;
    int since_poll = 0;
    while(pi->getNextPoint(&p))
    {
        if(++since_poll >= kVertexPollInterval)
        {
            since_poll = 0;
            if(budget.aborted && budget.aborted())
                break;
        }
        const std::optional<QGeoCoordinate> coordinate =
            toWgs84(p.getX(), p.getY(), unprojectTransformation);
        if(coordinate)
            points.push_back(*coordinate);
        else
            ++diagnostics.points_dropped;
    }
    OGRPointIterator::destroy(pi);
    return points;
}

// [camp#22] Append the geometry (and, for a Multi* collection, each of its
// parts) to `out`, copying `attributes` onto every emitted part.
//
// The type switch is on wkbFlatten(getGeometryType()) so the 25D/Z/M/ZM variants
// land on their 2D case instead of the old `default: break` — a GeoJSON point
// carrying an elevation is wkbPoint25D and used to vanish. Multi* collections
// recurse one level per part, which also covers e.g. a MultiPolygon25D.
//
// `budget` is checked before every part of a collection and before every emission,
// so a huge multi-part feature stops mid-feature rather than running to its end.
void appendGeometry(const OGRGeometry *geometry,
                    const QMap<QString, QVariant> &attributes,
                    OGRCoordinateTransformation *unprojectTransformation,
                    std::vector<ParsedGeometry> &out,
                    ParseDiagnostics &diagnostics,
                    ParseBudget &budget)
{
    if(!geometry || budget.exhausted())
        return;

    switch(wkbFlatten(geometry->getGeometryType()))
    {
    case wkbPoint:
    {
        const OGRPoint *op = geometry->toPoint();
        if(op)
        {
            ParsedGeometry g;
            g.type = ParsedGeometry::Point;
            g.attributes = attributes;
            const std::optional<QGeoCoordinate> coordinate =
                toWgs84(op->getX(), op->getY(), unprojectTransformation);
            if(!coordinate)
            {
                // The one point this feature had would not transform; emitting the
                // geometry anyway would produce an empty Point.
                ++diagnostics.points_dropped;
                break;
            }
            g.exterior.push_back(*coordinate);
            out.push_back(std::move(g));
            budget.spend();
        }
        break;
    }
    case wkbLineString:
    {
        const OGRLineString *ols = geometry->toLineString();
        if(ols)
        {
            ParsedGeometry g;
            g.type = ParsedGeometry::LineString;
            g.attributes = attributes;
            g.exterior = readRing(ols, unprojectTransformation, diagnostics, budget);
            out.push_back(std::move(g));
            budget.spend();
        }
        break;
    }
    case wkbPolygon:
    {
        const OGRPolygon *op = geometry->toPolygon();
        // Only emit a polygon when it has an exterior ring — there is no outline
        // without one. [camp#22] COUNT the drop: every ParseDiagnostics field
        // exists so a caller can report what was deliberately left out, and an
        // empty polygon used to vanish with nothing said.
        if(op && !op->getExteriorRing())
        {
            ++diagnostics.polygons_without_exterior_ring;
        }
        else if(op)
        {
            ParsedGeometry g;
            g.type = ParsedGeometry::Polygon;
            g.attributes = attributes;
            g.exterior = readRing(op->getExteriorRing(), unprojectTransformation, diagnostics,
                                  budget);
            for(int ringNum = 0; ringNum < op->getNumInteriorRings(); ++ringNum)
            {
                // [camp#22] Break on ABORT, the way the geometry-collection part
                // loop below does — a polygon with very many short interior rings
                // otherwise spends one mutex-guarded predicate call per ring after
                // the abort has already been raised. Deliberately NOT
                // budget.exhausted(): that also fires at the geometry cap, which
                // would drop the holes of a polygon whose exterior is drawn in
                // full.
                if(budget.aborted && budget.aborted())
                    break;
                g.interiorRings.push_back(
                    readRing(op->getInteriorRing(ringNum), unprojectTransformation, diagnostics,
                             budget));
            }
            out.push_back(std::move(g));
            budget.spend();
        }
        break;
    }
    case wkbMultiPoint:
    case wkbMultiLineString:
    case wkbMultiPolygon:
    // [camp#22] A heterogeneous wkbGeometryCollection is handled by the SAME
    // recursion: OGRGeometryCollection is the base of the Multi* classes, each
    // part comes back through this switch on its own type, and a nested
    // collection recurses again. It used to be warn-and-dropped one line below
    // the recursion that already covered it — the very data-loss class this
    // parser's 25D/Multi* work set out to close. KML in particular emits a
    // MultiGeometry (= wkbGeometryCollection) whenever a placemark mixes a point
    // with its outline.
    case wkbGeometryCollection:
    {
        const OGRGeometryCollection *collection = geometry->toGeometryCollection();
        if(collection)
            for(int part = 0; part < collection->getNumGeometries(); ++part)
            {
                // The budget is what makes a 200 000-part MultiPolygon abortable:
                // without this the recursion runs the feature to its end and the
                // destructor's join waits for all of it.
                if(budget.exhausted())
                {
                    // Parts of this feature were never read — input remains
                    // whatever the rest of the file holds.
                    budget.input_remaining = true;
                    break;
                }
                appendGeometry(collection->getGeometryRef(part), attributes,
                               unprojectTransformation, out, diagnostics, budget);
            }
        break;
    }
    default:
        // Documented skip (the curve types): the formats this parser claims to
        // read do not normally emit these, and a silent drop is what made the
        // 25D/Multi* gap invisible for so long.
        //
        // [camp#22 round-4 should-fix] COUNTED here, REPORTED once per layer by
        // the caller — the way points_dropped and polygons_without_exterior_ring
        // already are. This used to qWarning() per geometry, and an unhandled
        // geometry does not spend the budget, so max_geometries bounded nothing
        // here: a large file of curve types emitted unbounded log I/O while the
        // worker read all of it. The TYPE NAME is kept (first one seen), which is
        // the part of the message that was worth having.
        ++diagnostics.geometries_unhandled;
        if(diagnostics.first_unhandled_geometry_type.isEmpty())
            if(const char *name = OGRGeometryTypeToName(geometry->getGeometryType()))
                diagnostics.first_unhandled_geometry_type = QString::fromUtf8(name);
        break;
    }
}

// [camp#22] Read every set field of `feature` into a name-keyed QVariant map.
// See the type-mapping table on ParsedGeometry::attributes. An unset or NULL
// field is omitted so "no value" stays distinguishable from an empty value.
QMap<QString, QVariant> readAttributes(const OGRFeature *feature)
{
    QMap<QString, QVariant> attributes;
    const OGRFeatureDefn *defn = feature->GetDefnRef();
    if(!defn)
        return attributes;
    for(int field = 0; field < defn->GetFieldCount(); ++field)
    {
        const OGRFieldDefn *field_defn = defn->GetFieldDefn(field);
        if(!field_defn)
            continue;
        if(!feature->IsFieldSetAndNotNull(field))
            continue;
        const QString name = QString::fromUtf8(field_defn->GetNameRef());
        switch(field_defn->GetType())
        {
        case OFTInteger:
            attributes[name] = feature->GetFieldAsInteger(field);
            break;
        case OFTInteger64:
            attributes[name] = static_cast<qlonglong>(feature->GetFieldAsInteger64(field));
            break;
        case OFTReal:
            attributes[name] = feature->GetFieldAsDouble(field);
            break;
        default:
            attributes[name] = QString::fromUtf8(feature->GetFieldAsString(field));
            break;
        }
    }
    return attributes;
}

}  // namespace

std::vector<ParsedLayer> parseVectorLayers(GDALDataset *dataset,
                                           const ParseOptions &options,
                                           ParseDiagnostics *diagnostics)
{
    ParseDiagnostics local;
    ParseDiagnostics &diag = diagnostics ? *diagnostics : local;
    // [camp#22] RESET a supplied object before binding to it. The header documents
    // the parameter as "filled in with what was skipped and why" — a description
    // of THIS parse — but every field is accumulated into, so a caller that reuses
    // one object across two parses would carry `aborted`, `geometry_cap_reached`,
    // `layers_total` and every counter into the second result and see an uncapped
    // parse reported as capped. No live caller reuses one (VectorLayer::load()
    // builds a fresh LoadResult per load), so this closes a latent API trap rather
    // than a live defect — at the cost of one assignment per parse.
    diag = ParseDiagnostics();

    // [camp#22 / #213] One cheap predicate, polled per layer AND per feature, so a
    // destructor that aborts the worker is joined in feature time rather than
    // file time. Reading it once before the open — which is what this used to do —
    // guarantees nothing: the whole parse happens after that read.
    const auto aborted = [&options]()
    {
        return options.aborted && options.aborted();
    };

    std::vector<ParsedLayer> result;
    // [camp#22] Geometries emitted so far, across every layer — the quantity
    // ParseOptions::max_geometries bounds. Tracked incrementally rather than
    // re-summed per feature so the check stays O(1) on a million-feature file.
    size_t emitted = 0;
    if(!dataset)
        return result;

    // [camp#22] Is there anything left to read once the geometry cap has been
    // reached? BOUNDED by construction: the current feature's own unread parts
    // (reported by the budget), then ONE lookahead feature on the current layer,
    // then at most one feature per remaining layer. That is the smallest amount of
    // reading that can distinguish "stopped at the cap with the file unread" from
    // "the file happened to hold exactly max_geometries geometries" — and the
    // status line built on this flag is worth that much I/O.
    //
    // A remaining layer whose spatial reference yields no transformation to WGS84
    // counts as input remaining: it is data this parse did not read, and the
    // layers_failed diagnostic is what says why a layer was skipped.
    const auto moreInputRemains = [dataset](OGRLayer *current, int layerIndex,
                                            const ParseBudget &budget)
    {
        if(budget.input_remaining)
            return true;
        if(current)
            if(OGRFeature *next = current->GetNextFeature())
            {
                OGRFeature::DestroyFeature(next);
                return true;
            }
        for(int j = layerIndex + 1; j < dataset->GetLayerCount(); ++j)
        {
            OGRLayer *remaining = dataset->GetLayer(j);
            if(!remaining)
                continue;
            remaining->ResetReading();
            if(OGRFeature *f = remaining->GetNextFeature())
            {
                OGRFeature::DestroyFeature(f);
                return true;
            }
        }
        return false;
    };

    // [camp#22 round-7 must-fix] Scopes ParseDiagnostics::first_unhandled_geometry_type
    // to one layer for the duration of that layer's body, then restores the
    // parse-wide first type. Written as a guard rather than a pair of statements
    // because the layer body has two early returns (abort, geometry cap) that
    // would otherwise leave the field holding a layer-scoped value — or nothing at
    // all, if the layer that returned early saw no unhandled geometry and an
    // earlier one did.
    struct LayerUnhandledTypeScope
    {
        ParseDiagnostics &diagnostics;
        QString previous;

        explicit LayerUnhandledTypeScope(ParseDiagnostics &d)
          : diagnostics(d), previous(d.first_unhandled_geometry_type)
        {
            diagnostics.first_unhandled_geometry_type.clear();
        }
        ~LayerUnhandledTypeScope()
        {
            // The parse-wide first type is the earlier one when there was one;
            // otherwise whatever this layer found (possibly still empty).
            if(!previous.isEmpty())
                diagnostics.first_unhandled_geometry_type = previous;
        }

        LayerUnhandledTypeScope(const LayerUnhandledTypeScope &) = delete;
        LayerUnhandledTypeScope &operator=(const LayerUnhandledTypeScope &) = delete;
    };

    for(int i = 0; i < dataset->GetLayerCount(); ++i)
    {
        if(aborted())
        {
            diag.aborted = true;
            return result;
        }

        OGRLayer *layer = dataset->GetLayer(i);
        if(!layer)
            continue;
        ++diag.layers_total;

        // [#152] One transformation per layer, freed by RAII at end-of-layer.
        // The original destroyed none, leaking one PROJ pipeline per layer.
        std::unique_ptr<OGRCoordinateTransformation, OGRCTDeleter> unprojectTransformation;
        if(OGRSpatialReference *projected = layer->GetSpatialRef())
        {
            OGRSpatialReference wgs84;
            wgs84.SetWellKnownGeogCS("WGS84");
            unprojectTransformation.reset(OGRCreateCoordinateTransformation(projected, &wgs84));
            if(!unprojectTransformation)
            {
                // [camp#22] The layer HAS a spatial reference and we could not
                // build a path from it to WGS84 (a missing PROJ grid, an SRS PROJ
                // cannot interpret). Falling through to the untransformed branch —
                // what this used to do — reads the layer's projected metres as
                // degrees and places every feature about 1e17 metres from where it
                // belongs, with nothing said. FAIL the layer instead, loudly.
                ++diag.layers_failed;
                const char *name = projected->GetName();
                qWarning() << "camp::vector::parseVectorLayers: skipping layer"
                           << layer->GetName()
                           << "- no coordinate transformation to WGS84 could be built from its"
                           << "spatial reference" << (name ? name : "(unnamed)")
                           << "; its coordinates cannot be interpreted as latitude/longitude";
                continue;
            }
        }

        ParsedLayer parsed;
        parsed.name = layer->GetName();

        // [camp#22 round-7 must-fix] The type NAME in the summary line is
        // per-layer, exactly like the counts it sits beside.
        // `first_unhandled_geometry_type` accumulates across the whole parse, so
        // layer 2 of a mixed dataset used to print layer 1's type next to its own
        // per-layer count. Snapshot and clear it at layer entry the way
        // `unhandled_before` snapshots the count; the guard restores the
        // parse-wide first type on EVERY exit from the layer body — including the
        // early returns at the abort check and the geometry cap — so the field
        // keeps the whole-parse meaning its header documents for callers.
        LayerUnhandledTypeScope unhandled_type_scope(diag);
        const int dropped_before = diag.points_dropped;
        const int polygons_dropped_before = diag.polygons_without_exterior_ring;
        const int unhandled_before = diag.geometries_unhandled;
        layer->ResetReading();
        OGRFeature *feature = layer->GetNextFeature();
        while(feature)
        {
            const size_t before = parsed.geometries.size();
            // The remaining budget, recomputed per feature and spent INSIDE the
            // recursion so a multi-part feature stops where the cap falls rather
            // than overshooting to the end of the feature and being trimmed.
            ParseBudget budget;
            if(options.max_geometries > 0)
            {
                const size_t cap = static_cast<size_t>(options.max_geometries);
                budget.remaining = emitted < cap ? cap - emitted : 0;
            }
            budget.aborted = options.aborted;
            appendGeometry(feature->GetGeometryRef(), readAttributes(feature),
                           unprojectTransformation.get(), parsed.geometries, diag, budget);
            OGRFeature::DestroyFeature(feature);
            emitted += parsed.geometries.size() - before;
            // [camp#22 round-4 must-fix] ABORT IS CHECKED BEFORE THE CAP, and the
            // order is load-bearing.
            //
            // The in-loop polls inside readRing() and appendGeometry() TRUNCATE
            // what they are building when the abort flag rises; the only thing
            // that keeps a truncated ring off the screen is ParseDiagnostics::
            // aborted, which VectorLayer::loadFinished() discards the whole result
            // on. When the cap was tested first, a feature that both crossed
            // max_geometries and carried a ring truncated by the vertex poll
            // returned with geometry_cap_reached set and aborted UNSET — a partial
            // shape presented as a complete one. Testing the abort first means a
            // truncated parse is always reported as aborted.
            //
            // A parse that hits both therefore reports aborted, not cap-reached,
            // and is NOT trimmed to exactly max_geometries: an aborted result is
            // partial by construction and its caller throws it away. The
            // "exactly max_geometries" contract is about the cap-reached return
            // below, which is the one a caller consumes.
            if(aborted())
            {
                // Return what was parsed so far. The caller knows it is partial
                // (diagnostics.aborted) and, in the case this exists for, is about
                // to throw it away anyway.
                diag.aborted = true;
                result.push_back(std::move(parsed));
                return result;
            }
            // [camp#22] The cap is checked HERE, per feature, so the rest of the
            // file is never read: the caller's cap on the items it builds bounds
            // the GUI thread, but the memory the parse itself takes is only
            // bounded by stopping the parse.
            //
            // The trim below is now a belt-and-braces no-op — the budget threaded
            // into appendGeometry stops the recursion exactly at the cap, so a
            // multi-part feature no longer overshoots it. It is kept because it is
            // the invariant this contract promises ("exactly max_geometries are
            // returned") and it costs one subtraction per file.
            if(options.max_geometries > 0 &&
               emitted >= static_cast<size_t>(options.max_geometries))
            {
                const size_t excess = emitted - static_cast<size_t>(options.max_geometries);
                if(excess > 0)
                    parsed.geometries.resize(parsed.geometries.size() - excess);
                // [camp#22 round-3 should-fix] The flag means "the rest of the
                // file was NOT READ", and VectorLayer puts exactly that sentence
                // in the Layers tab. A file holding exactly max_geometries
                // geometries reaches this branch having been read in full, so
                // setting the flag unconditionally made the one status line this
                // design leans on claim a partial read that never happened.
                // Establish that input actually remains first — see
                // moreInputRemains(), which is bounded at one feature per
                // remaining layer.
                diag.geometry_cap_reached = moreInputRemains(layer, i, budget);
                result.push_back(std::move(parsed));
                return result;
            }
            feature = layer->GetNextFeature();
        }

        if(diag.polygons_without_exterior_ring > polygons_dropped_before)
            qWarning() << "camp::vector::parseVectorLayers: layer" << layer->GetName()
                       << "- dropped"
                       << (diag.polygons_without_exterior_ring - polygons_dropped_before)
                       << "polygon(s) with no exterior ring";

        if(diag.points_dropped > dropped_before)
            qWarning() << "camp::vector::parseVectorLayers: layer" << layer->GetName()
                       << "- dropped" << (diag.points_dropped - dropped_before)
                       << "point(s) whose coordinate transformation failed";

        if(diag.geometries_unhandled > unhandled_before)
            qWarning() << "camp::vector::parseVectorLayers: layer" << layer->GetName()
                       << "- skipped" << (diag.geometries_unhandled - unhandled_before)
                       << "geometry(ies) of an unhandled type (first:"
                       << (diag.first_unhandled_geometry_type.isEmpty()
                               ? QStringLiteral("unnamed")
                               : diag.first_unhandled_geometry_type)
                       << ")";

        // unprojectTransformation's RAII deleter frees it here.
        result.push_back(std::move(parsed));
    }

    return result;
}

}  // namespace camp::vector
