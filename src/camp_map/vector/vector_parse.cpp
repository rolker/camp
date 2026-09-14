#include "vector_parse.h"

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

// Read one ring's vertices, transforming to WGS84, and destroy the iterator.
// [#152] The OGRPointIterator is always destroyed here — at every call site,
// including the interior-ring loop where the original code reassigned `pi` per
// ring and so leaked all but (at most) one.
std::vector<QGeoCoordinate> readRing(const OGRCurve *ring,
                                    OGRCoordinateTransformation *unprojectTransformation,
                                    ParseDiagnostics &diagnostics)
{
    std::vector<QGeoCoordinate> points;
    if(!ring)
        return points;
    OGRPointIterator *pi = ring->getPointIterator();
    OGRPoint p;
    while(pi->getNextPoint(&p))
    {
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
void appendGeometry(const OGRGeometry *geometry,
                    const QMap<QString, QVariant> &attributes,
                    OGRCoordinateTransformation *unprojectTransformation,
                    std::vector<ParsedGeometry> &out,
                    ParseDiagnostics &diagnostics)
{
    if(!geometry)
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
            g.exterior = readRing(ols, unprojectTransformation, diagnostics);
            out.push_back(std::move(g));
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
            g.exterior = readRing(op->getExteriorRing(), unprojectTransformation, diagnostics);
            for(int ringNum = 0; ringNum < op->getNumInteriorRings(); ++ringNum)
                g.interiorRings.push_back(
                    readRing(op->getInteriorRing(ringNum), unprojectTransformation, diagnostics));
            out.push_back(std::move(g));
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
                appendGeometry(collection->getGeometryRef(part), attributes,
                               unprojectTransformation, out, diagnostics);
        break;
    }
    default:
        // Documented, LOGGED skip (the curve types): the formats this parser
        // claims to read do not normally emit these, and a silent drop is what
        // made the 25D/Multi* gap invisible for so long.
        ++diagnostics.geometries_unhandled;
        qWarning() << "camp::vector::parseVectorLayers: skipping unhandled geometry type"
                   << OGRGeometryTypeToName(geometry->getGeometryType());
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

        const int dropped_before = diag.points_dropped;
        const int polygons_dropped_before = diag.polygons_without_exterior_ring;
        layer->ResetReading();
        OGRFeature *feature = layer->GetNextFeature();
        while(feature)
        {
            const size_t before = parsed.geometries.size();
            appendGeometry(feature->GetGeometryRef(), readAttributes(feature),
                           unprojectTransformation.get(), parsed.geometries, diag);
            OGRFeature::DestroyFeature(feature);
            emitted += parsed.geometries.size() - before;
            // [camp#22] The cap is checked HERE, per feature, so the rest of the
            // file is never read: the caller's cap on the items it builds bounds
            // the GUI thread, but the memory the parse itself takes is only
            // bounded by stopping the parse. A multi-part feature can carry the
            // total past the cap, so the excess is trimmed and the returned count
            // is exactly the cap.
            if(options.max_geometries > 0 &&
               emitted >= static_cast<size_t>(options.max_geometries))
            {
                const size_t excess = emitted - static_cast<size_t>(options.max_geometries);
                if(excess > 0)
                    parsed.geometries.resize(parsed.geometries.size() - excess);
                diag.geometry_cap_reached = true;
                result.push_back(std::move(parsed));
                return result;
            }
            if(aborted())
            {
                // Return what was parsed so far. The caller knows it is partial
                // (diagnostics.aborted) and, in the case this exists for, is about
                // to throw it away anyway.
                diag.aborted = true;
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

        // unprojectTransformation's RAII deleter frees it here.
        result.push_back(std::move(parsed));
    }

    return result;
}

}  // namespace camp::vector
