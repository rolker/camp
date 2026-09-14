#include "vector_parse.h"

#include <memory>

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
QGeoCoordinate toWgs84(double x, double y, OGRCoordinateTransformation *unprojectTransformation)
{
    if(unprojectTransformation)
    {
        unprojectTransformation->Transform(1, &x, &y);
        return QGeoCoordinate(x, y);   // transformed: (latitude, longitude)
    }
    return QGeoCoordinate(y, x);       // untransformed: x = longitude, y = latitude
}

// Read one ring's vertices, transforming to WGS84, and destroy the iterator.
// [#152] The OGRPointIterator is always destroyed here — at every call site,
// including the interior-ring loop where the original code reassigned `pi` per
// ring and so leaked all but (at most) one.
std::vector<QGeoCoordinate> readRing(const OGRCurve *ring, OGRCoordinateTransformation *unprojectTransformation)
{
    std::vector<QGeoCoordinate> points;
    if(!ring)
        return points;
    OGRPointIterator *pi = ring->getPointIterator();
    OGRPoint p;
    while(pi->getNextPoint(&p))
        points.push_back(toWgs84(p.getX(), p.getY(), unprojectTransformation));
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
                    std::vector<ParsedGeometry> &out)
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
            g.exterior.push_back(toWgs84(op->getX(), op->getY(), unprojectTransformation));
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
            g.exterior = readRing(ols, unprojectTransformation);
            out.push_back(std::move(g));
        }
        break;
    }
    case wkbPolygon:
    {
        const OGRPolygon *op = geometry->toPolygon();
        // Match the original: only emit a polygon when it has an exterior ring.
        if(op && op->getExteriorRing())
        {
            ParsedGeometry g;
            g.type = ParsedGeometry::Polygon;
            g.attributes = attributes;
            g.exterior = readRing(op->getExteriorRing(), unprojectTransformation);
            for(int ringNum = 0; ringNum < op->getNumInteriorRings(); ++ringNum)
                g.interiorRings.push_back(readRing(op->getInteriorRing(ringNum), unprojectTransformation));
            out.push_back(std::move(g));
        }
        break;
    }
    case wkbMultiPoint:
    case wkbMultiLineString:
    case wkbMultiPolygon:
    {
        const OGRGeometryCollection *collection = geometry->toGeometryCollection();
        if(collection)
            for(int part = 0; part < collection->getNumGeometries(); ++part)
                appendGeometry(collection->getGeometryRef(part), attributes,
                               unprojectTransformation, out);
        break;
    }
    default:
        // Documented, LOGGED skip (wkbGeometryCollection, the curve types): the
        // formats this parser claims to read do not normally emit these, and a
        // silent drop is what made the 25D/Multi* gap invisible for so long.
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

std::vector<ParsedLayer> parseVectorLayers(GDALDataset *dataset)
{
    std::vector<ParsedLayer> result;
    if(!dataset)
        return result;

    for(int i = 0; i < dataset->GetLayerCount(); ++i)
    {
        OGRLayer *layer = dataset->GetLayer(i);
        if(!layer)
            continue;

        // [#152] One transformation per layer, freed by RAII at end-of-layer.
        // The original destroyed none, leaking one PROJ pipeline per layer.
        std::unique_ptr<OGRCoordinateTransformation, OGRCTDeleter> unprojectTransformation;
        if(OGRSpatialReference *projected = layer->GetSpatialRef())
        {
            OGRSpatialReference wgs84;
            wgs84.SetWellKnownGeogCS("WGS84");
            unprojectTransformation.reset(OGRCreateCoordinateTransformation(projected, &wgs84));
        }

        ParsedLayer parsed;
        parsed.name = layer->GetName();

        layer->ResetReading();
        OGRFeature *feature = layer->GetNextFeature();
        while(feature)
        {
            appendGeometry(feature->GetGeometryRef(), readAttributes(feature),
                           unprojectTransformation.get(), parsed.geometries);
            OGRFeature::DestroyFeature(feature);
            feature = layer->GetNextFeature();
        }

        // unprojectTransformation's RAII deleter frees it here.
        result.push_back(std::move(parsed));
    }

    return result;
}

}  // namespace camp::vector
