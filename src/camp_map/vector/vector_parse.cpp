#include "vector_parse.h"

#include <memory>

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
    {
        if(unprojectTransformation)
        {
            double x = p.getX();
            double y = p.getY();
            unprojectTransformation->Transform(1, &x, &y);
            p.setX(x);
            p.setY(y);
        }
        points.emplace_back(p.getX(), p.getY());
    }
    OGRPointIterator::destroy(pi);
    return points;
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
            if(OGRGeometry *geometry = feature->GetGeometryRef())
            {
                switch(geometry->getGeometryType())
                {
                case wkbPoint:
                {
                    OGRPoint *op = dynamic_cast<OGRPoint *>(geometry);
                    if(op)
                    {
                        ParsedGeometry g;
                        g.type = ParsedGeometry::Point;
                        // Preserve original coordinate handling exactly: the
                        // untransformed point is (lat=Y, lon=X); a transformed
                        // point uses the (x,y) output as (lat, lon).
                        QGeoCoordinate location(op->getY(), op->getX());
                        if(unprojectTransformation)
                        {
                            double x = op->getX();
                            double y = op->getY();
                            unprojectTransformation->Transform(1, &x, &y);
                            location.setLatitude(x);
                            location.setLongitude(y);
                        }
                        g.exterior.push_back(location);
                        parsed.geometries.push_back(std::move(g));
                    }
                    break;
                }
                case wkbLineString:
                {
                    OGRLineString *ols = dynamic_cast<OGRLineString *>(geometry);
                    if(ols)
                    {
                        ParsedGeometry g;
                        g.type = ParsedGeometry::LineString;
                        g.exterior = readRing(ols, unprojectTransformation.get());
                        parsed.geometries.push_back(std::move(g));
                    }
                    break;
                }
                case wkbPolygon:
                {
                    OGRPolygon *op = dynamic_cast<OGRPolygon *>(geometry);
                    // Match the original: only emit a polygon when it has an
                    // exterior ring.
                    if(op && op->getExteriorRing())
                    {
                        ParsedGeometry g;
                        g.type = ParsedGeometry::Polygon;
                        g.exterior = readRing(op->getExteriorRing(), unprojectTransformation.get());
                        for(int ringNum = 0; ringNum < op->getNumInteriorRings(); ++ringNum)
                            g.interiorRings.push_back(readRing(op->getInteriorRing(ringNum), unprojectTransformation.get()));
                        parsed.geometries.push_back(std::move(g));
                    }
                    break;
                }
                default:
                    break;
                }
            }
            OGRFeature::DestroyFeature(feature);
            feature = layer->GetNextFeature();
        }

        // unprojectTransformation's RAII deleter frees it here.
        result.push_back(std::move(parsed));
    }

    return result;
}

}  // namespace camp::vector
