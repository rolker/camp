#ifndef CAMP_VECTOR_PARSE_H
#define CAMP_VECTOR_PARSE_H

#include <vector>

#include <QGeoCoordinate>
#include <QString>

class GDALDataset;

namespace camp::vector
{

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
};

struct ParsedLayer
{
    QString name;
    std::vector<ParsedGeometry> geometries;
};

// Parse every layer of an already-open OGR dataset into WGS84 plain data.
//
// [#152] For each layer this creates an OGRCoordinateTransformation and, per
// geometry, OGRPointIterators — and DESTROYS every one of them before
// returning (the transformation at end-of-layer, each iterator at end-of-use).
// OGRFeatures are freed via DestroyFeature. The caller retains ownership of
// `dataset` (open/close is the caller's responsibility); this function opens no
// dataset of its own.
std::vector<ParsedLayer> parseVectorLayers(GDALDataset *dataset);

}  // namespace camp::vector

#endif  // CAMP_VECTOR_PARSE_H
