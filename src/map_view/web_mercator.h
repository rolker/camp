#ifndef WEB_MERCATOR_H
#define WEB_MERCATOR_H

#include <QGeoCoordinate>
#include <cmath>

// Functions to convert between WGS84 coordinates and
// web mercator coordinates.
//
// https://en.wikipedia.org/wiki/Web_Mercator_projection
// https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
//
// In our case, map coordinates are equivalent to pixel coordinates in
// the single zoom level 0 tile that coveres the earth, so they range from 0
// to 256.
namespace web_mercator
{

// At the poles, y diverges so latitudes must be truncated.
// Web mercator uses the following (about 85.0511 degrees)
// resulting in a square aspect ratio.
// Maximum latitude in radians.
constexpr double maximum_latitude = 1.484422229745332367; //atan(sinh(M_PI))

// Size in pixels of tiles used with web mercator projections.
// At zoom level 0, one square tile covers the earth from 0 latitude
// to 0 latitude and maximum_latitude to -maximum_latitude.
constexpr int tile_size = 256;

// Size of a zoom level 0 pixel at the equator in meters.
constexpr double meters_per_pixel_at_equator = 156543.03;

// Convert a point in map coordinates to a WGS84 position.
QGeoCoordinate mapToGeo(const QPointF& point);

// Convert a WGS84 point to map coordinates.
QPointF geoToMap(const QGeoCoordinate& point);

// Calculates approximate size of a pixel at given point.
double metersPerPixel(const QPointF& point);

} // namespace web_mercator

#endif
