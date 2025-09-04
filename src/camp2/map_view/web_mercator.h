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
// Web mercator coordinates are roughly meters relative to 
// Lat/Lon 0,0.
//
namespace web_mercator
{

// At the poles, y diverges so latitudes must be truncated.
// Web mercator uses the following (about 85.0511 degrees)
// resulting in a square aspect ratio.
// Maximum latitude in radians.
constexpr double maximum_latitude = 1.484422229745332367; //atan(sinh(M_PI))

constexpr double earth_radius_at_equator = 6378137;

// Convert a point in map coordinates to a WGS84 position.
QGeoCoordinate mapToGeo(const QPointF& point);

// Convert a WGS84 point to map coordinates.
QPointF geoToMap(const QGeoCoordinate& point);

// Calculates approximate size of a map unit at given point.
double metersPerUnit(const QPointF& point);

// WKT String for ESPG:3857 WGS 84 / Pseudo-Mercator
// https://epsg.io/3857
constexpr char wkt[] = "PROJCS[\"WGS 84 / Pseudo-Mercator\",GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]],PROJECTION[\"Mercator_1SP\"],PARAMETER[\"central_meridian\",0],PARAMETER[\"scale_factor\",1],PARAMETER[\"false_easting\",0],PARAMETER[\"false_northing\",0],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AXIS[\"Easting\",EAST],AXIS[\"Northing\",NORTH],EXTENSION[\"PROJ4\",\"+proj=merc +a=6378137 +b=6378137 +lat_ts=0 +lon_0=0 +x_0=0 +y_0=0 +k=1 +units=m +nadgrids=@null +wktext +no_defs\"],AUTHORITY[\"EPSG\",\"3857\"]]";

} // namespace web_mercator

#endif
