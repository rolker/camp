#include "web_mercator.h"
#include <QPointF>

namespace web_mercator
{

double mapToLatitudeRadians(const QPointF& point)
{
  if(point.y() < 0.0 || point.y() > tile_size)
    return std::nan("");
  double n = M_PI - 2.0 * M_PI * point.y()/double(tile_size);
  return atan(0.5 * (exp(n) - exp(-n)));
}

QGeoCoordinate mapToGeo(const QPointF& point)
{
  double lat = 180.0 / M_PI * mapToLatitudeRadians(point);
  double lon = fmod((point.x()/double(tile_size))*360.0 - 180.0, 360.0);
  if(lon < -180.0)
    lon += 360.0;
  if(lon > 180.0)
    lon -= 360.0;
  return QGeoCoordinate(lat, lon);
}

QPointF geoToMap(const QGeoCoordinate& point)
{
  double x = tile_size*(point.longitude() + 180.0) / 360.0;
  double latrad = point.latitude() * M_PI/180.0;
  double y = tile_size*(1.0 - asinh(tan(latrad)) / M_PI) / 2.0;
  return QPointF(x,y);
}

double metersPerPixel(const QPointF& point)
{
  return meters_per_pixel_at_equator*cos(mapToLatitudeRadians(point));
}

} // namespace web_mercator
