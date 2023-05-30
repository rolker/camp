#include "web_mercator.h"
#include <QPointF>
#include <QTransform>

namespace web_mercator
{

double mapToLatitudeRadians(const QPointF& point)
{
  double n = point.y()/earth_radius_at_equator;
  if(n < -M_PI || n > M_PI)
    return std::nan("");
  return atan(0.5 * (exp(n) - exp(-n)));
}

QGeoCoordinate mapToGeo(const QPointF& point)
{
  double lat = 180.0 / M_PI * mapToLatitudeRadians(point);
  double lon = fmod((point.x()/earth_radius_at_equator)*180/M_PI, 360.0);
  if(lon < -180.0)
    lon += 360.0;
  if(lon > 180.0)
    lon -= 360.0;
  return QGeoCoordinate(lat, lon);
}

QPointF geoToMap(const QGeoCoordinate& point)
{
  double x = point.longitude()*(M_PI/180.0)*earth_radius_at_equator;
  double latrad = point.latitude() * M_PI/180.0;
  double y = asinh(tan(latrad))*earth_radius_at_equator;
  return QPointF(x,y);
}

double metersPerUnit(const QPointF& point)
{
  return cos(mapToLatitudeRadians(point));
}

} // namespace web_mercator
