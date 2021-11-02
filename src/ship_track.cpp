#include "ship_track.h"
#include <cmath>
#include <QDebug>

ShipTrack::ShipTrack(QGraphicsItem *parentItem):GeoGraphicsItem(parentItem)
{

}

void ShipTrack::drawTriangle(QPainterPath& path, BackgroundRaster* bg, const QGeoCoordinate& location, double heading_degrees, double scale) const
{
  if(std::isnan(heading_degrees))
  {
    QPointF center = geoToPixel(location, bg);
    path.addEllipse(center, 15*scale, 15*scale);
    return;
  }
  QGeoCoordinate tip = location.atDistanceAndAzimuth(15*scale,heading_degrees);
  QGeoCoordinate llcorner = location.atDistanceAndAzimuth(15*scale,heading_degrees-150);
  QGeoCoordinate lrcorner = location.atDistanceAndAzimuth(15*scale,heading_degrees+150);

  QPointF ltip = geoToPixel(tip, bg);
  QPointF lllocal = geoToPixel(llcorner, bg);
  QPointF lrlocal = geoToPixel(lrcorner, bg);

  path.moveTo(ltip);
  path.lineTo(lllocal);
  path.lineTo(lrlocal);
  path.lineTo(ltip);
}

void ShipTrack::drawShipOutline(QPainterPath& path, BackgroundRaster* bg, const QGeoCoordinate& location, double heading_degrees, float dimension_to_bow, float dimension_to_port, float dimension_to_stbd, float dimension_to_stern) const
{
  if(std::isnan(heading_degrees))
  {
    QPointF center = geoToPixel(location, bg);
    float radius = std::max(dimension_to_bow, dimension_to_stern);
    radius = std::max(radius, dimension_to_port);
    radius = std::max(radius, dimension_to_stbd);
    QGeoCoordinate radius_away = location.atDistanceAndAzimuth(radius,0);
    QPointF radius_away_local = geoToPixel(radius_away, bg);
    QPointF radius_local = center-radius_away_local;
    float radius_pixel = sqrt(radius_local.rx()*radius_local.rx()+radius_local.ry()*radius_local.ry());
    path.addEllipse(center, radius_pixel, radius_pixel);
    return;
  }

  float length = dimension_to_bow+dimension_to_stern;
  float width = dimension_to_port+dimension_to_stbd;
  QGeoCoordinate llcorner = location.atDistanceAndAzimuth(dimension_to_port,270+heading_degrees).atDistanceAndAzimuth(dimension_to_stern,180+heading_degrees);
  QGeoCoordinate lrcorner = llcorner.atDistanceAndAzimuth(width,90+heading_degrees);
  QGeoCoordinate urcorner = lrcorner.atDistanceAndAzimuth(length,heading_degrees);
  QGeoCoordinate ulcorner = urcorner.atDistanceAndAzimuth(width,270+heading_degrees);
  QGeoCoordinate rkink = lrcorner.atDistanceAndAzimuth(length*.8,heading_degrees);
  QGeoCoordinate lkink = llcorner.atDistanceAndAzimuth(length*.8,heading_degrees);
  QGeoCoordinate bow = ulcorner.atDistanceAndAzimuth(width/2.0,90+heading_degrees);
  QPointF lllocal = geoToPixel(llcorner, bg);
  QPointF lrlocal = geoToPixel(lrcorner, bg);
  QPointF lkinkl = geoToPixel(lkink, bg);
  QPointF rkinkl = geoToPixel(rkink, bg);
  QPointF bowl = geoToPixel(bow, bg);
  
  path.moveTo(lllocal);
  path.lineTo(lrlocal);
  path.lineTo(rkinkl);
  path.lineTo(bowl);
  path.lineTo(lkinkl);
  path.lineTo(lllocal);
}
