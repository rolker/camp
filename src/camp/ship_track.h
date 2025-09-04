#ifndef CAMP_SHIP_TRACK_H
#define CAMP_SHIP_TRACK_H

#include "geographicsitem.h"

class ShipTrack: public GeoGraphicsItem
{
public:
  ShipTrack(QGraphicsItem *parentItem = nullptr);

protected:
  void drawTriangle(QPainterPath &path, BackgroundRaster* bg, QGeoCoordinate const &location, double heading_degrees, double scale=1.0) const;
  void drawShipOutline(QPainterPath &path, BackgroundRaster* bg, QGeoCoordinate const &location, double heading_degrees, float dimension_to_bow, float dimension_to_port, float dimension_to_stbd, float dimension_to_stern) const;

};

#endif