#ifndef CAMP_SHIP_TRACK_H
#define CAMP_SHIP_TRACK_H

#include "geographicsitem.h"

class ShipTrack: public GeoGraphicsItem
{
public:
  ShipTrack(QGraphicsItem *parentItem = nullptr);

protected:
  // [#59 PR6] Chart-independent: positions come from the bg-free geoToPixel
  // (Web-Mercator scene), so no BackgroundRaster is needed. `scale` is
  // metres-per-display-pixel (see GeoGraphicsItem::metresPerPixel).
  void drawTriangle(QPainterPath &path, QGeoCoordinate const &location, double heading_degrees, double scale=1.0) const;
  void drawShipOutline(QPainterPath &path, QGeoCoordinate const &location, double heading_degrees, float dimension_to_bow, float dimension_to_port, float dimension_to_stbd, float dimension_to_stern) const;

};

#endif