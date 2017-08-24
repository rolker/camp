#ifndef LOCATIONPOSITION_H
#define LOCATIONPOSITION_H

#include <QGeoCoordinate>
#include <QPointF>

struct LocationPosition
{
    QGeoCoordinate location;
    QPointF pos;
};

#endif
