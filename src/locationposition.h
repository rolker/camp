#ifndef LOCATIONPOSITION_H
#define LOCATIONPOSITION_H

#include <QGeoCoordinate>
#include <QPointF>
#include <cmath>

struct LocationPosition
{
    QGeoCoordinate location;
    QPointF pos;
};

struct LocationPositionHeadingTime: public LocationPosition
{
    /// Degrees
    float heading = std::nan("");

    /// Seconds since 1970
    double time = 0.0;
};

#endif
