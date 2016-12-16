#include "trackline.h"
#include "waypoint.h"
#include <QPainter>

TrackLine::TrackLine(QObject *parent, QGraphicsItem *parentItem) :GeoGraphicsItem(parent, parentItem)
{

}

QRectF TrackLine::boundingRect() const
{
    return childrenBoundingRect();
}

void TrackLine::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    auto children = childItems();
    if (children.length() > 1)
    {
        painter->save();

        painter->setPen(Qt::red);


        auto first = children.begin();
        auto second = first;
        second++;
        while(second != children.end())
        {
            painter->drawLine((*first)->pos(),(*second)->pos());
            first++;
            second++;
        }

        painter->restore();

    }

}

void TrackLine::addWaypoint(const QGeoCoordinate &location)
{
    Waypoint *wp = new Waypoint(parent(),this);
    wp->setLocation(location);
    wp->setPos(wp->geoToPixel(location));
    update();
}
