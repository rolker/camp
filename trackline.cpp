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

QPainterPath TrackLine::shape() const
{
    auto children = childItems();
    if (children.length() > 1)
    {
        auto i = children.begin();
        QPainterPath ret((*i)->pos());
        i++;
        while(i != children.end())
        {
            ret.lineTo((*i)->pos());
            i++;
        }
        QPainterPathStroker pps;
        pps.setWidth(5);
        return pps.createStroke(ret);

    }
    return QGraphicsItem::shape();
}

void TrackLine::addWaypoint(const QGeoCoordinate &location)
{
    Waypoint *wp = new Waypoint(parent(),this);
    wp->setLocation(location);
    wp->setPos(wp->geoToPixel(location));
    wp->setFlag(QGraphicsItem::ItemIsMovable);
    wp->setFlag(QGraphicsItem::ItemIsSelectable);
    wp->setFlag(QGraphicsItem::ItemSendsGeometryChanges);
    update();
}
