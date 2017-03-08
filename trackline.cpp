#include "trackline.h"
#include "waypoint.h"
#include <QPainter>
#include <QJsonObject>
#include <QJsonArray>

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

Waypoint * TrackLine::createWaypoint()
{
    Waypoint *wp = new Waypoint(parent(),this);
    wp->setFlag(QGraphicsItem::ItemIsMovable);
    wp->setFlag(QGraphicsItem::ItemIsSelectable);
    wp->setFlag(QGraphicsItem::ItemSendsGeometryChanges);
    return wp;

}

void TrackLine::addWaypoint(const QGeoCoordinate &location)
{
    Waypoint *wp = createWaypoint();
    wp->setLocation(location);
    wp->setPos(wp->geoToPixel(location));
    update();
}

void TrackLine::write(QJsonObject &json) const
{
    json["type"] = "TrackLine";
    QJsonArray wpArray;
    auto children = childItems();
    for(auto child: children)
    {
        Waypoint *wp = qgraphicsitem_cast<Waypoint*>(child);
        QJsonObject wpObject;
        wp->write(wpObject);
        wpArray.append(wpObject);
    }
    json["waypoints"] = wpArray;
}

void TrackLine::read(const QJsonObject &json)
{
    QJsonArray waypointsArray = json["waypoints"].toArray();
    for(int wpIndex = 0; wpIndex < waypointsArray.size(); wpIndex++)
    {
        QJsonObject wpObject = waypointsArray[wpIndex].toObject();
        if(wpIndex == 0)
        {
            QGeoCoordinate position(wpObject["latitude"].toDouble(),wpObject["longitude"].toDouble());
            setPos(geoToPixel(position));
        }
        Waypoint *wp = createWaypoint();
        wp->read(wpObject);
    }

}
