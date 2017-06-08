#include "trackline.h"
#include "waypoint.h"
#include <QPainter>
#include <QJsonObject>
#include <QJsonArray>
#include <QStandardItem>
#include <QDebug>

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

        QPen p;
        p.setColor(Qt::red);
        p.setCosmetic(true);
        p.setWidth(3);
        painter->setPen(p);

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

    QStandardItem *item = new QStandardItem("wayoint");
    item->setData(QVariant::fromValue<Waypoint*>(wp));
    m_item->appendRow(item);

    wp->setFlag(QGraphicsItem::ItemIsMovable);
    wp->setFlag(QGraphicsItem::ItemIsSelectable);
    wp->setFlag(QGraphicsItem::ItemSendsGeometryChanges);
    return wp;

}

void TrackLine::addWaypoint(const QGeoCoordinate &location)
{
    qDebug() << "Trackline adding waypoint: " << location;
    Waypoint *wp = createWaypoint();
    qDebug() << static_cast<const void *>(wp);
    wp->setLocation(location);
    //wp->setPos(wp->geoToPixel(location));
    emit trackLineUpdated();
    update();
}

QList<Waypoint *> TrackLine::waypoints() const
{
    QList<Waypoint *> ret;
    auto children = childItems();
    for(auto child: children)
    {
        Waypoint *wp = qgraphicsitem_cast<Waypoint*>(child);
        if(wp)
            ret.append(wp);
    }
    return ret;
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

void TrackLine::setItem(QStandardItem *item)
{
    m_item = item;
}
