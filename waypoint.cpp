#include "waypoint.h"
#include <QPainter>
#include "autonomousvehicleproject.h"
#include "backgroundraster.h"
#include <QJsonObject>
#include <QDebug>

Waypoint::Waypoint(QObject *parent, QGraphicsItem *parentItem) :GeoGraphicsItem(parent, parentItem)
{

}


QGeoCoordinate const &Waypoint::location() const
{
    return m_location;
}

void Waypoint::setLocation(QGeoCoordinate const &location)
{
    qDebug() << "Waypoint::setLocation " << static_cast<const void *>(this) << location;
    setPos(geoToPixel(location));
    m_location = location;
}

QRectF Waypoint::boundingRect() const
{
    qreal penWidth = 1;
    return QRectF(-10 - penWidth / 2, -10 - penWidth / 2, 20 + penWidth, 20 + penWidth);
}

void Waypoint::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    painter->save();

    //double scale = painter->transform().m11();
    //painter->scale(1/scale,1/scale);

    painter->setPen(Qt::red);
    painter->drawRoundedRect(-10,-10,20,20,8,8);

    painter->restore();
}

QVariant Waypoint::itemChange(GraphicsItemChange change, const QVariant &value)
{
    if(change == ItemPositionChange || change == ItemScenePositionHasChanged)
    {

        AutonomousVehicleProject *avp = qobject_cast<AutonomousVehicleProject*>(parent());
        BackgroundRaster *bgr = avp->getBackgroundRaster();
        QPointF projectedPosition = bgr->pixelToProjectedPoint(scenePos());
        m_location = bgr->unproject(projectedPosition);
        qDebug() << "itemChange" << m_location;
        parentItem()->update();
    }
    if(change == ItemPositionChange)
        emit waypointAboutToMove();
    if(change == ItemPositionHasChanged)
        emit waypointMoved();

    return QGraphicsItem::itemChange(change,value);
}

void Waypoint::write(QJsonObject &json) const
{
    json["type"] = "Waypoint";
    json["latitude"] = m_location.latitude();
    json["longitude"] = m_location.longitude();
}

void Waypoint::read(const QJsonObject &json)
{
    QGeoCoordinate position(json["latitude"].toDouble(),json["longitude"].toDouble());
    setLocation(position);
}
