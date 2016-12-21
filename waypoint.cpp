#include "waypoint.h"
#include <QPainter>
#include "autonomousvehicleproject.h"
#include "backgroundraster.h"
#include <QJsonObject>

Waypoint::Waypoint(QObject *parent, QGraphicsItem *parentItem) :GeoGraphicsItem(parent, parentItem)
{

}


QGeoCoordinate const &Waypoint::location() const
{
    return m_location;
}

void Waypoint::setLocation(QGeoCoordinate const &location)
{
    m_location = location;
    setPos(geoToPixel(location));
}

QRectF Waypoint::boundingRect() const
{
    qreal penWidth = 1;
    return QRectF(-10 - penWidth / 2, -10 - penWidth / 2, 20 + penWidth, 20 + penWidth);
}

void Waypoint::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    painter->save();

    painter->setPen(Qt::red);
    painter->drawRoundedRect(-10,-10,20,20,8,8);

    painter->restore();
}

QVariant Waypoint::itemChange(GraphicsItemChange change, const QVariant &value)
{
    if(change == ItemPositionChange)
    {
        QPointF newPos = value.toPointF();
        AutonomousVehicleProject *avp = qobject_cast<AutonomousVehicleProject*>(parent());
        BackgroundRaster *bgr = avp->getBackgroundRaster();
        QPointF projectedPosition = bgr->pixelToProjectedPoint(scenePos());
        m_location = bgr->unproject(projectedPosition);
        parentItem()->update();
    }
    return QGraphicsItem::itemChange(change,value);
}

void Waypoint::write(QJsonObject &json) const
{
    json["latitude"] = m_location.latitude();
    json["longitude"] = m_location.longitude();
}
