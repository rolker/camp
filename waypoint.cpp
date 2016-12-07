#include "waypoint.h"
#include <QPainter>

Waypoint::Waypoint(QObject *parent, QGraphicsItem *parentItem) : QObject(parent), QGraphicsItem(parentItem)
{

}


QGeoCoordinate const &Waypoint::location() const
{
    return m_location;
}

void Waypoint::setLocation(QGeoCoordinate const &location)
{
    m_location = location;
}

QRectF Waypoint::boundingRect() const
{
    qreal penWidth = 1;
    return QRectF(-10 - penWidth / 2, -10 - penWidth / 2, 20 + penWidth, 20 + penWidth);
}

void Waypoint::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    painter->drawRoundedRect(-10,-10,20,20,8,8);
}
