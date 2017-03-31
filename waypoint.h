#ifndef WAYPOINT_H
#define WAYPOINT_H

#include "geographicsitem.h"

class Waypoint : public GeoGraphicsItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    explicit Waypoint(QObject *parent = 0, QGraphicsItem *parentItem =0);

    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    QGeoCoordinate const &location() const;
    void setLocation(QGeoCoordinate const &location);

    void write(QJsonObject &json) const;
    void read(const QJsonObject &json);
signals:

public slots:

signals:
    void waypointMoved();
    void waypointAboutToMove();

protected:
    QVariant itemChange(GraphicsItemChange change, const QVariant &value);

private:
    QGeoCoordinate m_location;
};

#endif // WAYPOINT_H
