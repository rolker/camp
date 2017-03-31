#ifndef TRACKLINE_H
#define TRACKLINE_H

#include "geographicsitem.h"

class Waypoint;
class QStandardItem;

class TrackLine : public GeoGraphicsItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)

public:
    explicit TrackLine(QObject *parent = 0, QGraphicsItem *parentItem =0);

    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    QPainterPath shape() const;

    Waypoint * createWaypoint();
    void addWaypoint(QGeoCoordinate const &location);

    QList<Waypoint *> waypoints() const;

    void write(QJsonObject &json) const;
    void read(const QJsonObject &json);

    void setItem(QStandardItem *item);

signals:
    void trackLineUpdated();

public slots:

private:

    QStandardItem * m_item;
};

#endif // TRACKLINE_H
