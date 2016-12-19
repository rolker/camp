#ifndef TRACKLINE_H
#define TRACKLINE_H

#include "geographicsitem.h"


class TrackLine : public GeoGraphicsItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)

public:
    explicit TrackLine(QObject *parent = 0, QGraphicsItem *parentItem =0);

    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    QPainterPath shape() const;

    void addWaypoint(QGeoCoordinate const &location);

signals:

public slots:
};

#endif // TRACKLINE_H
