#ifndef POINT_H
#define POINT_H

#include "geographicsmissionitem.h"


class Point : public GeoGraphicsMissionItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    explicit Point(QObject *parent = 0, QGraphicsItem *parentItem =0);
    
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    QGeoCoordinate const &location() const;
    void setLocation(QGeoCoordinate const &location);
    
    void write(QJsonObject &json) const;
    void read(const QJsonObject &json);
    
public slots:
    void updateProjectedPoints();

private:
    QGeoCoordinate m_location;

};

#endif // POINT_H
