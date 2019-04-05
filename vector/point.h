#ifndef POINT_H
#define POINT_H

#include "geographicsmissionitem.h"


class Point : public GeoGraphicsMissionItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    explicit Point(MissionItem *parent = 0);
    
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    QGeoCoordinate const &location() const;
    void setLocation(QGeoCoordinate const &location);
    
    void write(QJsonObject &json) const override;
    void writeToMissionPlan(QJsonArray & navArray) const override;
    void read(const QJsonObject &json) override;

    int type() const override {return PointType;}
    
public slots:
    void updateProjectedPoints() override;

private:
    QGeoCoordinate m_location;

};

#endif // POINT_H
