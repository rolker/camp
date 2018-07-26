#ifndef POLYGON_H
#define POLYGON_H

#include "geographicsmissionitem.h"
#include "locationposition.h"

class Polygon : public GeoGraphicsMissionItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    explicit Polygon(MissionItem *parent = 0);
    
    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
    QPainterPath shape() const override;
    
    void write(QJsonObject &json) const override;
    void writeToMissionPlan(QJsonArray & navArray) const override;
    void read(const QJsonObject &json) override;
    
    void addExteriorPoint(QGeoCoordinate const &location);
    void addInteriorPoint(QGeoCoordinate const &location);
    void addInteriorRing();

    void updateBBox();
    
    int type() const override {return PolygonType;}
    
public slots:
    void updateProjectedPoints() override;

private:
    QList<LocationPosition> m_exteriorRing;
    QPolygonF m_exteriorPolygon;
    QList<QList<LocationPosition> > m_interiorRings;
    QList<QPolygonF> m_interiorPolygons;
    QRectF m_bbox;
    
    
};

#endif // POLYGON_H
