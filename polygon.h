#ifndef POLYGON_H
#define POLYGON_H

#include "missionitem.h"
#include "geographicsitem.h"
#include "locationposition.h"

class Polygon : public MissionItem, public GeoGraphicsItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    explicit Polygon(QObject *parent = 0, QGraphicsItem *parentItem =0);
    
    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
    QPainterPath shape() const override;
    
    void write(QJsonObject &json) const override;
    void read(const QJsonObject &json) override;
    
    QStandardItem * createItem(const QString & label) override;

    void addExteriorPoint(QGeoCoordinate const &location);
    void addInteriorPoint(QGeoCoordinate const &location);
    void addInteriorRing();

public slots:
    void updateProjectedPoints();

private:
    QList<LocationPosition> m_exteriorRing;
    QList<QList<LocationPosition> > m_interiorRings;
    QRectF m_bbox;
    
    void updateBBox();
};

#endif // POLYGON_H
