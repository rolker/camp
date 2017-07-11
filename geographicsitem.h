#ifndef GEOGRAPHICSITEM_H
#define GEOGRAPHICSITEM_H

#include "missionitem.h"
#include <QGraphicsItem>
#include <QGeoCoordinate>

class AutonomousVehicleProject;
class QStandardItem;
class QGrpahicsTextItem;

class GeoGraphicsItem : public MissionItem, public QGraphicsItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)

public:
    GeoGraphicsItem(QObject *parent = 0, QGraphicsItem *parentItem = Q_NULLPTR);

    QPointF geoToPixel(QGeoCoordinate const &point) const;
    QGeoCoordinate pixelToGeo(QPointF const &point) const;

    AutonomousVehicleProject * project() const;

    virtual void write(QJsonObject &json) const = 0;
    virtual void read(const QJsonObject &json) = 0;

    void setItem(QStandardItem * item);
    QStandardItem * item() const;

public slots:
    virtual void updateProjectedPoints() =0;

protected:
    QGraphicsTextItem *m_label;

private:
    QStandardItem *m_item;
};

#endif // GEOGRAPHICSITEM_H
