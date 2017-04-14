#ifndef GEOGRAPHICSITEM_H
#define GEOGRAPHICSITEM_H

#include <QObject>
#include <QGraphicsItem>
#include <QGeoCoordinate>

class AutonomousVehicleProject;
class QStandardItem;

class GeoGraphicsItem : public QObject, public QGraphicsItem
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

private:
    QStandardItem *m_item;
};

#endif // GEOGRAPHICSITEM_H
