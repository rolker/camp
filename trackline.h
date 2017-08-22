#ifndef TRACKLINE_H
#define TRACKLINE_H

#include "missionitem.h"
#include "geographicsitem.h"

class Waypoint;
class QStandardItem;

class TrackLine : public MissionItem, public GeoGraphicsItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)

public:
    explicit TrackLine(QObject *parent = 0, QGraphicsItem *parentItem =0);

    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    QPainterPath shape() const;

    Waypoint * createWaypoint();
    Waypoint * addWaypoint(QGeoCoordinate const &location);

    QList<Waypoint *> waypoints() const;

    void write(QJsonObject &json) const;
    void read(const QJsonObject &json);
    
    QStandardItem * createItem(const QString & label) override;


signals:
    void trackLineUpdated();

public slots:
    void updateProjectedPoints();

private:
};

#endif // TRACKLINE_H
