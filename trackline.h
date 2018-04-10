#ifndef TRACKLINE_H
#define TRACKLINE_H

#include "geographicsmissionitem.h"

class Waypoint;
class QStandardItem;

class TrackLine : public GeoGraphicsMissionItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)

public:
    explicit TrackLine(MissionItem *parent = 0);

    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    QPainterPath shape() const;

    Waypoint * createWaypoint();
    Waypoint * addWaypoint(QGeoCoordinate const &location);
    void removeWaypoint(Waypoint *wp);

    QList<Waypoint *> waypoints() const;

    void write(QJsonObject &json) const;
    void read(const QJsonObject &json);
    
    int type() const override {return TrackLineType;}
    
    bool canAcceptChildType(const std::string & childType) const override;
    
signals:
    void trackLineUpdated();

public slots:
    void updateProjectedPoints();

private:
};

#endif // TRACKLINE_H
