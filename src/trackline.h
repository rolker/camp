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

    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
    QPainterPath shape() const override;
    
    //void drawArrow(QPainterPath &path, QPointF const &from, QPointF const &to) const;

    Waypoint * createWaypoint();
    Waypoint * addWaypoint(QGeoCoordinate const &location);
    void removeWaypoint(Waypoint *wp);

    QList<Waypoint *> waypoints() const;

    void write(QJsonObject &json) const override;
    void writeToMissionPlan(QJsonArray & navArray) const override;
    void read(const QJsonObject &json) override;
    
    int type() const override {return TrackLineType;}
    
    bool canAcceptChildType(const std::string & childType) const override;
    
    QList<QList<QGeoCoordinate> > getLines() const override;
    
signals:
    void trackLineUpdated();

public slots:
    void updateProjectedPoints();
    void reverseDirection();
    void planPath();

private:
};

#endif // TRACKLINE_H
