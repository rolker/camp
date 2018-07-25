#ifndef SURVEYAREA_H
#define SURVEYAREA_H

#include "geographicsmissionitem.h"

class SurveyArea : public GeoGraphicsMissionItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
    
public:
    explicit SurveyArea(MissionItem *parent = 0);
    
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    QPainterPath shape() const;
    
    Waypoint * createWaypoint();
    Waypoint * addWaypoint(QGeoCoordinate const &location);
    void removeWaypoint(Waypoint *wp);
    
    QList<Waypoint *> waypoints() const;

    void write(QJsonObject &json) const;
    void read(const QJsonObject &json);
    
    int type() const override {return SurveyAreaType;}
    
    bool canAcceptChildType(const std::string & childType) const override;
    
    QList<QList<QGeoCoordinate> > getLines() const override;
    
signals:
    
public slots:
    void updateProjectedPoints();
    
private:
};

#endif
