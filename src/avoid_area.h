#ifndef AVOID_AREA_H
#define AVOID_AREA_H

#include "geographicsmissionitem.h"

class AvoidArea : public GeoGraphicsMissionItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
    
public:
    explicit AvoidArea(MissionItem *parent = 0, int row = -1);
    
    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    QPainterPath shape() const;

    Waypoint * createPoint();
    Waypoint * addPoint(QGeoCoordinate const &location);
    void removePoint(Waypoint *wp);

    QList<Waypoint *> points() const;

    int type() const override {return AvoidAreaType;}

    void write(QJsonObject &json) const override;
    void writeToMissionPlan(QJsonArray & navArray) const override;
    void read(const QJsonObject &json) override;

    bool canAcceptChildType(const std::string & childType) const override;
    bool canBeSentToRobot() const override;



    QVariant itemChange(GraphicsItemChange change, const QVariant &value) override;

signals:
    void avoidAreaChanged();
public slots:
    void updateProjectedPoints();

protected:
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;

private:

};

#endif
