#ifndef WAYPOINT_H
#define WAYPOINT_H

#include "missionitem.h"
#include "geographicsitem.h"

class Waypoint : public MissionItem, public GeoGraphicsItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    explicit Waypoint(QObject *parent = 0, QGraphicsItem *parentItem =0);

    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    QGeoCoordinate const &location() const;
    void setLocation(QGeoCoordinate const &location);
    void updateLocation();

    void write(QJsonObject &json) const;
    void read(const QJsonObject &json);
    
    QStandardItem * createItem(const QString & label) override;

public slots:
    void updateProjectedPoints();

signals:
    void waypointMoved(Waypoint *wp);
    void waypointAboutToMove();

protected:
    QVariant itemChange(GraphicsItemChange change, const QVariant &value);

private:
    QGeoCoordinate m_location;
    bool m_internalPositionChangeFlag;
};

#endif // WAYPOINT_H
