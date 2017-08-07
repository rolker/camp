#ifndef GEOGRAPHICSITEM_H
#define GEOGRAPHICSITEM_H

#include "missionitem.h"
#include <QGraphicsItem>
#include <QGeoCoordinate>

class AutonomousVehicleProject;

class GeoGraphicsItem : public MissionItem, public QGraphicsItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)

public:
    GeoGraphicsItem(QObject *parent = 0, QGraphicsItem *parentItem = Q_NULLPTR);

    QPointF geoToPixel(QGeoCoordinate const &point) const;
    QGeoCoordinate pixelToGeo(QPointF const &point) const;

    void prepareGeometryChange();

    bool showLabelFlag() const;
    void setShowLabelFlag(bool show=true);
    void setLabel(QString const &label);

public slots:
    virtual void updateProjectedPoints() =0;

private:
    QGraphicsSimpleTextItem *m_label;
    QString m_labelText;
    bool m_showLabelFlag;

};

#endif // GEOGRAPHICSITEM_H
