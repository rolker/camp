#ifndef GEOGRAPHICSITEM_H
#define GEOGRAPHICSITEM_H

#include "missionitem.h"
#include <QGraphicsItem>
#include <QGeoCoordinate>

class AutonomousVehicleProject;
class QGrpahicsTextItem;

class GeoGraphicsItem : public MissionItem, public QGraphicsItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)

public:
    GeoGraphicsItem(QObject *parent = 0, QGraphicsItem *parentItem = Q_NULLPTR);

    QPointF geoToPixel(QGeoCoordinate const &point) const;
    QGeoCoordinate pixelToGeo(QPointF const &point) const;

    virtual void write(QJsonObject &json) const = 0;
    virtual void read(const QJsonObject &json) = 0;

    void prepareGeometryChange();

    bool showLabelFlag() const;
    void setShowLabelFlag(bool show=true);
    void setLabel(QString const &label);

public slots:
    virtual void updateProjectedPoints() =0;

private:
    QGraphicsTextItem *m_label;
    QString m_labelText;
    bool m_showLabelFlag;

};

#endif // GEOGRAPHICSITEM_H
