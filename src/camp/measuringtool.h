#ifndef MEASURINGTOOL_H
#define MEASURINGTOOL_H

#include "geographicsitem.h"

class AutonomousVehicleProject;

class MeasuringTool : public QObject, public GeoGraphicsItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    // [#59 ADR-0003] Parented to the Map scene-origin anchor (not a chart);
    // keeps the project for speed()/ETE. See ProjectView::mousePressEvent.
    MeasuringTool(QGraphicsItem* parentItem, AutonomousVehicleProject* project);

    QRectF boundingRect() const override;
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget);
    QPainterPath shape() const override;

    int type() const override {return MeasuringToolType;}

    void setStart(QGeoCoordinate start);
    void setFinish(QGeoCoordinate finish);

private:
    AutonomousVehicleProject* m_project;
    QGeoCoordinate m_start;
    QGeoCoordinate m_finish;

};

#endif // MEASURINGTOOL_H
