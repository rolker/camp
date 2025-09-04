#ifndef MEASURINGTOOL_H
#define MEASURINGTOOL_H

#include "geographicsitem.h"

class BackgroundRaster;

class MeasuringTool : public QObject, public GeoGraphicsItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    MeasuringTool(BackgroundRaster* parent);

    QRectF boundingRect() const override;
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget);
    QPainterPath shape() const override;
    
    int type() const override {return MeasuringToolType;}
    
    void setStart(QGeoCoordinate start);
    void setFinish(QGeoCoordinate finish);

private:
    QGeoCoordinate m_start;
    QGeoCoordinate m_finish;

};

#endif // MEASURINGTOOL_H
