#include "measuringtool.h"
#include "autonomousvehicleproject.h"
#include <QPainter>
#include <QDebug>
#include <math.h>

MeasuringTool::MeasuringTool(QGraphicsItem* parentItem, AutonomousVehicleProject* project): QObject(nullptr), GeoGraphicsItem(parentItem), m_project(project)
{
    setShowLabelFlag(true);
    setZValue(10.0);
}


QRectF MeasuringTool::boundingRect() const
{
    return shape().boundingRect().marginsAdded(QMarginsF(3,3,3,3));
}

void MeasuringTool::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    painter->save();

    QPen p;
    p.setCosmetic(true);
    p.setColor(Qt::magenta);
    p.setWidth(3);
    painter->setPen(p);
    painter->drawPath(shape());
    painter->restore();
}

QPainterPath MeasuringTool::shape() const
{
    QPainterPath ret;
    ret.moveTo(0,0);
    auto delta = geoToPixel(m_finish)-geoToPixel(m_start);
    ret.lineTo(delta);
    auto distance =  sqrt(delta.x()*delta.x()+delta.y()*delta.y());
    ret.addEllipse(QPointF(0, 0), distance, distance);
    return ret;
}

void MeasuringTool::setStart(QGeoCoordinate start)
{
    m_start = start;
    setPos(geoToPixel(start));
}

void MeasuringTool::setFinish(QGeoCoordinate finish)
{
    prepareGeometryChange();
    m_finish = finish;
    auto azimuth = m_start.azimuthTo(m_finish);
    auto distance = m_start.distanceTo(m_finish);
    QString distanceString = QString::number(distance,'f',0);
    if (distance < 10) distanceString = QString::number(distance,'f',1);
    if (distance < 1) distanceString = QString::number(distance,'f',2);
    QString labelString = distanceString+" meters\nbearing "+QString::number(int(azimuth))+" degrees";
    
    auto speed = m_project ? m_project->speed() : 0.0;
    if(speed > 0.0)
    {
        double distanceInNMs = distance*0.000539957;
        double time = distanceInNMs/speed;
        if (time < 1.0)
            if(time*60 < 1.0)
                labelString += "\nETE: "+QString::number(int(time*60*60))+" (s)";
            else
                labelString += "\nETE: "+QString::number(int(time*60))+" (min)";
        else
            labelString += "\nETE: "+QString::number(time,'f',2)+" (h)";
    }
    
    setLabel(labelString);
    auto halfDistance = (geoToPixel(m_finish)-geoToPixel(m_start))/2.0;
    setLabelPosition(halfDistance);
    update();
}
