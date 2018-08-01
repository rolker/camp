#include "measuringtool.h"
#include "backgroundraster.h"
#include <QPainter>
#include <QDebug>
#include "platform.h"

MeasuringTool::MeasuringTool(BackgroundRaster* parent): QObject(parent), GeoGraphicsItem(parent)
{
    setShowLabelFlag(true);
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
    ret.lineTo(geoToPixel(m_finish,dynamic_cast<BackgroundRaster*>(parent())->autonomousVehicleProject())-geoToPixel(m_start,dynamic_cast<BackgroundRaster*>(parent())->autonomousVehicleProject()));
    return ret;
}

void MeasuringTool::setStart(QGeoCoordinate start)
{
    m_start = start;
    setPos(geoToPixel(start,dynamic_cast<BackgroundRaster*>(parent())->autonomousVehicleProject()));
}

void MeasuringTool::setFinish(QGeoCoordinate finish)
{
    prepareGeometryChange();
    m_finish = finish;
    auto azimuth = m_start.azimuthTo(m_finish);
    auto distance = m_start.distanceTo(m_finish);
    QString labelString = QString::number(int(distance))+" meters\nbearing "+QString::number(int(azimuth))+" degrees";
    
    BackgroundRaster* bgr = dynamic_cast<BackgroundRaster*>(parent());
    AutonomousVehicleProject* avp = bgr->autonomousVehicleProject();
    Platform *platform = avp->currentPlatform();
    if(platform)
    {
        double distanceInNMs = distance*0.000539957;
        double time = distanceInNMs/platform->speed();
        if (time < 1.0)
            if(time*60 < 1.0)
                labelString += "\nETE: "+QString::number(int(time*60*60))+" (s)";
            else
                labelString += "\nETE: "+QString::number(int(time*60))+" (min)";
        else
            labelString += "\nETE: "+QString::number(time,'f',2)+" (h)";
    }
    
    setLabel(labelString);
    auto halfDistance = (geoToPixel(m_finish,avp)-geoToPixel(m_start,avp))/2.0;
    setLabelPosition(halfDistance);
    update();
}
