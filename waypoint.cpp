#include "waypoint.h"
#include <QPainter>
#include "autonomousvehicleproject.h"
#include "backgroundraster.h"
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>

Waypoint::Waypoint(MissionItem *parent) :GeoGraphicsMissionItem(parent), m_internalPositionChangeFlag(false)
{
    m_unlockedColor = Qt::darkRed;
    m_lockedColor = Qt::darkGreen;
}

QGeoCoordinate const &Waypoint::location() const
{
    return m_location;
}

void Waypoint::setLocation(QGeoCoordinate const &location)
{
    //qDebug() << "Waypoint::setLocation " << static_cast<const void *>(this) << location;
    setPos(geoToPixel(location,autonomousVehicleProject()));
    m_location = location;
    setLabel(location.toString());
}

QRectF Waypoint::boundingRect() const
{
    return shape().boundingRect().marginsAdded(QMarginsF(2,2,2,2));
}

void Waypoint::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    painter->save();

    QPen p;
    if(locked())
        p.setColor(m_lockedColor);
    else
        p.setColor(m_unlockedColor);
    p.setCosmetic(true);
    p.setWidth(5);
    painter->setPen(p);
    
    painter->drawPath(shape());

    painter->restore();
}

QPainterPath Waypoint::shape() const
{
    QPainterPath ret;
    qreal scale = 1.0;
    auto bgr = autonomousVehicleProject()->getBackgroundRaster();
    if(bgr)
        scale = 1.0/bgr->mapScale();// scaledPixelSize();
    //qDebug() << "scale: " << scale;
    scale = std::max(0.05,scale);
    ret.addRoundedRect(-10*scale,-10*scale,20*scale,20*scale,8*scale,8*scale);
    return ret;
}


void Waypoint::updateLocation()
{
    AutonomousVehicleProject *avp = autonomousVehicleProject();
    BackgroundRaster *bgr = avp->getBackgroundRaster();
    QPointF projectedPosition = bgr->pixelToProjectedPoint(scenePos());
    m_location = bgr->unproject(projectedPosition);
    setLabel(m_location.toString());
}

QVariant Waypoint::itemChange(GraphicsItemChange change, const QVariant &value)
{
    if(!m_internalPositionChangeFlag)
    {
        if(change == ItemPositionChange || change == ItemScenePositionHasChanged)
        {
            updateLocation();
            parentItem()->update();
        }
        if(change == ItemPositionChange)
            emit waypointAboutToMove();
        if(change == ItemPositionHasChanged)
            emit waypointMoved(this);
    }

    return QGraphicsItem::itemChange(change,value);
}

void Waypoint::write(QJsonObject &json) const
{
    MissionItem::write(json);
    json["type"] = "Waypoint";
    json["latitude"] = m_location.latitude();
    json["longitude"] = m_location.longitude();
}

void Waypoint::writeToMissionPlan(QJsonArray& navArray) const
{
    QJsonObject waypointObject;
    writeBehaviorsToMissionPlanObject(waypointObject);
    
    waypointObject["pathtype"] = "waypoint";
    
    QJsonArray wpNavArray;
    writeNavToMissionPlan(wpNavArray);
    waypointObject["nav"] = wpNavArray;
    
    navArray.append(waypointObject);
}

void Waypoint::writeNavToMissionPlan(QJsonArray& navArray) const
{
    QJsonObject navObject;

    QJsonObject orientationObject;
    orientationObject["heading"] = QJsonValue::Null;
    orientationObject["pitch"] = QJsonValue::Null;
    orientationObject["roll"] = QJsonValue::Null;
    navObject["orientation"] = orientationObject;

    QJsonObject positionObject;
    positionObject["altitude"] = m_location.altitude();
    positionObject["latitude"] = m_location.latitude();
    positionObject["longitude"] = m_location.longitude();
    navObject["position"] = positionObject;
    
    navArray.append(navObject);
}


void Waypoint::read(const QJsonObject &json)
{
    MissionItem::read(json);
    QGeoCoordinate position(json["latitude"].toDouble(),json["longitude"].toDouble());
    m_internalPositionChangeFlag = true;
    setLocation(position);
    m_internalPositionChangeFlag = false;
}

void Waypoint::updateProjectedPoints()
{
    m_internalPositionChangeFlag = true;
    setPos(geoToPixel(m_location,autonomousVehicleProject()));
    m_internalPositionChangeFlag = false;
}

QList<QList<QGeoCoordinate> > Waypoint::getLines() const
{
    QList<QList<QGeoCoordinate> > ret;
    ret.append(QList<QGeoCoordinate>());
    ret.back().append(m_location);
    return ret;
}
