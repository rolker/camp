#include "waypoint.h"
#include <QPainter>
#include "autonomousvehicleproject.h"
#include "backgroundraster.h"
#include "map_view/web_mercator.h"
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>

Waypoint::Waypoint(MissionItem *parent, int row) :GeoGraphicsMissionItem(parent, row), m_internalPositionChangeFlag(false)
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
    // [#59 PR3a] Scene is Web Mercator; recover geo directly from the scene
    // position rather than the (depth-only) background raster's pixel space.
    // See ADR-0002.
    m_location = web_mercator::mapToGeo(scenePos());
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

bool Waypoint::readGeoJson(const QJsonObject &json)
{
  if(json["type"] == "Feature")
  {
    readGeoJsonProperties(json);
    if(json.contains("geometry") && json["geometry"].isObject())
    {
      const auto& geom = json["geometry"].toObject();
      std::string geomType;
      if(geom.contains("type") && geom["type"].isString())
        geomType = geom["type"].toString().toStdString();
      if(geomType == "Point")
      {
        if(geom.contains("coordinates") && geom["coordinates"].isArray())
        {
          const auto& coords = geom["coordinates"].toArray();
          if(coords.size() >= 2)
          {
            double lon = coords[0].toDouble();
            double lat = coords[1].toDouble();
            double alt = 0.0;
            if(coords.size() >= 3)
              alt = coords[2].toDouble();
            QGeoCoordinate position(lat, lon, alt);
            m_internalPositionChangeFlag = true;
            setLocation(position);
            m_internalPositionChangeFlag = false;
            return true;
          }
        }
      }
    }
  }
  return false;
}

void Waypoint::writeGeoJson(QJsonObject &json, QString name) const
{
  MissionItem::writeGeoJson(json, name);
  QJsonObject geometry;
  geometry["type"] = "Point";
  QJsonArray coordinates;
  writeToGeoJsonCoordinates(coordinates);
  geometry["coordinates"] = coordinates;
  json["geometry"] = geometry;
}

void Waypoint::writeToGeoJsonCoordinates(QJsonArray & json) const
{
  json.append(m_location.longitude());
  json.append(m_location.latitude());
  json.append(m_location.altitude());
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

void Waypoint::hoverEnterEvent(QGraphicsSceneHoverEvent * event)
{
    GeoGraphicsMissionItem::hoverEnterEvent(event);
    setLabel(objectName() + "\n" + m_location.toString(QGeoCoordinate::Degrees)+"\n"+m_location.toString(QGeoCoordinate::DegreesMinutesWithHemisphere));
    //setLabelPosition(geoToPixel(m_location,autonomousVehicleProject()));
    setShowLabelFlag(true);
}

void Waypoint::hoverLeaveEvent(QGraphicsSceneHoverEvent * event)
{
    GeoGraphicsMissionItem::hoverLeaveEvent(event);
    setShowLabelFlag(false);
}

bool Waypoint::canBeSentToRobot() const
{
    return false;
}
