#include "geographicsmissionitem.h"

#include "backgroundraster.h"
#include <QDebug>
#include <QVector2D>
#include <QtMath>

GeoGraphicsMissionItem::GeoGraphicsMissionItem(MissionItem* parent, int row):MissionItem(parent, row),m_lockedColor(50,200,50),m_unlockedColor(Qt::red), m_locked(false)
{
    if(parent)
    {
        QGraphicsItem *parentItem = parent->findParentGraphicsItem();
        setParentItem(parentItem);
        setAcceptHoverEvents(true);
        setOpacity(.5);
        setFlag(QGraphicsItem::ItemIsMovable);
        setFlag(QGraphicsItem::ItemIsSelectable);
        setFlag(QGraphicsItem::ItemSendsGeometryChanges);
    }
}

void GeoGraphicsMissionItem::updateBackground(BackgroundRaster* bg)
{
    setParentItem(bg);
    updateProjectedPoints();
}


void GeoGraphicsMissionItem::hoverEnterEvent(QGraphicsSceneHoverEvent* event)
{
    //qDebug() << "Enter item!";
    if(!m_locked)
        setOpacity(1.0);
}

void GeoGraphicsMissionItem::hoverLeaveEvent(QGraphicsSceneHoverEvent* event)
{
    //qDebug() << "Leave item!";
    setOpacity(.5);
}

QGraphicsItem * GeoGraphicsMissionItem::findParentGraphicsItem()
{
    return this;
}

QList<GeoGraphicsMissionItem *> GeoGraphicsMissionItem::childrenGeoGraphicsMissionItems() const
{
    QList<GeoGraphicsMissionItem *> ret;
    for(auto cmi: childMissionItems())
    {
        GeoGraphicsMissionItem * gmi = qobject_cast<GeoGraphicsMissionItem*>(cmi);
        if(gmi)
            ret.append(gmi);
    }
    return ret;
}


void GeoGraphicsMissionItem::lock()
{
    setFlag(QGraphicsItem::ItemIsMovable, false);
    setOpacity(.5);
    m_locked = true;
    for(auto childItem: childrenGeoGraphicsMissionItems())
        childItem->lock();
}

void GeoGraphicsMissionItem::unlock()
{
    setFlag(QGraphicsItem::ItemIsMovable, true);
    m_locked = false;
    for(auto childItem: childrenGeoGraphicsMissionItems())
        childItem->unlock();
}

bool GeoGraphicsMissionItem::locked() const
{
    return m_locked;
}

void GeoGraphicsMissionItem::drawArrow(QPainterPath& path, const QPointF& from, const QPointF& to, bool drawAtBeginning) const
{
    qreal scale = 1.0;
    auto bgr = autonomousVehicleProject()->getBackgroundRaster();
    if(bgr)
        scale = 1.0/bgr->mapScale();// scaledPixelSize();
    //qDebug() << "scale: " << scale;
    scale = std::max(0.05,scale);
    
    QPointF anchor = to;
    if(drawAtBeginning)
        anchor = from;

    path.moveTo(anchor);
    QVector2D v(to-from);
    v.normalize();
    QVector2D left(-v.y(),v.x());
    QVector2D right(v.y(),-v.x());
    QVector2D back = -v;
    path.lineTo(anchor+(left+back*2).toPointF()*10*scale);
    path.moveTo(anchor);
    path.lineTo(anchor+(right+back*2).toPointF()*10*scale);
    path.moveTo(anchor);
    
}

void GeoGraphicsMissionItem::drawTriangle(QPainterPath& path, const QGeoCoordinate& location, double heading_degrees, double scale) const
{
    QGeoCoordinate tip = location.atDistanceAndAzimuth(15*scale,heading_degrees);
    QGeoCoordinate llcorner = location.atDistanceAndAzimuth(15*scale,heading_degrees-150);
    QGeoCoordinate lrcorner = location.atDistanceAndAzimuth(15*scale,heading_degrees+150);

    QPointF ltip = geoToPixel(tip,autonomousVehicleProject());
    QPointF lllocal = geoToPixel(llcorner,autonomousVehicleProject());
    QPointF lrlocal = geoToPixel(lrcorner,autonomousVehicleProject());

    path.moveTo(ltip);
    path.lineTo(lllocal);
    path.lineTo(lrlocal);
    path.lineTo(ltip);
}

void GeoGraphicsMissionItem::updateETE()
{
    double cumulativeDistance = 0.0;
    
    QGeoCoordinate minPosition;
    QGeoCoordinate maxPosition;
    QGeoCoordinate lastPosition;
    
    auto lines = getLines();
    
    for (auto l: lines)
        if (l.length() > 1)
        {
            auto first = l.begin();
            if (!minPosition.isValid())
            {
                minPosition = *first;
                maxPosition = *first;
            }
            if (lastPosition.isValid())
                cumulativeDistance += lastPosition.distanceTo(*first);
            auto second = first;
            second++;
            while(second != l.end())
            {
                minPosition.setLatitude(std::min(minPosition.latitude(), second->latitude()));
                minPosition.setLongitude(std::min(minPosition.longitude(), second->longitude()));
                maxPosition.setLatitude(std::max(maxPosition.latitude(), second->latitude()));
                maxPosition.setLongitude(std::max(maxPosition.longitude(), second->longitude()));
                cumulativeDistance += first->distanceTo(*second);
                lastPosition = *second;
                first++;
                second++;
            }
        }
        
    double distanceInNMs = cumulativeDistance*0.000539957; // meters to NMs.
    QString label = "Distance: "+QString::number(int(cumulativeDistance))+" (m), "+QString::number(distanceInNMs,'f',1)+" (nm)";

    if(m_speed > 0.0)
    {
        double time = distanceInNMs/m_speed;
        if(time < 1.0)
            label += "\nETE: "+QString::number(int(time*60))+" (min)";
        else
            label += "\nETE: "+QString::number(time,'f',2)+" (h)";
    }

    AutonomousVehicleProject* avp = autonomousVehicleProject();
    if(avp)
    {
        qreal scale = avp->mapScale();
        qreal distance = minPosition.distanceTo(maxPosition);
        qreal bearing = minPosition.azimuthTo(maxPosition);
        qreal bearing_rad = qDegreesToRadians(bearing);
        qreal cos_bearing = cos(bearing_rad);
        qreal sin_bearing = sin(bearing_rad);
        qreal dy = cos_bearing*distance*0.2;
        qreal dx = sin_bearing*distance*0.2;
        //qDebug() << "scale: " << scale << " distance: " << distance << " bearing: " << bearing << " dx,dy: " << dx << "," << dy;
        setLabelPosition(QPointF(dx*scale, dy*scale));
    }

    setLabel(label);
}

void GeoGraphicsMissionItem::readChildren(const QJsonArray &json, int row)
{
    prepareGeometryChange();
    MissionItem::readChildren(json, row);
}