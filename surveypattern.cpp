#include "surveypattern.h"
#include "waypoint.h"
#include <QPainter>
#include <QtMath>
#include <QJsonObject>
#include <QDebug>
#include "platform.h"
#include "autonomousvehicleproject.h"

SurveyPattern::SurveyPattern(MissionItem *parent):GeoGraphicsMissionItem(parent),
    m_startLocation(nullptr),m_endLocation(nullptr),m_spacing(1.0),m_direction(0.0),m_spacingLocation(nullptr),m_arcCount(6),m_maxSegmentLength(0.0),m_internalUpdateFlag(false)
{
    setShowLabelFlag(true);
}

Waypoint * SurveyPattern::createWaypoint()
{
    Waypoint * wp = createMissionItem<Waypoint>();
    wp->setFlag(QGraphicsItem::ItemIsMovable);
    wp->setFlag(QGraphicsItem::ItemIsSelectable);
    wp->setFlag(QGraphicsItem::ItemSendsGeometryChanges);
    wp->setFlag(QGraphicsItem::ItemSendsScenePositionChanges);
    connect(wp, &Waypoint::waypointMoved, this, &SurveyPattern::waypointHasChanged);
    connect(wp, &Waypoint::waypointAboutToMove, this, &SurveyPattern::waypointAboutToChange);
    return wp;
}

void SurveyPattern::setStartLocation(const QGeoCoordinate &location)
{
    if(m_startLocation == nullptr)
    {
        m_startLocation = createWaypoint();
        m_startLocation->setObjectName("start");
    }
    m_startLocation->setLocation(location);
    m_startLocation->setPos(m_startLocation->geoToPixel(location,autonomousVehicleProject()));
    update();
}

void SurveyPattern::setEndLocation(const QGeoCoordinate &location, bool calc)
{
    if(m_endLocation == nullptr)
    {
        m_endLocation = createWaypoint();
        m_endLocation->setObjectName("end");
    }
    m_endLocation->setLocation(location);
    m_endLocation->setPos(m_endLocation->geoToPixel(location,autonomousVehicleProject()));
    if(calc)
        calculateFromWaypoints();
    update();
}

void SurveyPattern::setSpacingLocation(const QGeoCoordinate &location, bool calc)
{
    if(m_spacingLocation == nullptr)
    {
        m_spacingLocation = createWaypoint();
        m_spacingLocation->setObjectName("spacing/direction");
    }
    m_spacingLocation->setLocation(location);
    if(calc)
        calculateFromWaypoints();
    update();
}

void SurveyPattern::calculateFromWaypoints()
{
    if(m_startLocation && m_endLocation)
    {
        qreal ab_distance = m_startLocation->location().distanceTo(m_endLocation->location());
        qreal ab_angle = m_startLocation->location().azimuthTo(m_endLocation->location());

        qreal ac_distance = 1.0;
        qreal ac_angle = 90.0;
        if(m_spacingLocation)
        {
            ac_distance = m_startLocation->location().distanceTo(m_spacingLocation->location());
            ac_angle = m_startLocation->location().azimuthTo(m_spacingLocation->location());
            m_spacing = ac_distance;
            m_direction = ac_angle-90;
        }
        qreal leg_heading = ac_angle-90.0;
        m_lineLength = ab_distance*qCos(qDegreesToRadians(ab_angle-leg_heading));
        m_totalWidth = ab_distance*qSin(qDegreesToRadians(ab_angle-leg_heading));
    }
}


void SurveyPattern::setArcCount(int ac)
{
    m_arcCount = ac;
    update();
}

void SurveyPattern::setMaxSegmentLength(double maxLength)
{
    m_maxSegmentLength = maxLength;
    update();
}

void SurveyPattern::write(QJsonObject &json) const
{
    json["type"] = "SurveyPattern";
    if(m_startLocation)
    {
        QJsonObject slObject;
        m_startLocation->write(slObject);
        json["startLocation"] = slObject;
    }
    if(m_endLocation)
    {
        QJsonObject elObject;
        m_endLocation->write(elObject);
        json["endLocation"] = elObject;
    }
    json["spacing"] = m_spacing;
    json["direction"] = m_direction;
}

void SurveyPattern::read(const QJsonObject &json)
{
    m_startLocation = createWaypoint();
    m_startLocation->read(json["startLocation"].toObject());
    m_endLocation = createWaypoint();
    m_endLocation->read(json["endLocation"].toObject());
    setDirectionAndSpacing(json["direction"].toDouble(),json["spacing"].toDouble());
}


bool SurveyPattern::hasSpacingLocation() const
{
    return (m_spacingLocation != nullptr);
}

double SurveyPattern::spacing() const
{
    return m_spacing;
}

double SurveyPattern::direction() const
{
    return m_direction;
}

double SurveyPattern::lineLength() const
{
    return m_lineLength;
}

double SurveyPattern::totalWidth() const
{
    return m_totalWidth;
}

int SurveyPattern::arcCount() const
{
    return m_arcCount;
}

double SurveyPattern::maxSegmentLength() const
{
    return m_maxSegmentLength;
}

Waypoint * SurveyPattern::startLocationWaypoint() const
{
    return m_startLocation;
}

Waypoint * SurveyPattern::endLocationWaypoint() const
{
    return m_endLocation;
}

void SurveyPattern::setDirectionAndSpacing(double direction, double spacing)
{
    m_direction = direction;
    m_spacing = spacing;
    QGeoCoordinate c = m_startLocation->location().atDistanceAndAzimuth(spacing,direction+90.0);
    m_internalUpdateFlag = true;
    setSpacingLocation(c,false);
    m_internalUpdateFlag = false;
}

void SurveyPattern::setLineLength(double lineLength)
{
    m_lineLength = lineLength;
    updateEndLocation();
}

void SurveyPattern::setTotalWidth(double totalWidth)
{
    m_totalWidth = totalWidth;
    updateEndLocation();
}

void SurveyPattern::updateEndLocation()
{
    m_internalUpdateFlag = true;
    QGeoCoordinate p = m_startLocation->location().atDistanceAndAzimuth(m_lineLength, m_direction);
    p = p.atDistanceAndAzimuth(m_totalWidth,m_direction+90.0);
    setEndLocation(p,false);
    m_internalUpdateFlag = false;
}

QRectF SurveyPattern::boundingRect() const
{
    return shape().boundingRect();
}

void SurveyPattern::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    auto lines = getLines();
    if(lines.length() > 0)
    {
        painter->save();

        QPen p;
        p.setColor(Qt::red);
        p.setCosmetic(true);
        p.setWidth(3);
        painter->setPen(p);
        
        for (auto l:lines)
        {
            auto first = l.begin();
            auto second = first;
            second++;
            while(second != l.end())
            {
                p.setWidth(10);
                p.setColor(Qt::blue);
                painter->setPen(p);
                painter->drawPoint(m_startLocation->geoToPixel(*first,autonomousVehicleProject()));
                p.setWidth(3);
                p.setColor(Qt::red);
                painter->setPen(p);
                painter->drawLine(m_startLocation->geoToPixel(*first,autonomousVehicleProject()),m_startLocation->geoToPixel(*second,autonomousVehicleProject()));
                first++;
                second++;
            }
            p.setWidth(10);
            p.setColor(Qt::blue);
            painter->setPen(p);
            painter->drawPoint(m_startLocation->geoToPixel(*first,autonomousVehicleProject()));
        }
        painter->restore();
    }
    return;

}

void SurveyPattern::updateLabel()
{
    double cumulativeDistance = 0.0;
    
    auto lines = getLines();
    for (auto l: lines)
        if (l.length() > 1)
        {
            auto first = l.begin();
            auto second = first;
            second++;
            while(second != l.end())
            {
                cumulativeDistance += first->distanceTo(*second);
                first++;
                second++;
            }
        }

    double distanceInNMs = cumulativeDistance*0.000539957;
    QString label = "Distance: "+QString::number(cumulativeDistance)+" (m), "+QString::number(distanceInNMs)+" (nm)";

    AutonomousVehicleProject* avp = autonomousVehicleProject();
    if(avp)
    {
        Platform *platform = avp->currentPlatform();
        if(platform)
        {
            double time = distanceInNMs/platform->speed();
            label += " ETE: "+QString::number(time)+" (h)";
        }
    }

    setLabel(label);
}

QPainterPath SurveyPattern::shape() const
{
    auto lines = getLines();
    if(!lines.empty())
    {
        if(!lines.front().empty())
        {
            QPainterPath ret(m_startLocation->geoToPixel(lines.front().front(),autonomousVehicleProject()));
            for(auto l: lines)
                for(auto p:l)
                    ret.lineTo(m_startLocation->geoToPixel(p,autonomousVehicleProject()));
            QPainterPathStroker pps;
            pps.setWidth(10);
            return pps.createStroke(ret);
        }
    }
    return QPainterPath();
}

// QList<QGeoCoordinate> SurveyPattern::getPath() const
// {
//     QList<QGeoCoordinate> ret;
//     if(m_startLocation)
//     {
//         ret.append(m_startLocation->location());
//         QGeoCoordinate lastLocation = ret.back();
//         if(m_endLocation)
//         {
//             qreal ab_distance = m_startLocation->location().distanceTo(m_endLocation->location());
//             qreal ab_angle = m_startLocation->location().azimuthTo(m_endLocation->location());
// 
//             qreal ac_distance = 1.0;
//             qreal ac_angle = 90.0;
//             if(m_spacingLocation)
//             {
//                 ac_distance = m_startLocation->location().distanceTo(m_spacingLocation->location());
//                 ac_angle = m_startLocation->location().azimuthTo(m_spacingLocation->location());
//             }
//             qreal leg_heading = ac_angle-90.0;
//             qreal leg_length = ab_distance*qCos(qDegreesToRadians(ab_angle-leg_heading));
//             //qDebug() << "getPath: leg_length: " << leg_length << " leg_heading: " << leg_heading;
//             qreal surveyWidth = ab_distance*qSin(qDegreesToRadians(ab_angle-leg_heading));
// 
//             int line_count = qCeil(surveyWidth/ac_distance);
//             for (int i = 0; i < line_count; i++)
//             {
//                 int dir = i%2;
//                 ret.append(lastLocation.atDistanceAndAzimuth(leg_length,leg_heading+dir*180));
//                 if (i < line_count-1)
//                 {
//                     lastLocation = ret.back();
//                     if (m_arcCount > 1)
//                     {
//                         qreal deltaAngle = 180.0/float(m_arcCount);
//                         qreal r = ac_distance/2.0;
//                         qreal h = r*cos(deltaAngle*M_PI/360.0);
//                         qreal d = 2.0*h*tan(deltaAngle*M_PI/360.0);
//                         qreal currentAngle = leg_heading+dir*180;
//                         if(leg_length < 0.0)
//                         {
//                             currentAngle += 180.0;
//                             deltaAngle = -deltaAngle;
//                         }
//                         if(dir)
//                             currentAngle += deltaAngle/2.0;
//                         else
//                             currentAngle -= deltaAngle/2.0;
//                         for(int j = 0; j < m_arcCount-1; j++)
//                         {
//                             if(dir)
//                                 currentAngle -= deltaAngle;
//                             else
//                                 currentAngle += deltaAngle;
//                             ret.append(ret.back().atDistanceAndAzimuth(d,currentAngle));
//                         }
//                     }
//                     ret.append(lastLocation.atDistanceAndAzimuth(ac_distance,ac_angle));
//                 }
//                 lastLocation = ret.back();
//             }
//             if (ret.length() < 2)
//                 ret.append(m_endLocation->location());
//         }
//     }
//     return ret;
// }

QList<QList<QGeoCoordinate> > SurveyPattern::getLines() const
{
    QList<QList<QGeoCoordinate> > ret;
    if(m_startLocation)
    {
        QList<QGeoCoordinate> line;
        line.append(m_startLocation->location());
        QGeoCoordinate lastLocation = line.back();
        if(m_endLocation)
        {
            qreal ab_distance = m_startLocation->location().distanceTo(m_endLocation->location());
            qreal ab_angle = m_startLocation->location().azimuthTo(m_endLocation->location());

            qreal ac_distance = 1.0;
            qreal ac_angle = 90.0;
            if(m_spacingLocation)
            {
                ac_distance = m_startLocation->location().distanceTo(m_spacingLocation->location());
                ac_angle = m_startLocation->location().azimuthTo(m_spacingLocation->location());
            }
            qreal leg_heading = ac_angle-90.0;
            qreal leg_length = ab_distance*qCos(qDegreesToRadians(ab_angle-leg_heading));
            //qDebug() << "getPath: leg_length: " << leg_length << " leg_heading: " << leg_heading;
            qreal surveyWidth = ab_distance*qSin(qDegreesToRadians(ab_angle-leg_heading));

            int line_count = qCeil(surveyWidth/ac_distance);
            for (int i = 0; i < line_count; i++)
            {
                int dir = i%2;
                if(m_maxSegmentLength > 0.0 && fabs(leg_length) > m_maxSegmentLength)
                {
                    int segCount = ceil(fabs(leg_length)/m_maxSegmentLength);
                    double segLength = leg_length/double(segCount);
                    for(int j = 0; j < segCount; j++)
                    {
                        line.append(lastLocation.atDistanceAndAzimuth(segLength,leg_heading+dir*180));
                        lastLocation = line.back();
                    }
                }
                else
                    line.append(lastLocation.atDistanceAndAzimuth(leg_length,leg_heading+dir*180));
                ret.append(line);
                line = QList<QGeoCoordinate>();
                if (i < line_count-1)
                {
                    lastLocation = ret.back().back();
                    if (m_arcCount > 1)
                    {
                        QList<QGeoCoordinate> arc;
                        arc.append(lastLocation);
                        qreal deltaAngle = 180.0/float(m_arcCount);
                        qreal r = ac_distance/2.0;
                        qreal h = r*cos(deltaAngle*M_PI/360.0);
                        qreal d = 2.0*h*tan(deltaAngle*M_PI/360.0);
                        qreal currentAngle = leg_heading+dir*180;
                        if(leg_length < 0.0)
                        {
                            currentAngle += 180.0;
                            deltaAngle = -deltaAngle;
                        }
                        if(dir)
                            currentAngle += deltaAngle/2.0;
                        else
                            currentAngle -= deltaAngle/2.0;
                        for(int j = 0; j < m_arcCount; j++)
                        {
                            if(dir)
                                currentAngle -= deltaAngle;
                            else
                                currentAngle += deltaAngle;
                            arc.append(arc.back().atDistanceAndAzimuth(d,currentAngle));
                        }
                        ret.append(arc);
                    }
                    line.append(lastLocation.atDistanceAndAzimuth(ac_distance,ac_angle));
                }
                lastLocation = ret.back().back();
            }
            //if (ret.length() < 2)
                //ret.append(m_endLocation->location());
        }
    }
    return ret;
}


void SurveyPattern::waypointAboutToChange()
{
    prepareGeometryChange();
}

void SurveyPattern::waypointHasChanged(Waypoint *wp)
{
    if(!m_internalUpdateFlag)
        calculateFromWaypoints();
    updateLabel();
    emit surveyPatternUpdated();
}

void SurveyPattern::updateProjectedPoints()
{
    if(m_startLocation)
        m_startLocation->updateProjectedPoints();
    if(m_endLocation)
        m_endLocation->updateProjectedPoints();
    if(m_spacingLocation)
        m_spacingLocation->updateProjectedPoints();
}

void SurveyPattern::onCurrentPlatformUpdated()
{
    updateLabel();
}

