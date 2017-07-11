#include "surveypattern.h"
#include "waypoint.h"
#include <QPainter>
#include <QtMath>
#include <QJsonObject>
#include <QDebug>

SurveyPattern::SurveyPattern(QObject *parent, QGraphicsItem *parentItem):GeoGraphicsItem(parent, parentItem),
    m_startLocation(nullptr),m_endLocation(nullptr),m_spacing(1.0),m_direction(0.0),m_spacingLocation(nullptr),m_arcCount(6)
{

}

Waypoint * SurveyPattern::createWaypoint()
{
    Waypoint * wp = new Waypoint(parent(),this);
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
        m_startLocation = createWaypoint();
    m_startLocation->setLocation(location);
    m_startLocation->setPos(m_startLocation->geoToPixel(location));
    update();
}

void SurveyPattern::setEndLocation(const QGeoCoordinate &location)
{
    if(m_endLocation == nullptr)
        m_endLocation = createWaypoint();
    m_endLocation->setLocation(location);
    m_endLocation->setPos(m_endLocation->geoToPixel(location));
    update();
}

void SurveyPattern::setSpacingLocation(const QGeoCoordinate &location, bool calc)
{
    if(m_spacingLocation == nullptr)
        m_spacingLocation = createWaypoint();
    m_spacingLocation->setLocation(location);
    if(calc)
    {
        m_spacing = m_startLocation->location().distanceTo(m_spacingLocation->location());
        m_direction = m_startLocation->location().azimuthTo(m_spacingLocation->location())-90;
    }
    update();
}

void SurveyPattern::setArcCount(int ac)
{
    m_arcCount = ac;
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

int SurveyPattern::arcCount() const
{
    return m_arcCount;
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
    setSpacingLocation(c,false);
}

QRectF SurveyPattern::boundingRect() const
{
    return shape().boundingRect();
}

void SurveyPattern::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    double cumulativeDistance = 0.0;
    auto children = getPath();
    if (children.length() > 1)
    {
        painter->save();

        QPen p;
        p.setColor(Qt::red);
        p.setCosmetic(true);
        p.setWidth(3);
        painter->setPen(p);

        auto first = children.begin();
        auto second = first;
        second++;
        while(second != children.end())
        {
            painter->drawLine(m_startLocation->geoToPixel(*first),m_startLocation->geoToPixel(*second));
            cumulativeDistance += first->distanceTo(*second);
            first++;
            second++;
        }

        painter->restore();

    }

    m_label->setPlainText("Distance: "+QString::number(cumulativeDistance)+" (m), "+QString::number(cumulativeDistance*0.000539957)+" (nm)");

}

QPainterPath SurveyPattern::shape() const
{
    auto children = getPath();
    auto i = children.begin();
    QPainterPath ret(m_startLocation->geoToPixel(*i));
    i++;
    while(i != children.end())
    {
        ret.lineTo(m_startLocation->geoToPixel(*i));
        i++;
    }
    QPainterPathStroker pps;
    pps.setWidth(5);
    return pps.createStroke(ret);

}

QList<QGeoCoordinate> SurveyPattern::getPath() const
{
    QList<QGeoCoordinate> ret;
    if(m_startLocation)
    {
        ret.append(m_startLocation->location());
        QGeoCoordinate lastLocation = ret.back();
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
            qDebug() << "getPath: leg_length: " << leg_length << " leg_heading: " << leg_heading;
            qreal surveyWidth = ab_distance*qSin(qDegreesToRadians(ab_angle-leg_heading));

            int line_count = qCeil(surveyWidth/ac_distance);
            for (int i = 0; i < line_count; i++)
            {
                int dir = i%2;
                ret.append(lastLocation.atDistanceAndAzimuth(leg_length,leg_heading+dir*180));
                if (i < line_count-1)
                {
                    lastLocation = ret.back();
                    if (m_arcCount > 1)
                    {
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
                        for(int j = 0; j < m_arcCount-1; j++)
                        {
                            if(dir)
                                currentAngle -= deltaAngle;
                            else
                                currentAngle += deltaAngle;
                            ret.append(ret.back().atDistanceAndAzimuth(d,currentAngle));
                        }
                    }
                    ret.append(lastLocation.atDistanceAndAzimuth(ac_distance,ac_angle));
                }
                lastLocation = ret.back();
            }
            if (ret.length() < 2)
                ret.append(m_endLocation->location());
        }
    }
    return ret;
}

void SurveyPattern::waypointAboutToChange()
{
    prepareGeometryChange();
}

void SurveyPattern::waypointHasChanged()
{
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
