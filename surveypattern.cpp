#include "surveypattern.h"
#include "waypoint.h"
#include <QPainter>
#include <QtMath>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>
#include "platform.h"
#include "autonomousvehicleproject.h"
#include "surveyarea.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace bg = boost::geometry;

SurveyPattern::SurveyPattern(MissionItem *parent):GeoGraphicsMissionItem(parent),
    m_startLocation(nullptr),m_endLocation(nullptr),m_spacing(1.0),m_direction(0.0),m_alignment(Alignment::start),m_spacingLocation(nullptr),m_internalUpdateFlag(false)
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
    setPos(m_startLocation->geoToPixel(location,autonomousVehicleProject()));
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
        m_spacing = ab_distance/10.0;
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


void SurveyPattern::write(QJsonObject &json) const
{
    MissionItem::write(json);
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
    switch(m_alignment)
    {
        case start:
            json["alignment"] = "start";
            break;
        case center:
            json["alignment"] = "center";
            break;
        case finish:
            json["alignment"] = "finish";
            break;
    }
    QJsonArray tracklineArray;
    auto lines = getLines();
    for (auto line: lines){
        QJsonObject tracklineObject;
        tracklineObject["type"] = "TrackLine";
        QJsonArray wpArray;
        for (auto wp: line)
        {
            QJsonObject wpObject;
            wpObject["type"] = "Waypoint";
            wpObject["latitude"] = wp.latitude();
            wpObject["longitude"] = wp.longitude();
            wpArray.append(wpObject);
        }
        tracklineObject["waypoints"] = wpArray;
        tracklineArray.append(tracklineObject);
    }
    json["children"] = tracklineArray;
}

void SurveyPattern::writeToMissionPlan(QJsonArray& navArray) const
{
    auto lines = getLines();
    for(int i = 0; i < lines.size(); i++)
    {
        auto l = lines[i];
        QJsonObject navItem;
        navItem["pathtype"] = "trackline";
        AutonomousVehicleProject* avp = autonomousVehicleProject();
        if(avp)
        {
            Platform *platform = avp->currentPlatform();
            if(platform)
            {
                QJsonObject params;
                params["speed_ms"] = platform->speed()*0.514444; // knots to m/s
                navItem["parameters"] = params;
            }
        }
        writeBehaviorsToMissionPlanObject(navItem);
        QJsonArray pathNavArray;
        for(auto wp: l)
        {
            Waypoint * temp_wp = new Waypoint();
            temp_wp->setLocation(wp);
            temp_wp->writeNavToMissionPlan(pathNavArray);
            delete temp_wp;
        }
        navItem["nav"] = pathNavArray;
        navItem["type"] = "survey_line";
        navArray.append(navItem);
    }    
}

void SurveyPattern::read(const QJsonObject &json)
{
    m_startLocation = createWaypoint();
    m_startLocation->read(json["startLocation"].toObject());
    m_endLocation = createWaypoint();
    m_endLocation->read(json["endLocation"].toObject());
    setDirectionAndSpacing(json["direction"].toDouble(),json["spacing"].toDouble());
    if(json.contains("alignment"))
    {
        if(json["alignment"] == "start")
            m_alignment = start;
        if(json["alignment"] == "center")
            m_alignment = center;
        if(json["alignment"] == "finish")
            m_alignment = finish;
    }
    calculateFromWaypoints();
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

SurveyPattern::Alignment SurveyPattern::alignment() const
{
    return m_alignment;
}


double SurveyPattern::lineLength() const
{
    return m_lineLength;
}

double SurveyPattern::totalWidth() const
{
    return m_totalWidth;
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

void SurveyPattern::setAlignment(SurveyPattern::Alignment alignment)
{
    prepareGeometryChange();
    m_alignment = alignment;
    qDebug() << "alignment: " << alignment;
    update();
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
    return shape().boundingRect().marginsAdded(QMarginsF(5,5,5,5));
}

void SurveyPattern::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    auto lines = getLines();
    if(lines.length() > 0)
    {
        painter->save();

        bool selected = false;
        if(autonomousVehicleProject()->currentSelected() == this)
            selected = true;

        QPen p;
        p.setCosmetic(true);
        if (selected)
        {
            p.setWidth(7);
            p.setColor(Qt::white);
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
                    p.setWidth(8);
                    p.setColor(Qt::black);
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
        }
        if(locked())
            p.setColor(m_lockedColor);
        else
            p.setColor(m_unlockedColor);
        p.setWidth(3);
        painter->setPen(p);

        bool turn = true; 
        for (auto l:lines)
        {
            turn = !turn;
            auto first = l.begin();
            auto second = first;
            second++;
            while(second != l.end())
            {
                p.setWidth(10);
                p.setColor(Qt::blue);
                painter->setPen(p);
                painter->drawPoint(m_startLocation->geoToPixel(*first,autonomousVehicleProject()));
                if (selected)
                    p.setWidth(5);
                else
                    p.setWidth(3);
                if(locked())
                    p.setColor(m_lockedColor);
                else
                    p.setColor(m_unlockedColor);
                painter->setPen(p);
                painter->drawLine(m_startLocation->geoToPixel(*first,autonomousVehicleProject()),m_startLocation->geoToPixel(*second,autonomousVehicleProject()));
                
//                 if(!turn || m_arcCount < 2)
//                 {
//                     QPainterPath ret(m_startLocation->geoToPixel(*first,autonomousVehicleProject()));
//                     drawArrow(ret,m_startLocation->geoToPixel(*second,autonomousVehicleProject()),m_startLocation->geoToPixel(*first,autonomousVehicleProject()));
//                     painter->drawPath(ret);
//                 }
                
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


QList<QList<QGeoCoordinate> > SurveyPattern::getLines() const
{
    QList<QList<QGeoCoordinate> > ret;
    if(m_startLocation && m_endLocation)
    {

        qreal diagonal_distance = m_startLocation->location().distanceTo(m_endLocation->location());
        qreal diagonal_angle = m_startLocation->location().azimuthTo(m_endLocation->location());

        qreal line_spacing = 1.0;
        qreal spacing_angle = 90.0;
        if(m_spacingLocation)
        {
            line_spacing = m_startLocation->location().distanceTo(m_spacingLocation->location());
            spacing_angle = m_startLocation->location().azimuthTo(m_spacingLocation->location());
        }
        else
            line_spacing = diagonal_distance/10.0;

        qreal leg_heading = spacing_angle-90.0;
        qreal leg_length = diagonal_distance*qCos(qDegreesToRadians(diagonal_angle-leg_heading));

        qreal surveyWidth = diagonal_distance*qSin(qDegreesToRadians(diagonal_angle-leg_heading));

        int line_count = qCeil(surveyWidth/line_spacing);
        
        qreal residual_distance = surveyWidth - ((line_count-1)*line_spacing);

        QList<QGeoCoordinate> line;
        line.append(m_startLocation->location().atDistanceAndAzimuth(m_alignment*residual_distance/2.0,spacing_angle));
        line.append(line.back().atDistanceAndAzimuth(leg_length,leg_heading));
        ret.append(line);
        
        for (int i = 1; i < line_count; i++)
        {
            line = QList<QGeoCoordinate>();
            line.append(ret.back().back().atDistanceAndAzimuth(line_spacing,spacing_angle));
            line.append(ret.back().front().atDistanceAndAzimuth(line_spacing,spacing_angle));
            ret.append(line);
        }
        
        // Check if we are a child of a SurveyArea.
        SurveyArea * surveyAreaParent = qobject_cast<SurveyArea*>(parent());
        if(surveyAreaParent)
        {
            typedef bg::model::d2::point_xy<double> BPoint;
            typedef bg::model::multi_point<BPoint> BMultiPoint;
            typedef bg::model::linestring<BPoint> BLineString;
            typedef bg::model::polygon<BPoint> BPolygon;
            typedef bg::model::multi_linestring<BLineString> BMultiLineString;

            BPolygon area_poly;
            
            for(auto wp: surveyAreaParent->waypoints())
            {
                BPoint p(wp->location().latitude(), wp->location().longitude());
                area_poly.outer().push_back(p);
            }
            
            bg::correct(area_poly);
            
            QList<QList<QGeoCoordinate> > clipped_ret;
            for(auto line: ret)
            {
                BPoint p1(line.front().latitude(),line.front().longitude());
                BPoint p2(line.back().latitude(),line.back().longitude());
                BLineString bline;
                bline.push_back(p1);
                bline.push_back(p2);
                
                BMultiLineString mls;
                bg::intersection(bline, area_poly, mls);
                
                for(auto l: mls)
                {
                    QList<QGeoCoordinate> clipped_line;
                    for(auto p: l)
                        clipped_line.push_back(QGeoCoordinate(p.x(), p.y()));
                    clipped_ret.push_back(clipped_line);
                }
            }
            
            return clipped_ret;
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
    updateETE();
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


void SurveyPattern::reverseDirection()
{
    prepareGeometryChange();
    
    auto direction = m_direction;
    auto spacing = m_spacing;
    
    auto l = m_startLocation->location();
    
    m_startLocation->setLocation(m_endLocation->location());
    m_endLocation->setLocation(l);
    
    setDirectionAndSpacing(direction+180,spacing);
    calculateFromWaypoints();
    
    if(m_alignment == start)
        m_alignment = finish;
    else if(m_alignment == finish)
        m_alignment = start;
    
    emit surveyPatternUpdated();

    update();
}

