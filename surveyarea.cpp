#include "surveyarea.h"
#include "waypoint.h"
#include <QPainter>
#include <QJsonObject>
#include <QJsonArray>
#include "backgroundraster.h"
#include "trackline.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <QDebug>

typedef boost::geometry::model::d2::point_xy<double> BPoint;
typedef boost::geometry::model::multi_point<BPoint> BMultiPoint;
typedef boost::geometry::model::segment<BPoint> BSegment;
typedef boost::geometry::model::linestring<BPoint> BLineString;
typedef boost::geometry::model::polygon<BPoint> BPolygon;
typedef boost::geometry::model::multi_linestring<BLineString> BMultiLineString;



SurveyArea::SurveyArea(MissionItem *parent) :GeoGraphicsMissionItem(parent)
{
}

QRectF SurveyArea::boundingRect() const
{
    return childrenBoundingRect();
}

void SurveyArea::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    painter->save();

    QPen p;
    p.setColor(Qt::blue);
    p.setCosmetic(true);
    p.setWidth(2);
    painter->setPen(p);
    painter->drawPath(shape());   
    painter->restore();
}

QPainterPath SurveyArea::shape() const
{
    auto children = waypoints();
    if (children.length() > 1)
    {
        auto i = children.begin();
        QPainterPath ret((*i)->pos());
        auto last = i;
        i++;
        while(i != children.end())
        {
            ret.lineTo((*i)->pos());
            if (last == children.begin())
                drawArrow(ret,(*last)->pos(),(*i)->pos());
            last = i;
            i++;
        }
        ret.lineTo(children.front()->pos());
        QPainterPathStroker pps;
        pps.setWidth(5);
        return pps.createStroke(ret);

    }
    return QGraphicsItem::shape();
}

Waypoint * SurveyArea::createWaypoint()
{
    int i = childMissionItems().size();
    QString wplabel = "waypoint"+QString::number(i);
    Waypoint *wp = createMissionItem<Waypoint>(wplabel);

    wp->setFlag(QGraphicsItem::ItemIsMovable);
    wp->setFlag(QGraphicsItem::ItemIsSelectable);
    wp->setFlag(QGraphicsItem::ItemSendsGeometryChanges);
    return wp;
}

Waypoint * SurveyArea::addWaypoint(const QGeoCoordinate& location)
{
    Waypoint *wp = createWaypoint();
    wp->setLocation(location);
    update();
    return wp;
}

void SurveyArea::removeWaypoint(Waypoint* wp)
{
    autonomousVehicleProject()->deleteItem(wp);
}

QList<Waypoint *> SurveyArea::waypoints() const
{
    QList<Waypoint *> ret;
    auto children = childMissionItems();
    for(auto child: children)
    {
        Waypoint *wp = qobject_cast<Waypoint*>(child);
        if(wp)
            ret.append(wp);
    }
    return ret;
}

QList<QList<QGeoCoordinate> > SurveyArea::getLines() const
{
    QList<QList<QGeoCoordinate> > ret;
    return ret;
}

void SurveyArea::write(QJsonObject& json) const
{
    MissionItem::write(json);
    json["type"] = "SurveyArea";

    QJsonArray childrenArray;
    for(MissionItem *item: childMissionItems())
    {
        QJsonObject miObject;
        item->write(miObject);
        childrenArray.append(miObject);
    }
    
    json["children"] = childrenArray;
}

void SurveyArea::writeToMissionPlan(QJsonArray& navArray) const
{
    QJsonObject navItem;
    navItem["pathtype"] = "area";
    writeBehaviorsToMissionPlanObject(navItem);
    QJsonArray pathNavArray;
    auto children = childItems();
    for(auto child: children)
    {
        if(child->type() == GeoGraphicsItem::WaypointType)
        {
            Waypoint *wp = qgraphicsitem_cast<Waypoint*>(child);
            wp->writeNavToMissionPlan(pathNavArray);
        }
    }
    navItem["nav"] = pathNavArray;
    navArray.append(navItem);
}

void SurveyArea::read(const QJsonObject& json)
{
}

void SurveyArea::updateProjectedPoints()
{
    for(auto wp: waypoints())
        wp->updateProjectedPoints();
}

bool SurveyArea::canAcceptChildType(const std::string& childType) const
{
    if (childType == "Waypoint") return true;
    if (childType == "SurveyPattern") return true;
    return false;
}

void SurveyArea::generateAdaptiveTrackLines()
{
    double stepSize = 10; // length of segments used to advance the paths
    double swathAngle = 120; // angle of swath in degrees
    double tanHalfSwath = tan((swathAngle/2.0)*M_PI/180.0);

    auto wps = waypoints();
    
    BackgroundRaster *depthRaster = autonomousVehicleProject()->getDepthRaster();
    
    if(wps.size() > 2 && depthRaster)
    {

        BPolygon area_poly;
        
        for(auto wp: waypoints())
        {
            BPoint p(wp->location().longitude(), wp->location().latitude());
            area_poly.outer().push_back(p);
        }
        area_poly.outer().push_back(area_poly.outer().front());

        boost::geometry::validity_failure_type failure;
        bool valid = boost::geometry::is_valid(area_poly, failure);

        if(!valid)
            boost::geometry::correct(area_poly);
        // 1 stbd, -1 port
        int side = 1;
        if(!valid && failure == boost::geometry::failure_wrong_orientation)
            side = -1;

        
        // represents the edge we are trying to follow.
        std::vector<QGeoCoordinate> guidePath;
        
        // Initialize with first edge of polygon, split up in stepSize chunks.
        guidePath.push_back(wps[0]->location());
        double distance = guidePath[0].distanceTo(wps[1]->location());
        double heading = guidePath[0].azimuthTo(wps[1]->location());
        for(double i = stepSize; i < distance; i += stepSize)
            guidePath.push_back(guidePath[0].atDistanceAndAzimuth(i,heading));
        guidePath.push_back(wps[1]->location());
        
        bool done = false;
        int temp_count_until_coded_properly = 0;
        
        while(!done)
        {
            temp_count_until_coded_properly += 1;
            qDebug() << "SurveyArea::generateAdaptiveTrackLines trackline number: " << temp_count_until_coded_properly;
            
            std::vector<QGeoCoordinate> nextTrackLine = generateNextLine(guidePath, *depthRaster, tanHalfSwath, side);
            
            QString tllabel = "trackline"+QString::number(childMissionItems().size());
            TrackLine *tl = createMissionItem<TrackLine>(tllabel);
            for(auto l: nextTrackLine)
                tl->addWaypoint(l);
            
            // generate a new guide path based on the previous line
            // order will need to be reversed
            std::vector<QGeoCoordinate> newGuidePath = generateNextLine(nextTrackLine, *depthRaster, tanHalfSwath, side);

            guidePath.clear();
            for(auto p = newGuidePath.rbegin(); p != newGuidePath.rend(); p++)
                guidePath.push_back(*p);

            QString debug_tllabel = "debug_trackline"+QString::number(childMissionItems().size());
            TrackLine *debug_tl = createMissionItem<TrackLine>(debug_tllabel);
            for(auto l: guidePath)
                debug_tl->addWaypoint(l);
            
            side *= -1;

            if (temp_count_until_coded_properly > 5)
                done = true;
        }
    }
}

std::vector<QGeoCoordinate> SurveyArea::generateNextLine(std::vector<QGeoCoordinate> const &guidePath, BackgroundRaster const &depthRaster, double tanHalfSwath, int side)
{
    typedef std::pair<QGeoCoordinate,QGeoCoordinate> Segment;
    std::vector<Segment> segments;

    // for each segment of the guide path...
    for(int i = 0; i < guidePath.size()-1; i++)
    {
        double depth1 = depthRaster.getDepth(guidePath[i]);
        double swath_half_width1 = depth1*tanHalfSwath;
        double depth2 = depthRaster.getDepth(guidePath[i+1]);
        double swath_half_width2 = depth2*tanHalfSwath;
        double heading = guidePath[i].azimuthTo(guidePath[i+1]);
        segments.push_back(Segment(guidePath[i].atDistanceAndAzimuth(swath_half_width1,heading+(90*side)),guidePath[i+1].atDistanceAndAzimuth(swath_half_width2,heading+(90*side))));
    }
    
    std::vector<QGeoCoordinate> ret;
    ret.push_back(segments.front().first);
    for(int i = 0; i < segments.size()-1; i++)
    {
        BLineString s1, s2;
        s1.push_back(BPoint(segments[i].first.longitude(),segments[i].first.latitude()));
        s1.push_back(BPoint(segments[i].second.longitude(),segments[i].second.latitude()));
        s2.push_back(BPoint(segments[i+1].first.longitude(),segments[i+1].first.latitude()));
        s2.push_back(BPoint(segments[i+1].second.longitude(),segments[i+1].second.latitude()));
        BMultiPoint intersection;
        boost::geometry::intersection(s1,s2,intersection);
        qDebug() << "SurveyArea::generateNextLine: intersection count: " << intersection.size();
        ret.push_back(QGeoCoordinate(s2.front().y(),s2.front().x()));
    }
    ret.push_back(segments.back().second);
    return ret;
}
