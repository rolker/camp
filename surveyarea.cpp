#include "surveyarea.h"
#include "waypoint.h"
#include <QPainter>
#include <QJsonObject>
#include <QJsonArray>
#include "backgroundraster.h"
#include "trackline.h"
#include <QDebug>


SurveyArea::SurveyArea(MissionItem *parent) :GeoGraphicsMissionItem(parent)
{
    setShowLabelFlag(true);
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

        BMultiLineString generated_lines;
        
        while(true)
        {
            std::vector<QGeoCoordinate> nextTrackLine = generateNextLine(guidePath, *depthRaster, tanHalfSwath, side, area_poly, stepSize, generated_lines);
            if(nextTrackLine.empty())
                break;
            
            QString tllabel = "trackline"+QString::number(childMissionItems().size());
            TrackLine *tl = createMissionItem<TrackLine>(tllabel);
            generated_lines.resize(generated_lines.size()+1);
            for(auto l: nextTrackLine)
            {
                tl->addWaypoint(l);
                generated_lines[generated_lines.size()-1].push_back(BPoint(l.longitude(),l.latitude()));
            }
            
            // generate a new guide path based on the previous line
            // order will need to be reversed
            std::vector<QGeoCoordinate> newGuidePath = generateNextLine(nextTrackLine, *depthRaster, tanHalfSwath, side, area_poly, stepSize, generated_lines);
            if(newGuidePath.empty())
                break;

            guidePath.clear();
            for(auto p = newGuidePath.rbegin(); p != newGuidePath.rend(); p++)
                guidePath.push_back(*p);

//             QString debug_tllabel = "debug_trackline"+QString::number(childMissionItems().size());
//             TrackLine *debug_tl = createMissionItem<TrackLine>(debug_tllabel);
//             for(auto l: guidePath)
//                 debug_tl->addWaypoint(l);
//             
            side *= -1;
        }
    }
    updateETE();
}

std::vector<QGeoCoordinate> SurveyArea::generateNextLine(std::vector<QGeoCoordinate> const &guidePath, BackgroundRaster const &depthRaster, double tanHalfSwath, int side, BPolygon const &area_poly, double stepSize, BMultiLineString const & previousLines)
{
    std::vector<QGeoCoordinate> ret;
    for(int i = 0; i < guidePath.size(); i++)
    {
        double depth = depthRaster.getDepth(guidePath[i]);
        // TODO: Improve the following to not assume constant depth across swath.
        double swath_half_width = depth*tanHalfSwath;
        
        // Find the  heading between previous point and next point. Use current point if at either end.
        double heading = guidePath[std::max<int>(0,i-1)].azimuthTo(guidePath[std::min<int>(guidePath.size()-1,i+1)]);
        
        QGeoCoordinate candidate_point = guidePath[i].atDistanceAndAzimuth(swath_half_width,heading+(90*side));
        
        // check if turning too abruptly
        if(ret.size()>=2)
        {
            double last_heading = ret[ret.size()-2].azimuthTo(ret.back());
            double candidate_heading = ret.back().azimuthTo(candidate_point);
            double delta_heading = fabs(candidate_heading-last_heading);
            if(delta_heading > 180)
                delta_heading = 360-delta_heading;
            if(delta_heading < 90)
                ret.push_back(candidate_point);
        }
        else
            ret.push_back(candidate_point);
    }
    
    // remove intersections
    BLineString keptPoints;
    for(int i = 0; i < ret.size(); i++)
    {
        if(keptPoints.size() < 3)
        {
            keptPoints.push_back(BPoint(ret[i].longitude(), ret[i].latitude()));
        }
        else
        {
            BLineString candidateSegment;
            candidateSegment.push_back(keptPoints.back());
            candidateSegment.push_back(BPoint(ret[i].longitude(), ret[i].latitude()));
            std::vector<BPoint> intersections;
            boost::geometry::intersection(keptPoints,candidateSegment,intersections);
            qDebug() << "intersection count: " << intersections.size();
            if(intersections.size() < 2)
                keptPoints.push_back(candidateSegment.back());
        }
    }
    ret.clear();
    for(BPoint p: keptPoints)
        ret.push_back(QGeoCoordinate(p.y(),p.x()));

    // trim to polygon
    // start with removing segments outside the poly from the begining
    while(ret.size() > 1)
    {
        BLineString segment;
        segment.push_back(BPoint(ret[0].longitude(), ret[0].latitude()));
        segment.push_back(BPoint(ret[1].longitude(), ret[1].latitude()));
        if(boost::geometry::intersects(segment,area_poly))
            break;
        ret.erase(ret.begin());
    }
    // now, remove segments from the end that are outside the poly
    while(ret.size() > 1)
    {
        BLineString segment;
        segment.push_back(BPoint(ret[ret.size()-2].longitude(), ret[ret.size()-2].latitude()));
        segment.push_back(BPoint(ret.back().longitude(), ret.back().latitude()));
        if(boost::geometry::intersects(segment,area_poly))
            break;
        ret.pop_back();
    }

    // if we have at least one segment, check if they reach the edge and extend if necessary
    if(ret.size() > 1)
    {
        // extend the front
        while(true)
        {
            BPoint p(ret.front().longitude(),ret.front().latitude());
            if(!boost::geometry::intersects(p,area_poly))
                break;
            double back_heading = ret[1].azimuthTo(ret.front());
            QGeoCoordinate candidate_point = ret.front().atDistanceAndAzimuth(stepSize,back_heading);
            BLineString candidateSegment;
            candidateSegment.push_back(BPoint(candidate_point.longitude(),candidate_point.latitude()));
            candidateSegment.push_back(BPoint(ret.front().longitude(), ret.front().latitude()));
            if(boost::geometry::intersects(candidateSegment,previousLines))
                break;
            ret.insert(ret.begin(),candidate_point);
        }
        
        //extend the back
        while(true)
        {
            BPoint p(ret.back().longitude(), ret.back().latitude());
            if(!boost::geometry::intersects(p,area_poly))
                break;
            double heading = ret[ret.size()-2].azimuthTo(ret.back());
            QGeoCoordinate candidate_point = ret.back().atDistanceAndAzimuth(stepSize,heading);
            BLineString candidateSegment;
            candidateSegment.push_back(BPoint(ret.back().longitude(),ret.back().latitude()));
            candidateSegment.push_back(BPoint(candidate_point.longitude(),candidate_point.latitude()));
            if(boost::geometry::intersects(candidateSegment,previousLines))
                break;
            ret.push_back(candidate_point);
        }
    }

    // if not enough points to form a segment, don't return any
    if(ret.size() < 2)
        ret.clear();

    return ret;
}
