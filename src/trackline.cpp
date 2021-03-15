#include "trackline.h"
#include "waypoint.h"
#include <QPainter>
#include <QJsonObject>
#include <QJsonArray>
#include <QStandardItem>
#include <QDebug>
#include "autonomousvehicleproject.h"
#include "backgroundraster.h"
#include "astar.h"

TrackLine::TrackLine(MissionItem *parent) :GeoGraphicsMissionItem(parent)
{

}

QRectF TrackLine::boundingRect() const
{
    return childrenBoundingRect();
}

void TrackLine::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    auto children = waypoints();
    if (children.length() > 1)
    {
        bool selected = false;
        if(autonomousVehicleProject()->currentSelected() == this)
            selected = true;
        painter->save();

        QPen p;
        p.setCosmetic(true);
        
        if(selected)
        {
            p.setColor(Qt::black);
            p.setWidth(8);
            painter->setPen(p);
            painter->drawPath(shape());
        }
        
        if(locked())
            p.setColor(m_lockedColor);
        else
            p.setColor(m_unlockedColor);
        p.setWidth(3);
        painter->setPen(p);
        painter->drawPath(shape());

        painter->restore();

    }

}

QPainterPath TrackLine::shape() const
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
            drawArrow(ret,(*last)->pos(),(*i)->pos());
            last = i;
            i++;
        }
        return ret;
    }
    return QGraphicsItem::shape();
}


Waypoint * TrackLine::createWaypoint()
{
    int i = childMissionItems().size();
    QString wplabel = "waypoint"+QString::number(i);
    Waypoint *wp = createMissionItem<Waypoint>(wplabel);

    wp->setFlag(QGraphicsItem::ItemIsMovable);
    wp->setFlag(QGraphicsItem::ItemIsSelectable);
    wp->setFlag(QGraphicsItem::ItemSendsGeometryChanges);
    wp->setFlag(QGraphicsItem::ItemSendsScenePositionChanges);
    return wp;
}

Waypoint * TrackLine::addWaypoint(const QGeoCoordinate &location)
{
    Waypoint *wp = createWaypoint();
    wp->setLocation(location);
    emit trackLineUpdated();
    update();
    return wp;
}

void TrackLine::removeWaypoint(Waypoint* wp)
{
    autonomousVehicleProject()->deleteItem(wp);
}


QList<Waypoint *> TrackLine::waypoints() const
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

QList<QList<QGeoCoordinate> > TrackLine::getLines() const
{
    QList<QList<QGeoCoordinate> > ret;
    ret.append(QList<QGeoCoordinate>());
    for(auto wp:waypoints())
        ret.back().append(wp->location());
    return ret;
}


void TrackLine::write(QJsonObject &json) const
{
    MissionItem::write(json);
    json["type"] = "TrackLine";
    QJsonArray wpArray;
    auto children = childItems();
    for(auto child: children)
    {
        if(child->type() == GeoGraphicsItem::WaypointType)
        {
            Waypoint *wp = qgraphicsitem_cast<Waypoint*>(child);
            QJsonObject wpObject;
            wp->write(wpObject);
            wpArray.append(wpObject);
        }
    }
    json["waypoints"] = wpArray;
}

void TrackLine::writeToMissionPlan(QJsonArray& navArray) const
{
    QJsonObject navItem;
    navItem["pathtype"] = "trackline";
    navItem["type"] = "survey_line";
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

void TrackLine::read(const QJsonObject &json)
{
    QJsonArray waypointsArray = json["waypoints"].toArray();
    for(int wpIndex = 0; wpIndex < waypointsArray.size(); wpIndex++)
    {
        QJsonObject wpObject = waypointsArray[wpIndex].toObject();
        if(wpIndex == 0)
        {
            QGeoCoordinate position(wpObject["latitude"].toDouble(),wpObject["longitude"].toDouble());
            setPos(geoToPixel(position,autonomousVehicleProject()));
        }
        Waypoint *wp = createWaypoint();
        wp->read(wpObject);
    }

}

void TrackLine::updateProjectedPoints()
{
    for(auto wp: waypoints())
        wp->updateProjectedPoints();
}

void TrackLine::reverseDirection()
{
    prepareGeometryChange();
    QList<QGeoCoordinate> points;
    for(auto wp: waypoints())
        points.push_back(wp->location());
    for(auto wp: waypoints())
    {
        wp->setLocation(points.back());
        points.pop_back();
    }
    update();
}


bool TrackLine::canAcceptChildType(const std::string& childType) const
{
    return childType == "Waypoint";
}

void TrackLine::planPath()
{
    auto wps = waypoints();
    
    BackgroundRaster *depthRaster = autonomousVehicleProject()->getDepthRaster();

    std::vector<QGeoCoordinate> newWaypoints;
    
    for (int i = 0; i <  wps.size()-1; i++)
    {
        auto start = depthRaster->geoToPixel(wps[i]->location());
        auto finish = depthRaster->geoToPixel(wps[i+1]->location());
        qDebug() << "start: " << start << " finish: " << finish;
        astar::Context c;
        c.start.x = start.x();
        c.start.y = start.y();
        c.finish.x = finish.x();
        c.finish.y = finish.y();
        c.map = depthRaster;
        c.maxDepth = 15.0;
        c.minDepth = 3.0;
        c.shipDraft = 1.0;
        astar::AStar as;
        auto result = as.search(c);
        if(result.empty())
        {
            newWaypoints.push_back(wps[i]->location());
            newWaypoints.push_back(wps[i+1]->location());
        }
        else
            for(auto p: result)
                newWaypoints.push_back(depthRaster->pixelToGeo(QPointF(p.x,p.y)));
    }
    for(auto wp: wps)
        removeWaypoint(wp);

    for(auto nwp: newWaypoints)
        addWaypoint(nwp);
}
