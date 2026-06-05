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
#include "map_view/web_mercator.h"
#include <QMessageBox>
#include <algorithm>
#include <cmath>

TrackLine::TrackLine(MissionItem *parent, int row) :GeoGraphicsMissionItem(parent, row)
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
}

void TrackLine::writeToMissionPlan(QJsonArray& navArray) const
{
    QJsonObject navItem;
    navItem["pathtype"] = "trackline";
    navItem["type"] = "survey_line";
    writeBehaviorsToMissionPlanObject(navItem);
    QJsonArray pathNavArray;
    auto children = childMissionItems();
    for(auto child: children)
    {
        Waypoint *wp = qobject_cast<Waypoint*>(child);
        if(wp)
            wp->writeNavToMissionPlan(pathNavArray);
    }
    navItem["nav"] = pathNavArray;
    navArray.append(navItem);
}

void TrackLine::read(const QJsonObject &json)
{
    MissionItem::read(json);
}

bool TrackLine::readGeoJson(const QJsonObject &json)
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
      if(geomType == "LineString")
      {
        if(geom.contains("coordinates") && geom["coordinates"].isArray())
        {
          const auto& coords = geom["coordinates"].toArray();
          for(const auto& c: coords)
          {
            if(c.isArray())
            {
              const auto& point = c.toArray();
              if(point.size() >= 2)
              {
                double lon = point[0].toDouble();
                double lat = point[1].toDouble();
                double alt = 0.0;
                if(point.size() >= 3)
                  alt = point[2].toDouble();
                QGeoCoordinate position(lat, lon, alt);
                addWaypoint(position);
              }
            }
          }
          return true;
        }
      }
    }
  }
  return false;
}

void TrackLine::writeGeoJson(QJsonObject & json, QString name) const
{
  MissionItem::writeGeoJson(json, name);
  QJsonObject geometry;
  geometry["type"] = "LineString";
  QJsonArray coordinates;
  for(auto wp: waypoints())
  {
    QJsonArray point;
    point.append(wp->location().longitude());
    point.append(wp->location().latitude());
    point.append(wp->location().altitude());
    coordinates.append(point);
  }
  geometry["coordinates"] = coordinates;
  json["geometry"] = geometry;
  json["type"] = "Feature";
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
    if(childType == "Waypoint")
        return true;
    return MissionItem::canAcceptChildType(childType);
}

bool TrackLine::canBeSentToRobot() const
{
    return true;
}

void TrackLine::planPath()
{
    // [#59 PR3c] A* plans on a self-defined square grid in Web-Mercator metres
    // (fixed cell count), independent of any raster's pixel grid. Depth per cell
    // comes from AutonomousVehicleProject::getDepth(geo); cells with no coverage
    // are obstacles (unknown = unsafe). See ADR-0002.
    auto wps = waypoints();
    AutonomousVehicleProject* avp = autonomousVehicleProject();
    // avp can be null for an item not yet attached to a project (construction/load).
    if(!avp || wps.size() < 2 || !avp->hasDepth())
        return;

    const int N = 256;                  // fixed grid cell count per axis
    const double marginFraction = 0.5;  // expand the start->goal bbox by this fraction per side

    std::vector<QGeoCoordinate> newWaypoints;
    newWaypoints.push_back(wps[0]->location());  // exact first endpoint (never cell-snapped)
    int failedSegments = 0;

    for (int i = 0; i < wps.size()-1; i++)
    {
        const QPointF startWM = web_mercator::geoToMap(wps[i]->location());
        const QPointF finishWM = web_mercator::geoToMap(wps[i+1]->location());

        // Square planning box centred on the segment, expanded by the margin.
        const double cx = (startWM.x() + finishWM.x())/2.0;
        const double cy = (startWM.y() + finishWM.y())/2.0;
        double extent = std::max(std::abs(finishWM.x()-startWM.x()), std::abs(finishWM.y()-startWM.y()));
        if(extent <= 0.0)
            extent = 1.0;
        const double half = extent/2.0 + extent*marginFraction;
        const double originX = cx - half;
        const double originY = cy - half;
        const double cellSize = (2.0*half) / N;

        // Pre-sample depth at each cell centre (geo query, scene-independent).
        astar::Context c;
        c.gridSize = N;
        c.depthGrid.resize(static_cast<size_t>(N)*N);
        for(int gy = 0; gy < N; gy++)
            for(int gx = 0; gx < N; gx++)
            {
                const QPointF centre(originX + (gx+0.5)*cellSize, originY + (gy+0.5)*cellSize);
                const float d = avp->getDepth(web_mercator::mapToGeo(centre));
                c.depthGrid[static_cast<size_t>(gy)*N + gx] = std::isnan(d) ? astar::Context::unknownDepth : d;
            }

        auto toCell = [&](const QPointF& p)
        {
            const int gx = std::clamp(int((p.x()-originX)/cellSize), 0, N-1);
            const int gy = std::clamp(int((p.y()-originY)/cellSize), 0, N-1);
            return astar::Position(gx, gy);
        };
        c.start = toCell(startWM);
        c.finish = toCell(finishWM);
        c.maxDepth = 15.0;
        c.minDepth = 3.0;
        c.shipDraft = 1.0;

        astar::AStar as;
        auto result = as.search(c);
        if(result.empty())
            ++failedSegments;
        // Emit only the A* INTERIOR cells; the exact segment endpoints are kept
        // (wps[i+1] below, and wps[0] before the loop). This avoids cell-snap
        // drift at the endpoints and keeps segment joins continuous (the shared
        // waypoint is the same exact coordinate in both segments, not quantized
        // into two different grids). On failure result is empty -> a straight
        // segment to the exact endpoint.
        for(size_t k = 1; k + 1 < result.size(); k++)
        {
            const auto& p = result[k];
            const QPointF centre(originX + (p.x+0.5)*cellSize, originY + (p.y+0.5)*cellSize);
            newWaypoints.push_back(web_mercator::mapToGeo(centre));
        }
        newWaypoints.push_back(wps[i+1]->location());  // exact segment end (= next segment's exact start)
    }
    for(auto wp: wps)
        removeWaypoint(wp);

    for(const auto& nwp: newWaypoints)
        addWaypoint(nwp);

    // [#59 PR3c] A* failure falls back to a straight segment, which can cross the
    // unknown/too-shallow water the planner meant to avoid. Warn the operator so a
    // silent unsafe leg isn't mistaken for a planned one.
    if(failedSegments > 0)
        QMessageBox::warning(nullptr, tr("Plan path"),
            tr("%1 of %2 segment(s) could not be planned around obstacles or "
               "unsurveyed water; a straight line was used for those segments. "
               "Review the route before sending it to the boat.")
            .arg(failedSegments).arg(int(wps.size()-1)));
}
