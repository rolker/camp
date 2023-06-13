#include "searchpattern.h"
#include "waypoint.h"
#include <QPainter>
#include <QtMath>
#include <QJsonObject>
#include <QJsonArray>
#include "autonomousvehicleproject.h"
#include "surveyarea.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace bg = boost::geometry;

SearchPattern::SearchPattern(MissionItem *parent, int row):GeoGraphicsMissionItem(parent, row)
{
  setShowLabelFlag(true);  
}

bool SearchPattern::canBeSentToRobot() const
{
  return true;
}

Waypoint * SearchPattern::createWaypoint()
{
  Waypoint * wp = createMissionItem<Waypoint>();
  wp->setFlag(QGraphicsItem::ItemIsMovable);
  wp->setFlag(QGraphicsItem::ItemIsSelectable);
  wp->setFlag(QGraphicsItem::ItemSendsGeometryChanges);
  wp->setFlag(QGraphicsItem::ItemSendsScenePositionChanges);
  connect(wp, &Waypoint::waypointMoved, this, &SearchPattern::waypointHasChanged);
  connect(wp, &Waypoint::waypointAboutToMove, this, &SearchPattern::waypointAboutToChange);
  return wp;
}

void SearchPattern::setStartLocation(const QGeoCoordinate &location)
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

void SearchPattern::setEndLocation(const QGeoCoordinate &location, bool calc)
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

void SearchPattern::setSpacingLocation(const QGeoCoordinate &location, bool calc)
{
  if(m_spacingLocation == nullptr)
  {
    m_spacingLocation = createWaypoint();
    m_spacingLocation->setObjectName("firstLeg");
  }
  m_spacingLocation->setLocation(location);
  if(calc)
    calculateFromWaypoints();
  update();
}

void SearchPattern::calculateFromWaypoints()
{
  if(m_startLocation && m_endLocation)
  {
    m_searchRadius = m_startLocation->location().distanceTo(m_endLocation->location());
    if(m_spacingLocation)
    {
      m_firstLegDistance = m_startLocation->location().distanceTo(m_spacingLocation->location());
      m_firstLegHeading = m_startLocation->location().azimuthTo(m_spacingLocation->location());
    }
  }
}

void SearchPattern::write(QJsonObject &json) const
{
  MissionItem::write(json);
  json["type"] = "SearchPattern";
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
  json["first_leg_heading"] = m_firstLegHeading;
  json["first_leg_distance"] = m_firstLegDistance;
  switch(m_direction)
  {
  case clockwise:
    json["direction"] = "clockwise";
    break;
  case counterclockwise:
    json["direction"] = "counterclockwise";
    break;
  }
  QJsonArray tracklineArray;
  auto lines = getLines();
  for (auto line: lines)
  {
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

void SearchPattern::writeToMissionPlan(QJsonArray& navArray) const
{
  auto lines = getLines();
  for(int i = 0; i < lines.size(); i++)
  {
    auto l = lines[i];
    QJsonObject navItem;
    navItem["pathtype"] = "trackline";
    QJsonObject params;
    if (m_speed>0.0)
      params["speed_ms"] = m_speed*0.514444; // knots to m/s
    navItem["parameters"] = params;
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

void SearchPattern::read(const QJsonObject &json)
{
  m_startLocation = createWaypoint();
  m_startLocation->read(json["startLocation"].toObject());
  m_endLocation = createWaypoint();
  m_endLocation->read(json["endLocation"].toObject());
  setFirstLeg(json["first_leg_heading"].toDouble(),json["first_leg_distance"].toDouble());
  if(json.contains("direction"))
  {
    if(json["direction"] == "clockwise")
      m_direction = clockwise;
    if(json["direction"] == "counterclockwise")
      m_direction = counterclockwise;
  }
  calculateFromWaypoints();
}

bool SearchPattern::hasSpacingLocation() const
{
  return (m_spacingLocation != nullptr);
}

SearchPattern::Direction SearchPattern::direction() const
{
  return m_direction;
}

Waypoint * SearchPattern::startLocationWaypoint() const
{
    return m_startLocation;
}

Waypoint * SearchPattern::endLocationWaypoint() const
{
    return m_endLocation;
}

void SearchPattern::setFirstLeg(double heading, double distance)
{
  QGeoCoordinate c = m_startLocation->location().atDistanceAndAzimuth(distance, heading);
  m_internalUpdateFlag = true;
  setSpacingLocation(c,false);
  m_internalUpdateFlag = false;
}

void SearchPattern::updateEndLocation()
{
  m_internalUpdateFlag = true;
  double direction = 90.0;
  double radius = std::max(1.0, m_searchRadius);
  if(m_startLocation && m_endLocation)
    if(m_startLocation->location() != m_endLocation->location())
      direction = m_startLocation->location().azimuthTo(m_endLocation->location());

  QGeoCoordinate p = m_startLocation->location().atDistanceAndAzimuth(radius, direction);
  setEndLocation(p,false);
  m_internalUpdateFlag = false;
}


QRectF SearchPattern::boundingRect() const
{
  return shape().boundingRect().marginsAdded(QMarginsF(5,5,5,5));
}

void SearchPattern::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
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
        
        first++;
        second++;
      }

      if(!lines.empty() && lines[0].size() >= 2)
      {
        QPainterPath ret(m_startLocation->geoToPixel(lines[0][0],autonomousVehicleProject()));
        drawArrow(ret,m_startLocation->geoToPixel(lines[0][0],autonomousVehicleProject()),m_startLocation->geoToPixel(lines[0][1],autonomousVehicleProject()), true);
        painter->drawPath(ret);
      }
      if(!lines.empty() && lines.rbegin()->size() >= 2)
      {
        int start_index = lines.rbegin()->size()-2;
        int end_index = start_index + 1;
        QPainterPath ret(m_startLocation->geoToPixel((*lines.rbegin())[start_index],autonomousVehicleProject()));
        drawArrow(ret,m_startLocation->geoToPixel((*lines.rbegin())[start_index],autonomousVehicleProject()),m_startLocation->geoToPixel((*lines.rbegin())[end_index],autonomousVehicleProject()), false);
        painter->drawPath(ret);
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

QPainterPath SearchPattern::shape() const
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

QList<QList<QGeoCoordinate> > SearchPattern::getLines() const
{
  QList<QList<QGeoCoordinate> > ret;
  if(m_startLocation && m_searchRadius > 0.0 && m_firstLegDistance > 0.0)
  {
    double current_heading = m_firstLegHeading;
    double current_distance = m_firstLegDistance;
    auto current_location = m_startLocation->location();
    std::vector<double> distances;

    while (current_location.distanceTo(m_startLocation->location()) < m_searchRadius)
    {
      auto next_location = current_location.atDistanceAndAzimuth(current_distance, current_heading);
      QList<QGeoCoordinate> line;
      line.append(current_location);
      line.append(next_location);
      ret.append(line);
      if(m_direction == counterclockwise)
        current_heading -= 90.0;
      else
        current_heading += 90.0;
      distances.push_back(current_distance);
      if(distances.size() >= 2)
        current_distance = distances[distances.size()-2]+distances.front();
      current_location = next_location;
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

void SearchPattern::waypointAboutToChange()
{
  prepareGeometryChange();
}

void SearchPattern::waypointHasChanged(Waypoint *wp)
{
  if(!m_internalUpdateFlag)
    calculateFromWaypoints();
  updateETE();
  emit searchPatternUpdated();
}

void SearchPattern::updateProjectedPoints()
{
  if(m_startLocation)
    m_startLocation->updateProjectedPoints();
  if(m_endLocation)
    m_endLocation->updateProjectedPoints();
  if(m_spacingLocation)
    m_spacingLocation->updateProjectedPoints();
}

void SearchPattern::switchDirection()
{
    prepareGeometryChange();

    if(m_direction == clockwise)
      m_direction = counterclockwise;
    else
      m_direction = clockwise;
    
    calculateFromWaypoints();
    
    emit searchPatternUpdated();

    update();
}


void SearchPattern::updateETE()
{
  GeoGraphicsMissionItem::updateETE();
  AutonomousVehicleProject* avp = autonomousVehicleProject();
  if(avp && m_startLocation)
  {
    setLabelPosition(m_startLocation->geoToPixel(m_startLocation->location(), autonomousVehicleProject()));
  }
}
