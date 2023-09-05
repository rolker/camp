#include "orbit.h"

#include<QJsonObject>
#include<QJsonArray>
#include"autonomousvehicleproject.h"

Orbit::Orbit(MissionItem* parent, int row):MissionItem(parent, row)
{

}

void Orbit::write(QJsonObject& json) const
{
  MissionItem::write(json);
  json["type"] = "Orbit";
  json["radius"] = radius_;
  json["safetyDistance"] = safety_distance_;
  json["targetFrame"] = target_frame_.c_str();

  /// \todo why does an orbit have children?
  QJsonArray childrenArray;
  for(MissionItem *item: childMissionItems())
  {
    QJsonObject miObject;
    item->write(miObject);
    childrenArray.append(miObject);
  }
  
  json["children"] = childrenArray;
}

void Orbit::writeToMissionPlan(QJsonArray& navArray) const
{

  for(MissionItem *item: childMissionItems())
    item->writeToMissionPlan(navArray);
}

void Orbit::read(const QJsonObject& json)
{
  MissionItem::read(json);
  readChildren(json["children"].toArray());
}

void Orbit::updateProjectedPoints()
{
  for(auto child: childMissionItems())
    child->updateProjectedPoints();
}

bool Orbit::canAcceptChildType(const std::string& childType) const
{
  if(childType == "Waypoint")
    return true;
  return false;
}

bool Orbit::canBeSentToRobot() const
{
  return true;
}

double Orbit::radius() const
{
  return radius_;
}

double Orbit::safetyDistance() const
{
  return safety_distance_;
}

const std::string& Orbit::targetFrame() const
{
  return target_frame_;
}

void Orbit::setRadius(double radius)
{
  radius_ = radius;
}

void Orbit::setSafetyDistance(double distance)
{
  safety_distance_ = distance;
}

void Orbit::setTargetFrame(std::string target_frame)
{
  target_frame_ = target_frame;
}
