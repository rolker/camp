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
}

void Orbit::writeToMissionPlan(QJsonArray& navArray) const
{

  for(MissionItem *item: childMissionItems())
    item->writeToMissionPlan(navArray);
}

void Orbit::read(const QJsonObject& json)
{
  MissionItem::read(json);
  radius_ = json["radius"].toDouble();
  safety_distance_ = json["safetyDistance"].toDouble();
  target_frame_ = json["targetFrame"].toString().toStdString();
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
  return MissionItem::canAcceptChildType(childType);
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
