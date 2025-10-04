#include "group.h"

#include<QJsonObject>
#include<QJsonArray>
#include"autonomousvehicleproject.h"
#include"waypoint.h"
#include"trackline.h"

Group::Group(MissionItem* parent, int row):MissionItem(parent, row)
{

}


bool Group::readGeoJsonChildren(const QJsonArray& json)
{
  for(const auto& item: json)
  {
    if(item.isObject())
    {
      const auto& obj = item.toObject();
      std::string name = "GeoJsonItem";
      if(obj.contains("properties") && obj["properties"].isObject())
      {
        const auto& props = obj["properties"].toObject();
        if(props.contains("name") && props["name"].isString())
          name = props["name"].toString().toStdString();
      }
      if(obj["type"] == "Feature")
      {
        readGeoJsonProperties(obj);
        if(obj.contains("geometry") && obj["geometry"].isObject())
        {
          const auto& geom = obj["geometry"].toObject();
          std::string geomType;
          if(geom.contains("type") && geom["type"].isString())
            geomType = geom["type"].toString().toStdString();
          if(geomType == "Point")
          {
            auto wp = createMissionItem<Waypoint>(QString::fromStdString(name));
            if(wp)
              wp->readGeoJson(obj);
          }
          else if(geomType == "LineString")
          {
            auto tl = createMissionItem<TrackLine>(QString::fromStdString(name));
            if(tl)
              tl->readGeoJson(obj);
          }
        }
      }
    }
  }
  return true;
}

void Group::write(QJsonObject& json) const
{
    MissionItem::write(json);
    json["type"] = "Group";
}

void Group::writeToMissionPlan(QJsonArray& navArray) const
{
    for(MissionItem *item: childMissionItems())
        item->writeToMissionPlan(navArray);
}

void Group::writeToGeoJson(QJsonArray& json) const
{
  for(MissionItem *item: childMissionItems())
    item->writeToGeoJson(json);
}

void Group::updateProjectedPoints()
{
    for(auto child: childMissionItems())
        child->updateProjectedPoints();
}

bool Group::canAcceptChildType(const std::string& childType) const
{
    return true;
}

bool Group::canBeSentToRobot() const
{
    return true;
}
