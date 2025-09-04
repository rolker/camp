#include "group.h"

#include<QJsonObject>
#include<QJsonArray>
#include"autonomousvehicleproject.h"

Group::Group(MissionItem* parent, int row):MissionItem(parent, row)
{

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
