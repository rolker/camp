#include "group.h"

#include<QJsonObject>
#include<QJsonArray>
#include"autonomousvehicleproject.h"

Group::Group(MissionItem* parent):MissionItem(parent)
{

}

void Group::write(QJsonObject& json) const
{
    MissionItem::write(json);
    json["type"] = "Group";
    
    QJsonArray childrenArray;
    for(MissionItem *item: childMissionItems())
    {
        QJsonObject miObject;
        item->write(miObject);
        childrenArray.append(miObject);
    }
    
    json["children"] = childrenArray;
}

void Group::writeToMissionPlan(QJsonArray& navArray) const
{
    for(MissionItem *item: childMissionItems())
        item->writeToMissionPlan(navArray);
}

void Group::read(const QJsonObject& json)
{
    MissionItem::read(json);
    readChildren(json["children"].toArray());
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
