#include "group.h"

Group::Group(QObject* parent):MissionItem(parent)
{

}

void Group::write(QJsonObject& json) const
{

}

void Group::read(const QJsonObject& json)
{

}

void Group::updateProjectedPoints()
{
    for(auto child: childMissionItems())
        child->updateProjectedPoints();
}
