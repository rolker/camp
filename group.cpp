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

QList<MissionItem *> Group::childMissionItems() const
{
    QList<MissionItem*> ret;
    for(auto child: children())
    {
        MissionItem * childItem = qobject_cast<MissionItem*>(child);
        if(childItem)
            ret.append(childItem);
    }
    return ret;
}

void Group::updateProjectedPoints()
{
    for(auto child: childMissionItems())
        child->updateProjectedPoints();
}
