#include "missionitem.h"
#include "autonomousvehicleproject.h"
#include <QStandardItem>

MissionItem::MissionItem(QObject *parent) : QObject(parent)
{

}

AutonomousVehicleProject* MissionItem::autonomousVehicleProject() const
{
    QObject *o = parent();
    while(o)
    {
        AutonomousVehicleProject * avp = qobject_cast<AutonomousVehicleProject*>(o);
        if(avp)
            return avp;
        o = o->parent();
    }
    return nullptr;
}

void MissionItem::updateProjectedPoints()
{
}

QList<MissionItem *> MissionItem::childMissionItems() const
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

int MissionItem::row() const
{
    MissionItem *item = qobject_cast<MissionItem*>(parent());
    if(item)
        return item->childMissionItems().indexOf(const_cast<MissionItem*>(this));
    return 0;
}
