#include "missionitem.h"
#include "autonomousvehicleproject.h"

MissionItem::MissionItem(QObject *parent) : QObject(parent), m_item(nullptr)
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

void MissionItem::setItem(QStandardItem *item)
{
    m_item = item;
}


QStandardItem * MissionItem::item() const
{
    return m_item;
}
