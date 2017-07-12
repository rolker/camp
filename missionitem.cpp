#include "missionitem.h"
#include "autonomousvehicleproject.h"

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
