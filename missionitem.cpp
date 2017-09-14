#include "missionitem.h"
#include "autonomousvehicleproject.h"
#include <QStandardItem>

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

QStandardItem * MissionItem::createItem(QString const &label)
{
    m_item = new QStandardItem(label);
    m_item->setData(QVariant::fromValue(reinterpret_cast<quintptr>(this)));
    m_item->setFlags(m_item->flags()&~(Qt::ItemIsDropEnabled));
    return m_item;
}

QStandardItem * MissionItem::item() const
{
    return m_item;
}

void MissionItem::updateProjectedPoints()
{
}
