#include "group.h"

Group::Group(QObject* parent)
{

}

void Group::write(QJsonObject& json) const
{

}

void Group::read(const QJsonObject& json)
{

}

QList<MissionItem *> Group::childItems() const
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

QStandardItem * Group::createItem(const QString& label)
{
    return createItemDetails<Group>(label);
}
