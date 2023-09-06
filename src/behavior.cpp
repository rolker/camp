#include "behavior.h"
#include <QJsonObject>

Behavior::Behavior(MissionItem *parent, int row): MissionItem(parent, row)
{
}

void Behavior::writeToMissionPlan(QJsonArray& navArray) const
{

}

void Behavior::writeToMissionPlanObject(QJsonObject& behaviorsObject) const
{
    QJsonObject bo;
    bo["active"]=m_active;
    behaviorsObject[m_behaviorType] = bo;
}

void Behavior::write(QJsonObject& json) const
{
    json["type"] = "Behavior";
    json["behaviorType"] = m_behaviorType;
    json["data"] = data_;
    json["active"] = m_active;
}

void Behavior::read(const QJsonObject& json)
{
    MissionItem::read(json);
    setBehaviorType(json["behaviorType"].toString());
    m_active = json["active"].toBool();
    data_ = json["data"].toString();
}

bool Behavior::active() const
{
    return m_active;
}

void Behavior::setActive(bool active)
{
    m_active = active;
}

void Behavior::setBehaviorType(const QString& behaviorType)
{
    m_behaviorType = behaviorType;
    setObjectName("behavior - "+m_behaviorType);
}

void Behavior::setBehaviorData(const QString& behaviorData)
{
    data_ = behaviorData;
}

const QString & Behavior::behaviorType() const
{
    return m_behaviorType;
}

const QString & Behavior::behaviorData() const
{
    return data_;
}

bool Behavior::canBeSentToRobot() const
{
    return true;
}