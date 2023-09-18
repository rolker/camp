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
    bo["enabled"]=enabled_;
    behaviorsObject[m_behaviorType] = bo;
}

void Behavior::write(QJsonObject& json) const
{
    json["type"] = "Behavior";
    json["behaviorType"] = m_behaviorType;
    json["data"] = data_;
    json["enabled"] = enabled_;
    MissionItem::write(json);
}

void Behavior::read(const QJsonObject& json)
{
    MissionItem::read(json);
    setBehaviorType(json["behaviorType"].toString());
    enabled_ = json["enabled"].toBool();
    data_ = json["data"].toString();
}

bool Behavior::enabled() const
{
    return enabled_;
}

void Behavior::setEnabled(bool enabled)
{
    enabled_ = enabled;
}

void Behavior::setBehaviorType(const QString& behaviorType)
{
    m_behaviorType = behaviorType;
    //setObjectName("behavior - "+m_behaviorType);
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