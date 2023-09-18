#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include "missionitem.h"

class Behavior : public MissionItem
{
    Q_OBJECT
public:
    explicit Behavior(MissionItem *parent= 0, int row = -1);

    void write(QJsonObject& json) const override;
    void writeToMissionPlan(QJsonArray& navArray) const override;
    void writeToMissionPlanObject(QJsonObject& behaviorsObject) const;
    void read(const QJsonObject& json) override;
    bool canBeSentToRobot() const override;
    
    QString const &behaviorType() const;
    QString const &behaviorData() const;
    bool enabled() const;
    
    void setBehaviorType(QString const & behaviorType);
    void setEnabled(bool enabled);
    void setBehaviorData(QString const & behaviorData);

private:
    QString m_behaviorType;
    bool enabled_;
    QString data_;
};

#endif // BEHAVIOR_H
