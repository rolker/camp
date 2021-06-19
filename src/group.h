#ifndef GROUP_H
#define GROUP_H

#include "missionitem.h"

class Group : public MissionItem
{
    Q_OBJECT

public:
    Group(MissionItem *parent = 0, int row = -1);
    
    void write(QJsonObject &json) const override;
    void writeToMissionPlan(QJsonArray & navArray) const override;
    void read(const QJsonObject &json);
    
    bool canAcceptChildType(const std::string & childType) const override;
    bool canBeSentToRobot() const override;
    
public slots:
    void updateProjectedPoints() override;
};

#endif // GROUP_H
