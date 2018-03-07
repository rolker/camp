#ifndef GROUP_H
#define GROUP_H

#include "missionitem.h"

class Group : public MissionItem
{
    Q_OBJECT

public:
    Group(QObject *parent = 0);
    
    void write(QJsonObject &json) const;
    void read(const QJsonObject &json);
    
public slots:
    void updateProjectedPoints() override;
};

#endif // GROUP_H
