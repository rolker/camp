#ifndef PLATFORM_H
#define PLATFORM_H

#include "missionitem.h"

class Platform : public MissionItem
{
    Q_OBJECT
public:
    explicit Platform(MissionItem *parent = 0);

    void write(QJsonObject &json) const;
    void read(const QJsonObject &json);
    
    double speed() const;
    void setSpeed(double speed);
    
signals:
    void speedChanged();

public slots:

private:
    double m_speed;
};

#endif // PLATFORM_H
