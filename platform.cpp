#include "platform.h"
#include <QJsonObject>

Platform::Platform(MissionItem *parent) : MissionItem(parent), m_speed(0.0)
{

}

void Platform::read(const QJsonObject &json)
{
    m_speed = json["speed"].toDouble();
}


void Platform::write(QJsonObject &json) const
{
    json["type"]="Platform";
    json["speed"]=m_speed;
}

void Platform::writeToMissionPlan(QJsonArray& navArray) const
{
    
}

double Platform::speed() const
{
    return m_speed;
}

void Platform::setSpeed(double speed)
{
    m_speed = speed;
    emit speedChanged();
}

