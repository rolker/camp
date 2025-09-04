#ifndef ORBIT_H
#define ORBIT_H

#include "missionitem.h"
#include <QVector3D>

class Orbit : public MissionItem
{
  Q_OBJECT

public:
  Orbit(MissionItem *parent = 0, int row = -1);
    
  void write(QJsonObject &json) const override;
  void writeToMissionPlan(QJsonArray & navArray) const override;
  void read(const QJsonObject &json);
    
  bool canAcceptChildType(const std::string & childType) const override;
  bool canBeSentToRobot() const override;

  double radius() const;
  double safetyDistance() const;
  const std::string& targetFrame() const;
  const QVector3D& position() const;

  void setRadius(double radius);
  void setSafetyDistance(double distance);
  void setTargetFrame(std::string target_frame);
  void setPosition(const QVector3D& position);
    
public slots:
  void updateProjectedPoints() override;

private:
  double radius_ = 250.0;
  double safety_distance_ = 0.0;
  std::string target_frame_;
  QVector3D position_ = {0.0, 0.0, 0.0};
};

#endif // ORBIT_H
