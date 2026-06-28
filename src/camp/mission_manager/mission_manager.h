#ifndef CAMP_MISSION_MANAGER_H
#define CAMP_MISSION_MANAGER_H

#include <QWidget>
#include "ros/ros_widget.h"
#include "marine_interfaces/msg/geo_occupancy_vector_map.hpp"
#include "std_msgs/msg/string.hpp"

namespace Ui
{
class MissionManager;
}

class QGeoCoordinate;

class MissionManager: public camp_ros::ROSWidget
{
  Q_OBJECT

public:
  explicit MissionManager(QWidget *parent =0);
  ~MissionManager(); 

public slots:
  void updateRobotNamespace(QString robot_namespace);

  void sendCommand(const QString& command);

  void sendNextItem();
  void restartMission();
  void clearTasks();
  void sendCancelOverride();

  void sendMissionPlan(QString const &plan);
  void appendMission(QString const &plan);
  void prependMission(QString const &plan);
  void updateMission(QString const &plan);
  
  void sendHover(QGeoCoordinate const &targetLocation);
  void sendGoto(QGeoCoordinate const &targetLocation);
  void sendIdle();

  void sendGotoLine(int waypoint_index);
  void sendStartLine(int waypoint_index);

  void sendAvoidanceAreas(marine_interfaces::msg::GeoOccupancyVectorMap& map);


private slots:
  void on_gotoLinePushButton_clicked(bool checked);
  void on_startLinePushButton_clicked(bool checked);
  void on_cancelOverridePushButton_clicked(bool checked);

  void on_nextMissionItemPushButton_clicked(bool checked);
  void on_restartMissionPushButton_clicked(bool checked);
  void on_clearTasksPushButton_clicked(bool checked);

private:
  Ui::MissionManager* m_ui;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr send_command_publisher_;
  rclcpp::Publisher<marine_interfaces::msg::GeoOccupancyVectorMap>::SharedPtr send_avoidance_costmap_publisher_;


};


#endif