#include "mission_manager.h"
#include "ui_mission_manager.h"
#include <QGeoCoordinate>

MissionManager::MissionManager(QWidget *parent)
  :camp_ros::ROSWidget(parent), m_ui(new Ui::MissionManager)
{
  m_ui->setupUi(this);
}

MissionManager::~MissionManager()
{

}

void MissionManager::updateRobotNamespace(QString robot_namespace)
{
  if(node_)
  {
    // Mission status is now shown by the structured RunningTasksView; this
    // widget is a command panel and no longer subscribes to the Heartbeat.
    send_command_publisher_ = node_->create_publisher<std_msgs::msg::String>("/"+robot_namespace.toStdString()+"/marine/send_command",1);

    rclcpp::QoS qos(1);
    qos.transient_local();

    send_avoidance_costmap_publisher_ = node_->create_publisher<marine_interfaces::msg::GeoOccupancyVectorMap>("/"+robot_namespace.toStdString()+"/marine/avoidance_map", qos);
  }
}

void MissionManager::on_gotoLinePushButton_clicked(bool checked)
{
    //sendGotoLine(m_ui->lineNumberSpinBox->value());
}

void MissionManager::on_startLinePushButton_clicked(bool checked)
{
    //sendStartLine(m_ui->lineNumberSpinBox->value());
}

void MissionManager::on_nextMissionItemPushButton_clicked(bool checked)
{
  sendNextItem();
}

void MissionManager::on_restartMissionPushButton_clicked(bool checked)
{
  restartMission();
}

void MissionManager::on_clearTasksPushButton_clicked(bool checked)
{
  clearTasks();
}

void MissionManager::on_cancelOverridePushButton_clicked(bool checked)
{
  sendCancelOverride();
}

void MissionManager::sendMissionPlan(const QString& plan)
{
    //sendCommand("mission_plan "+plan.toStdString());
  sendCommand("mission_manager replace_task mission_plan "+plan);
//     std_msgs::String mp;
//     mp.data = plan.toStdString();
//     if(m_node)
//         m_mission_plan_publisher.publish(mp);
}

void MissionManager::appendMission(const QString& plan)
{
  sendCommand("mission_manager append_task mission_plan "+plan);
}

void MissionManager::clearTasks()
{
  sendCommand("mission_manager clear_tasks");
}

void MissionManager::sendCancelOverride()
{
  sendCommand("mission_manager cancel_override");
}

void MissionManager::prependMission(const QString& plan)
{
  sendCommand("mission_manager prepend_task mission_plan "+plan);
}

void MissionManager::updateMission(const QString& plan)
{
  sendCommand("mission_manager update_task mission_plan "+plan);
}

void MissionManager::sendHover(const QGeoCoordinate& hoverLocation)
{
  std::stringstream updates;
  updates << std::fixed << std::setprecision(7) << "mission_manager override hover " << hoverLocation.latitude() << " " << hoverLocation.longitude();
        
  sendCommand(updates.str().c_str());
}  

void MissionManager::sendGoto(const QGeoCoordinate& gotoLocation)
{
  std::stringstream updates;
  updates << std::fixed << std::setprecision(7) << "mission_manager override goto " << gotoLocation.latitude() << " " << gotoLocation.longitude();
        
  sendCommand(updates.str().c_str());
}

void MissionManager::sendIdle()
{
  sendCommand("mission_manager override idle");
}


void MissionManager::sendNextItem()
{
  std::stringstream updates;
  updates << "mission_manager next_task";
        
    sendCommand(updates.str().c_str());
}

void MissionManager::restartMission()
{
    std::stringstream updates;
    updates << "mission_manager restart_mission";
        
    sendCommand(updates.str().c_str());
}


void MissionManager::sendGotoLine(int waypoint_index)
{
    std::stringstream updates;
    updates << "goto_line " << waypoint_index;
        
    sendCommand(updates.str().c_str());
}

void MissionManager::sendStartLine(int waypoint_index)
{
    std::stringstream updates;
    updates << "start_line " << waypoint_index;
        
    sendCommand(updates.str().c_str());
}

void MissionManager::sendCommand(const QString& command)
{
    std_msgs::msg::String cmd;
    cmd.data = command.toStdString();
    //qDebug() << command;
    send_command_publisher_->publish(cmd);
}

void MissionManager::sendAvoidanceAreas(marine_interfaces::msg::GeoOccupancyVectorMap& map)
{
  if(node_)
  {
    map.header.stamp = node_->get_clock()->now();
    send_avoidance_costmap_publisher_->publish(map);
  }
}
