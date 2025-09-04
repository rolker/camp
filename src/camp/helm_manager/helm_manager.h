#ifndef CAMP_HELM_MANAGER_H
#define CAMP_HELM_MANAGER_H

#include <QWidget>
#include "rclcpp/rclcpp.hpp"
#include "project11_msgs/msg/heartbeat.hpp"
#include "std_msgs/msg/string.hpp"

namespace Ui
{
class HelmManager;
}

class HelmManager: public QWidget
{
  Q_OBJECT

public:
  explicit HelmManager(QWidget *parent =0);
  ~HelmManager(); 

  void setNode(rclcpp::Node::SharedPtr node);

signals:
  void pilotingModeUpdated(QString piloting_mode);
  void heartbeatTimesUpdated(double last_heartbeat_timestamp, double last_heartbeat_receive_time);
  void robotStatusUpdated(QString status);
  void requestPilotingMode(QString piloting_mode);

public slots:
  void updateRobotNamespace(QString robot_namespace);
  void sendPilotingModeRequest(QString piloting_mode);

private slots:
  void on_standbyPushButton_clicked(bool checked);
  void on_autonomousPushButton_clicked(bool checked);

  void on_timeLatencyConfigPushButton_clicked(bool checked);
  
  void updatePilotingMode(QString const &piloting_mode);
  void updateHeartbeatTimes(double last_heartbeat_timestamp, double last_heartbeat_receive_time);
  void watchdogUpdate();

private:
  void heartbeatCallback(const project11_msgs::msg::Heartbeat& message);

  Ui::HelmManager* ui;

  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<project11_msgs::msg::Heartbeat>::SharedPtr heartbeat_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr send_command_publisher_;

  rclcpp::Time last_heartbeat_timestamp_;
  rclcpp::Time last_heartbeat_receive_time_;
    
  QTimer * m_watchdog_timer;

  rclcpp::Duration max_green_duration_ = rclcpp::Duration(2, 0);
  rclcpp::Duration max_yellow_duration_ = rclcpp::Duration(5, 0);

};

#endif
