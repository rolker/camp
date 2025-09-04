#include "helm_manager.h"
#include "ui_helm_manager.h"
#include "ui_helm_manager_config.h"
#include <QStyle>
#include <QPalette>
#include <QTimer>
#include "std_msgs/msg/string.hpp"

HelmManager::HelmManager(QWidget* parent):
  QWidget(parent),
  ui(new Ui::HelmManager)
{
  ui->setupUi(this);

  connect(this, &HelmManager::pilotingModeUpdated, this, &HelmManager::updatePilotingMode, Qt::QueuedConnection);
  connect(this, &HelmManager::robotStatusUpdated, ui->robotStatusTextBrowser, &QTextBrowser::setText, Qt::QueuedConnection);
  connect(this, &HelmManager::requestPilotingMode, this, &HelmManager::sendPilotingModeRequest);
  connect(this, &HelmManager::heartbeatTimesUpdated, this, &HelmManager::updateHeartbeatTimes);

  m_watchdog_timer = new QTimer(this);
  connect(m_watchdog_timer, &QTimer::timeout, this, &HelmManager::watchdogUpdate);
  m_watchdog_timer->start(500);
}

HelmManager::~HelmManager()
{
  delete ui;
}

void HelmManager::setNode(rclcpp::Node::SharedPtr node)
{
  node_ = node;
}

void HelmManager::on_standbyPushButton_clicked(bool checked)
{
  emit requestPilotingMode("standby");
}

void HelmManager::on_autonomousPushButton_clicked(bool checked)
{
  emit requestPilotingMode("autonomous");
}

void HelmManager::on_timeLatencyConfigPushButton_clicked(bool checked)
{
  Ui::HelmManagerConfig configDialogUI;
  QDialog configDialog;
  configDialogUI.setupUi(&configDialog);
  configDialogUI.greenTimeoutSpinBox->setValue(max_green_duration_.seconds());
  configDialogUI.yellowTimeoutSpinBox->setValue(max_yellow_duration_.seconds());
  if(configDialog.exec())
  {
    max_green_duration_ = rclcpp::Duration::from_seconds(configDialogUI.greenTimeoutSpinBox->value());
    max_yellow_duration_ = rclcpp::Duration::from_seconds(configDialogUI.yellowTimeoutSpinBox->value());
  }
}

void HelmManager::updateRobotNamespace(QString robot_namespace)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "updateRobotNamespace: " << robot_namespace.toStdString());

  auto ns = robot_namespace.toStdString();

  if(ns.empty() || ns[0] != '/')
    ns = "/"+ns;

  if(!ns.empty() && ns.back() != '/')
    ns = ns + "/";
  


  heartbeat_subscription_ = node_->create_subscription<project11_msgs::msg::Heartbeat>(ns+"project11/heartbeat", 1, std::bind(&HelmManager::heartbeatCallback, this, std::placeholders::_1));
  send_command_publisher_ = node_->create_publisher<std_msgs::msg::String>(ns+"project11/send_command",1);
}

void HelmManager::sendPilotingModeRequest(QString piloting_mode)
{
  std_msgs::msg::String cmd;
  cmd.data = "piloting_mode "+piloting_mode.toStdString();
  send_command_publisher_->publish(cmd); 
}

void HelmManager::heartbeatCallback(const project11_msgs::msg::Heartbeat& message)
{
  rclcpp::Time last_heartbeat_receive_time = node_->get_clock()->now();
  rclcpp::Time last_heartbeat_timestamp = message.header.stamp;
    
  QString status_string;
  QString piloting_mode;
  for(auto kv: message.values)
  {
    status_string += kv.key.c_str();
    status_string += ": ";
    status_string += kv.value.c_str();
    status_string += "\n";
    if (kv.key == "piloting_mode")
      piloting_mode = kv.value.c_str();
  }

  emit pilotingModeUpdated(piloting_mode);
  emit robotStatusUpdated(status_string);
  emit heartbeatTimesUpdated(last_heartbeat_timestamp.seconds(), last_heartbeat_receive_time.seconds());


}

void HelmManager::updatePilotingMode(QString const &piloting_mode)
{
    if(piloting_mode == "standby")
    {
        QPalette pal = ui->standbyPushButton->palette();
        pal.setColor(QPalette::Button, Qt::green);
        ui->standbyPushButton->setPalette(pal);
        ui->standbyPushButton->setAutoFillBackground(true);
        ui->autonomousPushButton->setPalette(this->style()->standardPalette());
    } 
    else if(piloting_mode == "autonomous")
    {
        QPalette pal = ui->autonomousPushButton->palette();
        pal.setColor(QPalette::Button, Qt::green);
        ui->autonomousPushButton->setPalette(pal);
        ui->autonomousPushButton->setAutoFillBackground(true);
        ui->standbyPushButton->setPalette(this->style()->standardPalette());
    }
    else if(piloting_mode == "manual")
    {
        QPalette pal = ui->standbyPushButton->palette();
        pal.setColor(QPalette::Button, Qt::blue);
        ui->standbyPushButton->setPalette(pal);
        ui->standbyPushButton->setAutoFillBackground(true);
        pal = ui->autonomousPushButton->palette();
        pal.setColor(QPalette::Button, Qt::blue);
        ui->autonomousPushButton->setPalette(pal);
        ui->autonomousPushButton->setAutoFillBackground(true);
    }
    else
    {
        ui->standbyPushButton->setPalette(this->style()->standardPalette());
        ui->autonomousPushButton->setPalette(this->style()->standardPalette());
    }
}

void HelmManager::updateHeartbeatTimes(double last_heartbeat_timestamp, double last_heartbeat_receive_time)
{
  last_heartbeat_timestamp_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type()) + rclcpp::Duration::from_seconds(last_heartbeat_timestamp);
  last_heartbeat_receive_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type()) + rclcpp::Duration::from_seconds(last_heartbeat_receive_time);
}

void HelmManager::watchdogUpdate()
{
  auto now = node_->get_clock()->now();
  if(last_heartbeat_timestamp_.seconds() > 0.0)
  {
    auto diff = now-last_heartbeat_timestamp_;

    QPalette pal = palette();
    if(diff < max_green_duration_)
      pal.setColor(QPalette::Window, Qt::green);
    else if (diff < max_yellow_duration_)
      pal.setColor(QPalette::Window, Qt::yellow);
    else
      pal.setColor(QPalette::Window, Qt::red);
    this->setAutoFillBackground(true);
    this->setPalette(pal);

    auto last_receive_duration = now-last_heartbeat_receive_time_;
    auto latency = last_heartbeat_receive_time_ - last_heartbeat_timestamp_;
    QString msg = "Last HB: " + QString::number(last_receive_duration.seconds()) + "s Latency: " + QString::number(latency.seconds()) +"s";
    ui->timeLatencyLabel->setText(msg);
  }
}
