#include "helm_manager.h"
#include "ui_helm_manager.h"
#include "ui_helm_manager_config.h"
#include <QStyle>
#include <QPalette>
#include <QTimer>
#include "std_msgs/String.h"

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
  configDialogUI.greenTimeoutSpinBox->setValue(max_green_duration_.toSec());
  configDialogUI.yellowTimeoutSpinBox->setValue(max_yellow_duration_.toSec());
  if(configDialog.exec())
  {
    max_green_duration_ = ros::Duration(configDialogUI.greenTimeoutSpinBox->value());
    max_yellow_duration_ = ros::Duration(configDialogUI.yellowTimeoutSpinBox->value());
  }
}

void HelmManager::updateRobotNamespace(QString robot_namespace)
{
  ROS_DEBUG_STREAM("updateRobotNamespace: " << robot_namespace.toStdString());
  ros::NodeHandle nh;
  m_heartbeat_subscriber = nh.subscribe("/"+robot_namespace.toStdString()+"/project11/heartbeat" , 1, &HelmManager::heartbeatCallback, this);
  m_send_command_publisher = nh.advertise<std_msgs::String>("/"+robot_namespace.toStdString()+"/project11/send_command",1);
}

void HelmManager::sendPilotingModeRequest(QString piloting_mode)
{
  std_msgs::String cmd;
  cmd.data = "piloting_mode "+piloting_mode.toStdString();
  m_send_command_publisher.publish(cmd); 
}

void HelmManager::heartbeatCallback(const project11_msgs::Heartbeat::ConstPtr& message)
{
  ros::Time last_heartbeat_receive_time = ros::Time::now();
  ros::Time last_heartbeat_timestamp = message->header.stamp;
    
  QString status_string;
  QString piloting_mode;
  for(auto kv: message->values)
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
  emit heartbeatTimesUpdated(last_heartbeat_timestamp.toSec(), last_heartbeat_receive_time.toSec());


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
  m_last_heartbeat_timestamp.fromSec(last_heartbeat_timestamp);
  m_last_heartbeat_receive_time.fromSec(last_heartbeat_receive_time);

}

void HelmManager::watchdogUpdate()
{
  ros::Time now = ros::Time::now();
  ros::Duration diff = now-m_last_heartbeat_timestamp;

  QPalette pal = palette();
  if(diff < max_green_duration_)
    pal.setColor(QPalette::Background, Qt::green);
  else if (diff < max_yellow_duration_)
    pal.setColor(QPalette::Background, Qt::yellow);
  else
    pal.setColor(QPalette::Background, Qt::red);
  this->setAutoFillBackground(true);
  this->setPalette(pal);

  if(m_last_heartbeat_timestamp.isValid())
  {
    ros::Duration last_receive_duration = now-m_last_heartbeat_receive_time;
    ros::Duration latency = m_last_heartbeat_receive_time - m_last_heartbeat_timestamp;
    QString msg = "Last HB: " + QString::number(last_receive_duration.toSec()) + "s Latency: " + QString::number(latency.toSec()) +"s";
    ui->timeLatencyLabel->setText(msg);
  }
}
