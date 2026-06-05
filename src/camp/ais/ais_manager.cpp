#include "ais_manager.h"
#include "ui_ais_manager.h"
#include <QTimer>
#include "backgroundraster.h"
#include "ros/ros_context.h"

AISManager::AISManager(QWidget* parent):
  camp_ros::ROSWidget(parent),
  m_ui(new Ui::AISManager)
{
  m_ui->setupUi(this);

  connect(this, &AISManager::newAisReport, this, &AISManager::addAisReport, Qt::QueuedConnection);

  m_scan_timer = new QTimer(this);
  connect(m_scan_timer, &QTimer::timeout, this, &AISManager::scanForSources);
  m_scan_timer->start(1000);

  m_update_timer = new QTimer(this);
  m_update_timer->start(200);
}

AISManager::~AISManager()
{
  delete m_ui;
}

void AISManager::scanForSources()
{
  if(node_)
  {
    auto topics = node_->get_topic_names_and_types();
    for(auto topic: topics)
    {
      auto name = topic.first;
      if(m_sources.find(name) == m_sources.end())
      {
        for(auto topic_type: topic.second)
        {
          if (topic_type == "marine_ais_msgs/msg/AISContact")
          {
            RCLCPP_INFO_STREAM(node_->get_logger(), "Subscribing to ais topic: " << name);
            rclcpp::SubscriptionOptions sub_options;
            if (auto ctx = camp_ros::RosContext::instance())
              sub_options.callback_group = ctx->nextDedicatedGroup();
            m_sources[name] = node_->create_subscription<marine_ais_msgs::msg::AISContact>(name, 10, std::bind(&AISManager::aisContactCallback, this, std::placeholders::_1), sub_options);
            m_ui->sourcesListWidget->addItem(name.c_str());
            break;
          }
        }
      }
    }
  }

}

void AISManager::aisContactCallback(const marine_ais_msgs::msg::AISContact& message)
{
  if(isnan(message.pose.position.latitude) || isnan(message.pose.position.longitude))
    return;

  AISReport* r = new AISReport(message);
    emit newAisReport(r);
    r->deleteLater();
}


void AISManager::addAisReport(AISReport* report)
{
  if(m_contacts.find(report->mmsi) == m_contacts.end())
  {
    m_contacts[report->mmsi] = new AISContact(report, this, m_background);
    m_contacts[report->mmsi]->nodeStarted(node_, transform_buffer_);
    connect(m_update_timer, &QTimer::timeout, m_contacts[report->mmsi], &AISContact::updateView);
    m_ui->contactListWidget->addItem(QString::number(report->mmsi));
  }
  m_contacts[report->mmsi]->newReport(report);
}

void AISManager::onNodeUpdated()
{
  for(auto &contact: m_contacts)
    if(contact.second)
      contact.second->nodeStarted(node_, transform_buffer_);
}

void AISManager::updateBackground(BackgroundRaster * bg)
{
  m_background = bg;
  for(auto c: m_contacts)
  {
    c.second->setParentItem(bg);
    c.second->updateProjectedPoints();
  }

}

void AISManager::updateViewport(QPointF ll, QPointF ur)
{
  //ROS_INFO_STREAM( "viewport: " << ll.x() << ", " << ll.y() << " - " << ur.x() << ", " << ur.y());
}
