#include "ais_manager.h"
#include "ui_ais_manager.h"
#include <QTimer>
#include "backgroundraster.h"

AISManager::AISManager(QWidget* parent):
  QWidget(parent),
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
  ros::NodeHandle nh;

  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  for(const auto t: topic_info)
    if (t.datatype == "marine_msgs/Contact")
      if (m_sources.find(t.name) == m_sources.end())
      {
        m_sources[t.name] = nh.subscribe(t.name, 10, &AISManager::contactCallback, this);
        m_ui->sourcesListWidget->addItem(t.name.c_str());
      }

  
}

void AISManager::contactCallback(const marine_msgs::Contact::ConstPtr& message)
{
    if(message->position.latitude > 90 || message->position.longitude > 180)
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
    connect(m_update_timer, &QTimer::timeout, m_contacts[report->mmsi], &AISContact::updateView);
    m_ui->contactListWidget->addItem(QString::number(report->mmsi));
  }
  m_contacts[report->mmsi]->newReport(report);
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
