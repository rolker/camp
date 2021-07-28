#include "platform_manager.h"
#include "ui_platform_manager.h"
#include "platform.h"
#include "backgroundraster.h"

PlatformManager::PlatformManager(QWidget* parent):
  QWidget(parent),
  m_ui(new Ui::PlatformManager)
{
  m_ui->setupUi(this);
  
  //m_platforms["ben"] = new Platform;
  //m_ui->tabWidget->addTab(m_platforms["ben"], "ben");

  ros::NodeHandle nh;
  m_platform_list_sub = nh.subscribe("/project11/platforms", 5, &PlatformManager::platformListCallback, this);
}

PlatformManager::~PlatformManager()
{
  delete m_ui;
}

void PlatformManager::platformListCallback(const project11_msgs::PlatformList::ConstPtr &message)
{
  for(auto platform: message->platforms)
  {
    QMetaObject::invokeMethod(this, "updatePlatform", Qt::QueuedConnection, Q_ARG(project11_msgs::Platform, platform));
  }
}

void PlatformManager::updatePlatform(project11_msgs::Platform platform)
{
  if(m_platforms.find(platform.name) == m_platforms.end())
  {
    m_platforms[platform.name] = new Platform(this, m_background);
    m_ui->tabWidget->addTab(m_platforms[platform.name], platform.name.c_str());
  }
  m_platforms[platform.name]->update(platform);
}

void PlatformManager::updateBackground(BackgroundRaster * bg)
{
  m_background = bg;
  for(auto p: m_platforms)
  {
    p.second->setParentItem(bg);
    p.second->updateProjectedPoints();
  }
}

void PlatformManager::on_tabWidget_currentChanged(int index)
{
  if(index == -1)
    emit currentPlatform(nullptr);
  else
    emit currentPlatform(qobject_cast<Platform*>(m_ui->tabWidget->currentWidget()));
}
