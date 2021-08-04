#include "platform_manager.h"
#include "ui_platform_manager.h"
#include "platform.h"
#include "backgroundraster.h"

PlatformManager::PlatformManager(QWidget* parent):
  QWidget(parent),
  m_ui(new Ui::PlatformManager)
{
  m_ui->setupUi(this);
  ros::NodeHandle nh;
  m_platform_list_sub = nh.subscribe("/project11/platforms", 5, &PlatformManager::platformListCallback, this);
  
}

void PlatformManager::loadFromParameters()
{
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue platforms_dict;
  
  if(nh.getParam("project11/platforms", platforms_dict))
  {
    if(platforms_dict.getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
      for(auto platform:platforms_dict)
      {
        m_platforms[platform.first] = new Platform(this, m_background);
        m_ui->tabWidget->addTab(m_platforms[platform.first], platform.first.c_str());
        m_platforms[platform.first]->update(platform);
        connect(m_platforms[platform.first], &Platform::platformPosition, this, &PlatformManager::platformPosition);
      }
    }
  }
  if(m_ui->tabWidget->count()>0)
    on_tabWidget_currentChanged(0);

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

void PlatformManager::platformPosition(Platform * platform, QGeoCoordinate position)
{
  if(m_ui->tabWidget->currentWidget() == platform)
    emit currentPlatformPosition(position);
}
  