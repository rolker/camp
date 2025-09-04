#include "platform_manager.h"
#include "ui_platform_manager.h"
#include "platform.h"
#include "backgroundraster.h"

PlatformManager::PlatformManager(QWidget* parent):
  camp_ros::ROSWidget(parent),
  m_ui(new Ui::PlatformManager)
{
  m_ui->setupUi(this);
  
}


PlatformManager::~PlatformManager()
{
  delete m_ui;
}

void PlatformManager::onNodeUpdated()
{
  platform_list_subscription_ = node_->create_subscription<project11_msgs::msg::PlatformList>("/project11/platforms", 5, std::bind(&PlatformManager::platformListCallback, this, std::placeholders::_1));
}

void PlatformManager::platformListCallback(const project11_msgs::msg::PlatformList &message)
{
  for(const auto& platform: message.platforms)
  {
    QMetaObject::invokeMethod(this, "updatePlatform", Qt::QueuedConnection, Q_ARG(project11_msgs::msg::Platform, platform));
  }
}

void PlatformManager::updatePlatform(project11_msgs::msg::Platform platform)
{
  if(m_platforms.find(platform.name) == m_platforms.end())
  {
    m_platforms[platform.name] = new Platform(this, m_background);
    m_platforms[platform.name]->nodeStarted(node_, transform_buffer_);
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
  