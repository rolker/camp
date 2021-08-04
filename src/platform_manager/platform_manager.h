#ifndef CAMP_PLATFORM_MANAGER_H
#define CAMP_PLATFORM_MANAGER_H

#include <QWidget>
#include "ros/ros.h"
#include "project11_msgs/PlatformList.h"

Q_DECLARE_METATYPE(project11_msgs::Platform);

namespace Ui
{
  class PlatformManager;
}

class Platform;
class BackgroundRaster;

class PlatformManager: public QWidget
{
  Q_OBJECT
public:
  explicit PlatformManager(QWidget *parent=0);
  ~PlatformManager();

signals:
  void currentPlatform(Platform* platform);

public slots:
  void updateBackground(BackgroundRaster * bg);
  void loadFromParameters();
  
private slots:
  void updatePlatform(project11_msgs::Platform platform);
  void on_tabWidget_currentChanged(int index);

private:
  void platformListCallback(const project11_msgs::PlatformList::ConstPtr &message);


  Ui::PlatformManager* m_ui;
  ros::Subscriber m_platform_list_sub;

  std::map<std::string, Platform*> m_platforms;

  BackgroundRaster* m_background = nullptr;
};

#endif
