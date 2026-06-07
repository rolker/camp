#ifndef CAMP_PLATFORM_MANAGER_H
#define CAMP_PLATFORM_MANAGER_H

#include "ros/ros_widget.h"
#include <QGeoCoordinate>
#include "marine_interfaces/msg/platform_list.hpp"

Q_DECLARE_METATYPE(marine_interfaces::msg::Platform);

namespace Ui
{
  class PlatformManager;
}

class Platform;
class QGraphicsItem;

class PlatformManager: public camp_ros::ROSWidget
{
  Q_OBJECT
public:
  explicit PlatformManager(QWidget *parent=0);
  ~PlatformManager();

  void onNodeUpdated() override;

  // [#59 PR6] Persistent scene-origin anchor (AVP::originAnchor()) that platform
  // overlays parent to, so they render independent of any loaded chart.
  void setAnchor(QGraphicsItem* anchor) { m_anchor = anchor; }

signals:
  void currentPlatform(Platform* platform);
  void currentPlatformPosition(QGeoCoordinate position);

public slots:
  void updateBackground();
  void platformPosition(Platform * platform, QGeoCoordinate position);
  
private slots:
  void updatePlatform(marine_interfaces::msg::Platform platform);
  void on_tabWidget_currentChanged(int index);

private:
  void platformListCallback(const marine_interfaces::msg::PlatformList &message);

  Ui::PlatformManager* m_ui;
  rclcpp::Subscription<marine_interfaces::msg::PlatformList>::SharedPtr platform_list_subscription_;

  std::map<std::string, Platform*> m_platforms;

  QGraphicsItem* m_anchor = nullptr;
};

#endif
