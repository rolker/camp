#ifndef CAMP_AIS_MANAGER_H
#define CAMP_AIS_MANAGER_H

#include "ros/ros_widget.h"
#include "marine_interfaces/msg/contact.hpp"
#include "marine_ais_msgs/msg/ais_contact.hpp"
#include "ais_contact.h"

namespace Ui
{
class AISManager;
}

class BackgroundRaster;

class AISManager: public camp_ros::ROSWidget
{
  Q_OBJECT

public:
  explicit AISManager(QWidget *parent =0);
  ~AISManager();

  void onNodeUpdated() override;

signals:
  void newAisReport(AISReport *report);

public slots:
  void updateBackground(BackgroundRaster * bg);
  void updateViewport(QPointF ll, QPointF ur);

private slots:
  void scanForSources();
  void addAisReport(AISReport *report);

private:
  void contactCallback(const marine_interfaces::msg::Contact& message);
  void aisContactCallback(const marine_ais_msgs::msg::AISContact& message);

  Ui::AISManager* m_ui;
  std::map<std::string, rclcpp::Subscription<marine_ais_msgs::msg::AISContact>::SharedPtr > m_sources;
  QTimer* m_scan_timer;
  QTimer* m_update_timer;
  std::map<uint32_t, AISContact*> m_contacts;

  BackgroundRaster* m_background = nullptr;
};

#endif