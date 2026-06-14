#ifndef CAMP_AIS_MANAGER_H
#define CAMP_AIS_MANAGER_H

#include "ros/ros_object.h"
#include "marine_ais_msgs/msg/ais_contact.hpp"
#include "ais_contact.h"

class QGraphicsItem;

// [#59 PR5] Discovers AIS topics and renders contacts. No longer a window: it is
// a non-widget ROS object whose contacts parent to a Map layer ("AIS") shown in
// the Layers tab — visibility via the layer's checkbox, not a separate window.
class AISManager: public camp_ros::ROSObject
{
  Q_OBJECT

public:
  explicit AISManager(QObject *parent = nullptr);

  void onNodeUpdated() override;

  // [#59 PR5] The "AIS" Map layer (in topLevelLayers) that contacts parent to,
  // so they render independent of any chart and toggle with the layer's
  // checkbox. Set once by MainWindow after the project/map exists.
  void setAnchor(QGraphicsItem* anchor) { m_anchor = anchor; }

signals:
  void newAisReport(AISReport *report);

public slots:
  void updateBackground();
  void updateViewport(QPointF ll, QPointF ur);

private slots:
  void scanForSources();
  void addAisReport(AISReport *report);

private:
  void aisContactCallback(const marine_ais_msgs::msg::AISContact& message);

  std::map<std::string, rclcpp::Subscription<marine_ais_msgs::msg::AISContact>::SharedPtr > m_sources;
  QTimer* m_scan_timer;
  QTimer* m_update_timer;
  std::map<uint32_t, AISContact*> m_contacts;

  QGraphicsItem* m_anchor = nullptr;
};

#endif
