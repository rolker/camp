#ifndef CAMP_FOOTPRINT_MANAGER_H
#define CAMP_FOOTPRINT_MANAGER_H

#include <map>
#include <string>

#include "ros/ros_object.h"

class QTimer;
class QGraphicsItem;
class Footprint;

/// [#64] Discovers boat-footprint topics (nav2 `.../published_footprint`,
/// geometry_msgs/PolygonStamped) and renders each under the "Boat Footprint"
/// Map layer in the Layers tab. Non-widget ROS object; visibility is the layer's
/// checkbox. Mirrors CollisionMonitorManager.
class FootprintManager: public camp_ros::ROSObject
{
  Q_OBJECT
public:
  explicit FootprintManager(QObject* parent = nullptr);

  /// The "Boat Footprint" Map layer (in topLevelLayers) that footprints parent
  /// to. Set once by MainWindow after the project/map exists.
  void setAnchor(QGraphicsItem* anchor) { m_anchor = anchor; }

public slots:
  void updateBackground();

private slots:
  void scanForSources();

private:
  std::map<std::string, Footprint*> footprints_;
  QTimer* scan_timer_ = nullptr;
  QGraphicsItem* m_anchor = nullptr;
};

#endif
