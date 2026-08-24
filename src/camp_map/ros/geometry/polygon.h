#ifndef CAMP_ROS_GEOMETRY_POLYGON_H
#define CAMP_ROS_GEOMETRY_POLYGON_H

#include "../layer.h"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include <QtConcurrent>

#include <atomic>

namespace camp
{
namespace ros
{
namespace geometry
{

struct PolygonData
{
  QPolygonF polygon;
  QPointF position;
};

class Polygon: public Layer
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  Polygon(MapItem* parent, Node* node, QString topic);

  /// [camp#213] Joins the in-flight render worker before teardown. Without this,
  /// removing the layer while processPolygon() runs lets a worker bound to `this`
  /// write into a destroyed object (SIGSEGV) — the same defect fixed for
  /// OccupancyGrid in camp#209.
  ~Polygon() override;

signals:
  void newPolygonData(PolygonData data);

private slots:
  void polygonCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr data);
  void processPolygon(const geometry_msgs::msg::PolygonStamped::SharedPtr data);
  void updatePolygon(const PolygonData &data);

private:
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr subscription_;

  QFuture<void> process_future_;

  /// [camp#213] Set before teardown so a callback already dispatched when the
  /// destructor runs cannot start a fresh render. subscription_.reset() alone is
  /// not enough: rclcpp's executor holds its own strong reference across
  /// dispatch, so reset() neither cancels nor joins an in-flight callback.
  std::atomic<bool> shutdown_{false};

};

} // namespace geometry
} // namespace ros
} // namespace camp

#endif
