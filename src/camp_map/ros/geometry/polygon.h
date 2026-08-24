#ifndef CAMP_ROS_GEOMETRY_POLYGON_H
#define CAMP_ROS_GEOMETRY_POLYGON_H

#include "../layer.h"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include <QtConcurrent>
#include <QMutex>

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

  /// [camp#213] Guards process_future_ and shutdown_, which are both touched
  /// from the ROS executor thread (polygonCallback) and the GUI thread
  /// (~Polygon). Without it the future is read and assigned concurrently, which
  /// is a data race, and the gate below can be passed by a callback that then
  /// launches a worker the destructor has already joined past. Mirrors the
  /// handshake in GridMap (grids/grid_map.h).
  QMutex mutex_;

  QFuture<void> process_future_;

  /// [camp#213] Set before teardown so a callback already dispatched when the
  /// destructor runs cannot start a fresh render. subscription_.reset() alone is
  /// not enough: rclcpp's executor holds its own strong reference across
  /// dispatch, so reset() neither cancels nor joins an in-flight callback.
  /// Guarded by mutex_.
  bool shutdown_ = false;

};

} // namespace geometry
} // namespace ros
} // namespace camp

#endif
