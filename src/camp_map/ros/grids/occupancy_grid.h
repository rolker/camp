#ifndef CAMP_ROS_GRIDS_OCCUPANCY_GRID_H
#define CAMP_ROS_GRIDS_OCCUPANCY_GRID_H

#include "../layer.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <QtConcurrent>

#include <atomic>

namespace camp
{
namespace ros
{
namespace grids
{

struct OccupancyGridData
{
  QImage grid_image;
  QPointF origin;
  float meters_per_pixel = 1.0;
};

class OccupancyGrid: public Layer
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  OccupancyGrid(MapItem* parent, Node* node, QString topic);

  /// [camp#208] Visibility as seen from the ROS callback thread.
  ///
  /// QGraphicsItem::isVisible() is GUI-thread-owned state, and
  /// occupancyGridCallback runs on the ROS executor thread — reading it there is
  /// a data race. itemChange() (GUI thread) mirrors it into this atomic, and the
  /// callback reads the atomic instead.
  std::atomic<bool> visible_{false};

  /// [camp#209] Joins the in-flight render worker before teardown. Without this,
  /// removing the layer while processOccupancyGrid() is running lets the worker
  /// write into a destroyed object (SIGSEGV). GridMap and RasterLayer already do
  /// this; OccupancyGrid was the one that did not.
  ~OccupancyGrid() override;

protected:
  QVariant itemChange(GraphicsItemChange change, const QVariant& value) override;

signals:
  void occupancyGridUpdated(const OccupancyGridData &data);

private slots:
  void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid &grid);
  void processOccupancyGrid(const nav_msgs::msg::OccupancyGrid &grid);

  void updateOccupancyGrid(const OccupancyGridData &data);

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;

  QFuture<void> process_future_;
};

}  // namespace grids
}  // namespace ros
}  // namespace camp

Q_DECLARE_METATYPE(nav_msgs::msg::OccupancyGrid)
Q_DECLARE_METATYPE(camp::ros::grids::OccupancyGridData)

#endif  // CAMP_ROS_GRIDS_OCCUPANCY_GRID_H
