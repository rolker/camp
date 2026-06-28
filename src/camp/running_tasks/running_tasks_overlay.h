#ifndef CAMP_RUNNING_TASKS_OVERLAY_H
#define CAMP_RUNNING_TASKS_OVERLAY_H

#include <vector>

#include <QGraphicsScene>
#include <QMap>
#include <QString>

#include "marine_nav_interfaces/msg/task_information.hpp"
#include "ros/ros_widget.h"
#include "running_tasks/task_overlay_item.h"

class RunningTasksView;

/// CAMP-only glue that owns the running-task QGraphicsScene overlay.
///
/// This is the camp_ros::ROSWidget side of the P2 feature:
///   - getGeoCoordinate (from ROSWidget) converts each PoseStamped to earth.
///   - tasksUpdated from RunningTasksView triggers a rebuild-all of overlay items.
///   - taskSelected from RunningTasksView highlights the matching item.
///   - TaskOverlayItem::clicked feeds back into RunningTasksView::setSelectedTask.
///
/// Rebuild-all strategy: every tasksUpdated call removes all existing items and
/// recreates them. This is correct for the periodic ~1 Hz republish cadence; the
/// simplicity outweighs incremental-diff complexity at this update rate.
class RunningTasksOverlay : public camp_ros::ROSWidget
{
  Q_OBJECT

public:
  /// \param scene  The QGraphicsScene to add overlay items to (from
  ///               Platform::scene() — Platform is a GeoGraphicsItem).
  /// \param view  The RunningTasksView whose signals we consume.
  /// \param parent  QWidget parent (typically Platform's widget parent).
  explicit RunningTasksOverlay(QGraphicsScene* scene, RunningTasksView* view,
                               QWidget* parent = nullptr);
  ~RunningTasksOverlay() override;

private slots:
  void onTasksUpdated(
      const QString& current_task,
      const std::vector<marine_nav_interfaces::msg::TaskInformation>& tasks);
  void onTaskSelected(const QString& id);
  void onItemClicked(const QString& id);

private:
  void rebuildItems(
      const QString& current_task,
      const std::vector<marine_nav_interfaces::msg::TaskInformation>& tasks);
  void clearItems();

  QGraphicsScene* scene_;
  RunningTasksView* view_;
  QMap<QString, TaskOverlayItem*> items_;
  QString selected_id_;
};

#endif  // CAMP_RUNNING_TASKS_OVERLAY_H
