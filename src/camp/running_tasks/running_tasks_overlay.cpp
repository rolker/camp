#include "running_tasks/running_tasks_overlay.h"

#include <QSignalBlocker>

#include "running_tasks/running_tasks_view.h"

RunningTasksOverlay::RunningTasksOverlay(QGraphicsScene* scene,
                                         RunningTasksView* view,
                                         QWidget* parent)
  : camp_ros::ROSWidget(parent), scene_(scene), view_(view)
{
  connect(view_, &RunningTasksView::tasksUpdated,
          this, &RunningTasksOverlay::onTasksUpdated);
  connect(view_, &RunningTasksView::taskSelected,
          this, &RunningTasksOverlay::onTaskSelected);
}

RunningTasksOverlay::~RunningTasksOverlay()
{
  clearItems();
}

void RunningTasksOverlay::onTasksUpdated(
    const QString& current_task,
    const std::vector<marine_nav_interfaces::msg::TaskInformation>& tasks)
{
  rebuildItems(current_task, tasks);
}

void RunningTasksOverlay::onTaskSelected(const QString& id)
{
  if (id == selected_id_)
    return;  // idempotent — guard against map→tree→map round-trips

  selected_id_ = id;
  for (auto it = items_.begin(); it != items_.end(); ++it)
    it.value()->setSelected(it.key() == id);
}

void RunningTasksOverlay::onItemClicked(const QString& id)
{
  // Guard the selection-sync loop so a map click doesn't trigger
  // taskSelected → onTaskSelected → setSelected → redraw → re-click.
  // Mirror the QSignalBlocker pattern in RunningTasksView::applyPendingTasks.
  if (id == selected_id_)
    return;
  view_->setSelectedTask(id);
}

void RunningTasksOverlay::rebuildItems(
    const QString& current_task,
    const std::vector<marine_nav_interfaces::msg::TaskInformation>& tasks)
{
  if (!scene_ || !transform_buffer_)
    return;

  clearItems();

  for (const auto& task : tasks)
  {
    if (task.poses.empty())
      continue;

    // TaskInformation.poses is geometry_msgs/PoseStamped[] with per-pose headers.
    // getGeoCoordinate transforms each to earth frame with stale-stamp retry,
    // so the source frame (commonly map or earth) is handled transparently.
    QList<QGeoCoordinate> geo_poses;
    for (const auto& ps : task.poses)
    {
      QGeoCoordinate gc = getGeoCoordinate(ps.pose, ps.header);
      if (gc.isValid())
        geo_poses.append(gc);
    }
    if (geo_poses.isEmpty())
      continue;

    const QString id = QString::fromStdString(task.id);
    const bool is_current = !current_task.isEmpty() &&
        id.split('/', Qt::SkipEmptyParts).join('/') ==
        current_task.split('/', Qt::SkipEmptyParts).join('/');

    auto* item = new TaskOverlayItem(id, geo_poses, is_current, task.done);
    item->setSelected(id == selected_id_);

    connect(item, &TaskOverlayItem::clicked,
            this, &RunningTasksOverlay::onItemClicked);

    scene_->addItem(item);
    items_[id] = item;
  }
}

void RunningTasksOverlay::clearItems()
{
  for (auto* item : items_)
  {
    if (scene_)
      scene_->removeItem(item);
    delete item;
  }
  items_.clear();
}
