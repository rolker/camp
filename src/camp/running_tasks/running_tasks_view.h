#ifndef CAMP_RUNNING_TASKS_VIEW_H
#define CAMP_RUNNING_TASKS_VIEW_H

#include <QWidget>
#include <QString>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "marine_nav_interfaces/msg/task_feedback.hpp"
#include "marine_nav_interfaces/msg/task_information.hpp"
#include "running_tasks/running_tasks_model.h"

class QTimer;
class QTreeView;

/// Read-only structured running-task view. Node-injection widget (the
/// helm_manager idiom, not camp_ros::ROSWidget): hand it a node with setNode()
/// and a robot namespace with updateRobotNamespace(); it subscribes to
/// marine/status/mission_tasks and renders the structured TaskFeedback as a
/// tree, highlighting the currently-executing task. Designed to work both
/// embedded in CAMP and (later) hosted in an rqt plugin.
class RunningTasksView : public QWidget
{
  Q_OBJECT

public:
  explicit RunningTasksView(QWidget* parent = nullptr);
  ~RunningTasksView() override;

  void setNode(rclcpp::Node::SharedPtr node);

signals:
  /// User selected a task row (full id). Map glue (P2) consumes this.
  /// Only emitted for rows where hasTaskPoses() is true.
  void taskSelected(QString id);
  /// Emitted on the GUI thread after every model update with the current
  /// navigation task id and the full raw task list. The RunningTasksOverlay
  /// consumes this to rebuild overlay geometry without a second ROS subscription.
  void tasksUpdated(QString current_task,
                    std::vector<marine_nav_interfaces::msg::TaskInformation> tasks);

public slots:
  void updateRobotNamespace(QString robot_namespace);
  /// Select the row with the given full id (e.g. from a map click in P2).
  void setSelectedTask(QString id);

private slots:
  void applyPendingTasks();
  void onCurrentRowChanged(const QModelIndex& current,
                           const QModelIndex& previous);
  /// Periodic staleness check — colors the background green/yellow/red by the
  /// age of the last received TaskFeedback (mirrors HelmManager's watchdog).
  void watchdogUpdate();

private:
  void subscribe();
  void taskFeedbackCallback(const marine_nav_interfaces::msg::TaskFeedback& msg);

  QTreeView* tree_;
  RunningTasksModel* model_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<marine_nav_interfaces::msg::TaskFeedback>::SharedPtr
      subscription_;
  QString robot_namespace_;

  std::mutex pending_mutex_;
  QString pending_current_;
  std::vector<marine_nav_interfaces::msg::TaskInformation> pending_tasks_;

  // Message-staleness watchdog (green < max_green_ < yellow < max_yellow_ < red).
  QTimer* watchdog_timer_;
  rclcpp::Time last_message_time_;
  bool has_message_ = false;
  rclcpp::Duration max_green_duration_;
  rclcpp::Duration max_yellow_duration_;
};

#endif  // CAMP_RUNNING_TASKS_VIEW_H
