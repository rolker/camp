#ifndef CAMP_RUNNING_TASKS_VIEW_H
#define CAMP_RUNNING_TASKS_VIEW_H

#include <QWidget>
#include <QString>
#include <QVector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "marine_nav_interfaces/msg/task_feedback.hpp"
#include "running_tasks/running_tasks_model.h"

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
  void taskSelected(QString id);

public slots:
  void updateRobotNamespace(QString robot_namespace);
  /// Select the row with the given full id (e.g. from a map click in P2).
  void setSelectedTask(QString id);

private slots:
  void applyPendingTasks();
  void onCurrentRowChanged(const QModelIndex& current,
                           const QModelIndex& previous);

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
  QVector<TaskRow> pending_rows_;
};

#endif  // CAMP_RUNNING_TASKS_VIEW_H
