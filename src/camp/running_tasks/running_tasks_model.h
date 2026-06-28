#ifndef CAMP_RUNNING_TASKS_MODEL_H
#define CAMP_RUNNING_TASKS_MODEL_H

#include <memory>
#include <vector>

#include <QAbstractItemModel>
#include <QString>

#include "marine_nav_interfaces/msg/task_information.hpp"
#include "marine_nav_tasks/task.h"
#include "marine_nav_tasks/task_list.h"
#include "rclcpp/rclcpp.hpp"

/// Tree model over the boat's running task list. The hierarchy (slash-delimited
/// ids), message ordering, add/remove, done handling and YAML parsing all come
/// from marine_nav_tasks::TaskList / Task — the same model the boat uses — so the
/// operator view stays consistent with the autonomy side. This class is a thin
/// QAbstractItemModel adapter: it snapshots that domain tree into display nodes,
/// floats done tasks to the bottom of each level, and highlights the current
/// navigation task. Each display node keeps the marine_nav_tasks::TaskPtr it
/// mirrors, so later phases (P2 map linkage) can read task poses/markers from it.
class RunningTasksModel : public QAbstractItemModel
{
  Q_OBJECT
public:
  enum Column { Name = 0, Type, Priority, Status, Done, ColumnCount };

  explicit RunningTasksModel(QObject* parent = nullptr);
  ~RunningTasksModel() override;

  // QAbstractItemModel
  QModelIndex index(int row, int column,
                    const QModelIndex& parent = QModelIndex()) const override;
  QModelIndex parent(const QModelIndex& child) const override;
  int rowCount(const QModelIndex& parent = QModelIndex()) const override;
  int columnCount(const QModelIndex& parent = QModelIndex()) const override;
  QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
  QVariant headerData(int section, Qt::Orientation orientation,
                      int role = Qt::DisplayRole) const override;

  /// Rebuild from a fresh TaskFeedback snapshot: the current navigation task id
  /// plus the task list. `clock` is used by TaskList to stamp newly created tasks.
  void setTasks(
      const QString& current_task,
      const std::vector<marine_nav_interfaces::msg::TaskInformation>& tasks,
      rclcpp::Clock::SharedPtr clock);

  /// Full task id for an index (empty if invalid).
  QString idForIndex(const QModelIndex& index) const;
  /// Index of the row carrying the given full id (invalid if not present).
  QModelIndex indexForId(const QString& id) const;

private:
  struct Node
  {
    marine_nav_tasks::TaskPtr task;  ///< domain task this row mirrors
    QString segment;                 ///< last path component (display name)
    QString fullId;                  ///< full task id
    Node* parent = nullptr;
    int rowInParent = 0;
    std::vector<std::unique_ptr<Node>> children;
  };

  Node* nodeForIndex(const QModelIndex& index) const;
  Node* findById(Node* node, const QString& id) const;
  void buildNodes(const marine_nav_tasks::TaskList& task_list, Node* parent_node);
  void sortChildrenRecursive(Node* node);

  marine_nav_tasks::TaskList task_list_;
  std::unique_ptr<Node> root_;
  QString current_task_;
};

#endif  // CAMP_RUNNING_TASKS_MODEL_H
