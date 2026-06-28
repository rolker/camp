#ifndef CAMP_RUNNING_TASKS_MODEL_H
#define CAMP_RUNNING_TASKS_MODEL_H

#include <QAbstractItemModel>
#include <QString>
#include <QVector>
#include <memory>
#include <vector>

/// A single task as displayed in the running-tasks tree. Populated from
/// marine_nav_interfaces/TaskInformation on the GUI thread — the ROS message is
/// converted to this Qt-friendly struct off the executor thread so the model
/// never touches live ROS types.
struct TaskRow
{
  QString id;       ///< full slash-delimited id, e.g. "survey_a/line_1"
  QString type;     ///< task type, e.g. "survey_line"
  int priority = 0;
  QString status;   ///< status summary (first line of the status YAML)
  bool done = false;
};

/// Tree model over a flat TaskInformation list, keyed on the hierarchical id.
/// "survey_a/line_1" becomes a child of "survey_a"; intermediate path segments
/// with no explicit task get a synthetic group row. The task whose id equals the
/// current navigation task is rendered bold + highlighted.
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

  /// Replace the whole tree from a fresh TaskFeedback snapshot.
  void setTasks(const QString& current_task, const QVector<TaskRow>& rows);

  /// Full task id for an index (empty if invalid).
  QString idForIndex(const QModelIndex& index) const;
  /// Index of the row carrying the given full id (invalid if not present).
  QModelIndex indexForId(const QString& id) const;

private:
  struct Node
  {
    QString segment;          ///< this level's path segment (display name)
    QString fullId;           ///< full path id ("" for the hidden root)
    TaskRow row;              ///< task data; meaningful only when hasTask
    bool hasTask = false;     ///< false for a synthesized intermediate group
    Node* parent = nullptr;
    int rowInParent = 0;
    std::vector<std::unique_ptr<Node>> children;
  };

  Node* nodeForIndex(const QModelIndex& index) const;
  Node* findById(Node* node, const QString& id) const;
  void rebuild(const QVector<TaskRow>& rows);

  std::unique_ptr<Node> root_;
  QString current_task_;
};

#endif  // CAMP_RUNNING_TASKS_MODEL_H
