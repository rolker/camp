#include "running_tasks/running_tasks_model.h"

#include <QBrush>
#include <QColor>
#include <QFont>

RunningTasksModel::RunningTasksModel(QObject* parent)
  : QAbstractItemModel(parent), root_(std::make_unique<Node>())
{
}

RunningTasksModel::~RunningTasksModel() = default;

RunningTasksModel::Node* RunningTasksModel::nodeForIndex(
    const QModelIndex& index) const
{
  if (index.isValid())
    return static_cast<Node*>(index.internalPointer());
  return root_.get();
}

QModelIndex RunningTasksModel::index(int row, int column,
                                     const QModelIndex& parent) const
{
  if (!hasIndex(row, column, parent))
    return QModelIndex();
  Node* p = nodeForIndex(parent);
  if (!p || row < 0 || row >= static_cast<int>(p->children.size()))
    return QModelIndex();
  return createIndex(row, column, p->children[row].get());
}

QModelIndex RunningTasksModel::parent(const QModelIndex& child) const
{
  if (!child.isValid())
    return QModelIndex();
  Node* n = nodeForIndex(child);
  if (!n || !n->parent || n->parent == root_.get())
    return QModelIndex();
  return createIndex(n->parent->rowInParent, 0, n->parent);
}

int RunningTasksModel::rowCount(const QModelIndex& parent) const
{
  if (parent.column() > 0)
    return 0;
  Node* p = nodeForIndex(parent);
  return p ? static_cast<int>(p->children.size()) : 0;
}

int RunningTasksModel::columnCount(const QModelIndex& /*parent*/) const
{
  return ColumnCount;
}

QVariant RunningTasksModel::data(const QModelIndex& index, int role) const
{
  if (!index.isValid())
    return QVariant();
  Node* n = nodeForIndex(index);
  if (!n)
    return QVariant();

  const bool is_current = n->hasTask && !current_task_.isEmpty() &&
                          n->fullId == current_task_;

  switch (role)
  {
    case Qt::DisplayRole:
      switch (index.column())
      {
        case Name:
          return n->segment;
        case Type:
          return n->hasTask ? n->row.type : QVariant();
        case Priority:
          return n->hasTask ? QVariant(n->row.priority) : QVariant();
        case Status:
          return n->hasTask ? n->row.status : QVariant();
        case Done:
          return n->hasTask
                     ? QVariant(n->row.done ? QStringLiteral("done")
                                            : QString())
                     : QVariant();
        default:
          return QVariant();
      }
    case Qt::FontRole:
      if (is_current)
      {
        QFont font;
        font.setBold(true);
        return font;
      }
      return QVariant();
    case Qt::BackgroundRole:
      if (is_current)
        return QBrush(QColor(0xDD, 0xEE, 0xFF));
      return QVariant();
    case Qt::ToolTipRole:
      return n->hasTask ? n->fullId : QVariant();
    default:
      return QVariant();
  }
}

QVariant RunningTasksModel::headerData(int section, Qt::Orientation orientation,
                                       int role) const
{
  if (orientation != Qt::Horizontal || role != Qt::DisplayRole)
    return QVariant();
  switch (section)
  {
    case Name:
      return QStringLiteral("Task");
    case Type:
      return QStringLiteral("Type");
    case Priority:
      return QStringLiteral("Priority");
    case Status:
      return QStringLiteral("Status");
    case Done:
      return QStringLiteral("Done");
    default:
      return QVariant();
  }
}

void RunningTasksModel::rebuild(const QVector<TaskRow>& rows)
{
  root_ = std::make_unique<Node>();
  for (const TaskRow& row : rows)
  {
    const QStringList segments = row.id.split('/', Qt::SkipEmptyParts);
    if (segments.isEmpty())
      continue;
    Node* node = root_.get();
    QString accumulated;
    for (const QString& segment : segments)
    {
      accumulated = accumulated.isEmpty() ? segment
                                          : accumulated + '/' + segment;
      // Find an existing child for this segment, else create it.
      Node* child = nullptr;
      for (const std::unique_ptr<Node>& candidate : node->children)
      {
        if (candidate->segment == segment)
        {
          child = candidate.get();
          break;
        }
      }
      if (!child)
      {
        auto created = std::make_unique<Node>();
        created->segment = segment;
        created->fullId = accumulated;
        created->parent = node;
        created->rowInParent = static_cast<int>(node->children.size());
        child = created.get();
        node->children.push_back(std::move(created));
      }
      node = child;
    }
    // The leaf node for this id carries the task data. An intermediate node that
    // is also an explicit task (both "survey_a" and "survey_a/line_1" present)
    // gets its data filled here when its own row is processed.
    node->row = row;
    node->hasTask = true;
  }
}

void RunningTasksModel::setTasks(const QString& current_task,
                                 const QVector<TaskRow>& rows)
{
  beginResetModel();
  current_task_ = current_task;
  rebuild(rows);
  endResetModel();
}

QString RunningTasksModel::idForIndex(const QModelIndex& index) const
{
  Node* n = nodeForIndex(index);
  return n ? n->fullId : QString();
}

RunningTasksModel::Node* RunningTasksModel::findById(Node* node,
                                                     const QString& id) const
{
  if (!node)
    return nullptr;
  if (node != root_.get() && node->fullId == id)
    return node;
  for (const std::unique_ptr<Node>& child : node->children)
  {
    if (Node* found = findById(child.get(), id))
      return found;
  }
  return nullptr;
}

QModelIndex RunningTasksModel::indexForId(const QString& id) const
{
  if (id.isEmpty())
    return QModelIndex();
  Node* node = findById(root_.get(), id);
  if (!node || node == root_.get())
    return QModelIndex();
  return createIndex(node->rowInParent, 0, node);
}
