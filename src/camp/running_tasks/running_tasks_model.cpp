#include "running_tasks/running_tasks_model.h"

#include <algorithm>

#include <QBrush>
#include <QColor>
#include <QFont>
#include <QStringList>

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
  if (!n || !n->task)
    return QVariant();

  const auto& msg = n->task->message();
  const bool is_current = !current_task_.isEmpty() && n->fullId == current_task_;

  switch (role)
  {
    case Qt::DisplayRole:
      switch (index.column())
      {
        case Name:
          return n->segment;
        case Type:
          return QString::fromStdString(msg.type);
        case Priority:
          return QVariant(msg.priority);
        case Status:
          // Status is a YAML blob; show its first line as a summary for now.
          return QString::fromStdString(msg.status).section('\n', 0, 0).trimmed();
        case Done:
          return msg.done ? QStringLiteral("done") : QString();
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
      return n->fullId;
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

void RunningTasksModel::buildNodes(const marine_nav_tasks::TaskList& task_list,
                                   Node* parent_node)
{
  // Snapshot the domain task tree into display nodes, preserving the task list's
  // (message / run) order. Done-to-bottom reordering happens afterwards.
  for (const marine_nav_tasks::TaskPtr& task : task_list.tasks())
  {
    if (!task)
      continue;
    auto node = std::make_unique<Node>();
    node->task = task;
    node->fullId = QString::fromStdString(task->message().id);
    const QStringList parts = node->fullId.split('/', Qt::SkipEmptyParts);
    node->segment = parts.isEmpty() ? node->fullId : parts.last();
    node->parent = parent_node;
    node->rowInParent = static_cast<int>(parent_node->children.size());
    Node* raw = node.get();
    parent_node->children.push_back(std::move(node));
    buildNodes(task->children(), raw);
  }
}

void RunningTasksModel::sortChildrenRecursive(Node* node)
{
  // Float done tasks to the bottom at each level, preserving run order within
  // each group (stable_sort), then re-index rowInParent so index()/parent()/
  // indexForId stay consistent with the new order.
  std::stable_sort(
      node->children.begin(), node->children.end(),
      [](const std::unique_ptr<Node>& a, const std::unique_ptr<Node>& b)
      {
        const bool a_done = a->task && a->task->message().done;
        const bool b_done = b->task && b->task->message().done;
        // not-done (false) sorts before done (true); equal keeps run order.
        return a_done < b_done;
      });
  for (std::size_t i = 0; i < node->children.size(); ++i)
  {
    node->children[i]->rowInParent = static_cast<int>(i);
    sortChildrenRecursive(node->children[i].get());
  }
}

void RunningTasksModel::setTasks(
    const QString& current_task,
    const std::vector<marine_nav_interfaces::msg::TaskInformation>& tasks,
    rclcpp::Clock::SharedPtr clock)
{
  beginResetModel();
  // Normalize the current-task id the same way task ids are formed so the
  // highlight matches even if it arrives with stray slashes.
  current_task_ = current_task.split('/', Qt::SkipEmptyParts).join('/');
  // TaskList owns the canonical hierarchy/order/done/data; we snapshot it.
  task_list_.update(tasks, clock);
  root_ = std::make_unique<Node>();
  buildNodes(task_list_, root_.get());
  sortChildrenRecursive(root_.get());
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
