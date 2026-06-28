#include "running_tasks/running_tasks_view.h"

#include <QHeaderView>
#include <QItemSelectionModel>
#include <QMetaObject>
#include <QSignalBlocker>
#include <QTreeView>
#include <QVBoxLayout>

#include "ros/ros_context.h"

RunningTasksView::RunningTasksView(QWidget* parent)
  : QWidget(parent),
    tree_(new QTreeView(this)),
    model_(new RunningTasksModel(this))
{
  tree_->setModel(model_);
  tree_->setRootIsDecorated(true);
  tree_->setSelectionBehavior(QAbstractItemView::SelectRows);
  tree_->setSelectionMode(QAbstractItemView::SingleSelection);
  tree_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  tree_->header()->setStretchLastSection(true);

  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(tree_);

  connect(tree_->selectionModel(), &QItemSelectionModel::currentRowChanged,
          this, &RunningTasksView::onCurrentRowChanged);
}

RunningTasksView::~RunningTasksView()
{
  // Stop new callback dispatch before the members the callback touches
  // (pending_mutex_, pending_rows_) are torn down — they are declared after
  // subscription_ and so are destroyed first under reverse-order destruction.
  // (Full safety on shutdown also relies on the CAMP executor being stopped
  // before these widgets are destroyed; this guards the local ordering.)
  subscription_.reset();
}

void RunningTasksView::setNode(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  subscribe();
}

void RunningTasksView::updateRobotNamespace(QString robot_namespace)
{
  robot_namespace_ = robot_namespace;
  subscribe();
}

void RunningTasksView::subscribe()
{
  // Need both a node and a namespace before a subscription can be created.
  subscription_.reset();
  if (!node_ || robot_namespace_.isEmpty())
    return;

  // Match the boat publisher: marine/status/mission_tasks, depth-10 volatile.
  // Robustness over the udp_bridge comes from the boat's periodic re-publish,
  // not transient_local. Join the curated Realtime callback group when running
  // inside CAMP; in an rqt host RosContext::instance() is empty and the
  // subscription falls back to the node's default group.
  rclcpp::SubscriptionOptions options;
  if (auto ctx = camp_ros::RosContext::instance())
    options.callback_group =
        ctx->group(camp_ros::RosContext::Group::Realtime);

  const std::string topic =
      "/" + robot_namespace_.toStdString() + "/marine/status/mission_tasks";
  subscription_ =
      node_->create_subscription<marine_nav_interfaces::msg::TaskFeedback>(
          topic, rclcpp::QoS(10),
          std::bind(&RunningTasksView::taskFeedbackCallback, this,
                    std::placeholders::_1),
          options);
}

void RunningTasksView::taskFeedbackCallback(
    const marine_nav_interfaces::msg::TaskFeedback& msg)
{
  // Runs on the ROS executor thread — convert to Qt-friendly rows here and
  // hand off to the GUI thread; never touch widgets from this thread.
  QVector<TaskRow> rows;
  rows.reserve(static_cast<int>(msg.tasks.size()));
  for (const auto& task : msg.tasks)
  {
    TaskRow row;
    row.id = QString::fromStdString(task.id);
    row.type = QString::fromStdString(task.type);
    row.priority = task.priority;
    // Status is a YAML blob; show its first line as a summary for P1.
    row.status =
        QString::fromStdString(task.status).section('\n', 0, 0).trimmed();
    row.done = task.done;
    rows.push_back(row);
  }

  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_current_ = QString::fromStdString(msg.current_navigation_task);
    pending_rows_ = rows;
  }
  QMetaObject::invokeMethod(this, "applyPendingTasks", Qt::QueuedConnection);
}

void RunningTasksView::applyPendingTasks()
{
  QString current;
  QVector<TaskRow> rows;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    current = pending_current_;
    rows = pending_rows_;
  }

  // Preserve the selected task across the model reset.
  const QString selected = model_->idForIndex(tree_->currentIndex());

  model_->setTasks(current, rows);
  tree_->expandAll();

  if (!selected.isEmpty())
  {
    const QModelIndex restored = model_->indexForId(selected);
    if (restored.isValid())
    {
      // Restoring the prior selection must not look like a fresh user
      // selection: block signals so taskSelected() isn't re-emitted on every
      // periodic republish (which would re-trigger map glue in P2).
      const QSignalBlocker blocker(tree_->selectionModel());
      tree_->setCurrentIndex(restored);
    }
  }
}

void RunningTasksView::onCurrentRowChanged(const QModelIndex& current,
                                           const QModelIndex& /*previous*/)
{
  const QString id = model_->idForIndex(current);
  if (!id.isEmpty())
    emit taskSelected(id);
}

void RunningTasksView::setSelectedTask(QString id)
{
  const QModelIndex index = model_->indexForId(id);
  if (index.isValid())
    tree_->setCurrentIndex(index);
}
