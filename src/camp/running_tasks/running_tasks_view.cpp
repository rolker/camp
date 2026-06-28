#include "running_tasks/running_tasks_view.h"

#include <QHeaderView>
#include <QItemSelectionModel>
#include <QMetaObject>
#include <QPalette>
#include <QSignalBlocker>
#include <QTimer>
#include <QTreeView>
#include <QVBoxLayout>

#include "ros/ros_context.h"

RunningTasksView::RunningTasksView(QWidget* parent)
  : QWidget(parent),
    tree_(new QTreeView(this)),
    model_(new RunningTasksModel(this)),
    watchdog_timer_(new QTimer(this)),
    max_green_duration_(2, 0),
    max_yellow_duration_(5, 0)
{
  tree_->setModel(model_);
  tree_->setRootIsDecorated(true);
  tree_->setSelectionBehavior(QAbstractItemView::SelectRows);
  tree_->setSelectionMode(QAbstractItemView::SingleSelection);
  tree_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  tree_->header()->setStretchLastSection(true);

  // The task tree is the primary pane; claim vertical space so it grows when the
  // panel is stretched instead of the helm status / button row absorbing it.
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);

  auto* layout = new QVBoxLayout(this);
  // A small margin lets the staleness window color show as a frame around the
  // tree (the tree viewport paints its own background over the rest).
  layout->setContentsMargins(3, 3, 3, 3);
  layout->addWidget(tree_);

  connect(tree_->selectionModel(), &QItemSelectionModel::currentRowChanged,
          this, &RunningTasksView::onCurrentRowChanged);
  connect(watchdog_timer_, &QTimer::timeout, this,
          &RunningTasksView::watchdogUpdate);
}

RunningTasksView::~RunningTasksView()
{
  // Stop new callback dispatch before the members the callback touches
  // (pending_mutex_, pending_tasks_) are torn down — they are declared after
  // subscription_ and so are destroyed first under reverse-order destruction.
  // (Full safety on shutdown also relies on the CAMP executor being stopped
  // before these widgets are destroyed; this guards the local ordering.)
  subscription_.reset();
}

void RunningTasksView::setNode(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  subscribe();
  // The staleness watchdog needs the node clock; start it once we have a node
  // (mirrors HelmManager, which starts its 500 ms watchdog after setNode).
  if (node_ && !watchdog_timer_->isActive())
    watchdog_timer_->start(500);
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
  // Runs on the ROS executor thread — stash the raw snapshot and hand off to the
  // GUI thread; never touch widgets (or the model's TaskList) from this thread.
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_current_ = QString::fromStdString(msg.current_navigation_task);
    pending_tasks_ = msg.tasks;
  }
  QMetaObject::invokeMethod(this, "applyPendingTasks", Qt::QueuedConnection);
}

void RunningTasksView::applyPendingTasks()
{
  QString current;
  std::vector<marine_nav_interfaces::msg::TaskInformation> tasks;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    current = pending_current_;
    tasks = pending_tasks_;
  }

  // Record receipt for the staleness watchdog (TaskFeedback has no header, so
  // staleness is measured from receive time).
  if (node_)
  {
    last_message_time_ = node_->get_clock()->now();
    has_message_ = true;
  }

  // Preserve the selected task across the model reset.
  const QString selected = model_->idForIndex(tree_->currentIndex());

  // TaskList needs a clock to stamp newly created tasks; node_ is always set by
  // the time feedback arrives (a subscription requires it).
  model_->setTasks(current, tasks,
                   node_ ? node_->get_clock() : rclcpp::Clock::make_shared());
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

void RunningTasksView::watchdogUpdate()
{
  // Only color once messages have started arriving (matches HelmManager, which
  // gates on a non-zero last-heartbeat timestamp). Before then, leave the
  // default (un-filled) background.
  if (!node_ || !has_message_)
    return;

  const auto age = node_->get_clock()->now() - last_message_time_;
  QPalette pal = palette();
  if (age < max_green_duration_)
    pal.setColor(QPalette::Window, Qt::green);
  else if (age < max_yellow_duration_)
    pal.setColor(QPalette::Window, Qt::yellow);
  else
    pal.setColor(QPalette::Window, Qt::red);
  setAutoFillBackground(true);
  setPalette(pal);
}
