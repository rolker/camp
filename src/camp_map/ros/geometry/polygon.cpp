#include "polygon.h"
#include "crash_handler.h"
#include "../node.h"
#include "../../map_view/web_mercator.h"
#include <QPen>
#include <QBrush>

#include <QDebug>

namespace camp
{
namespace ros
{
namespace geometry
{

Polygon::Polygon(MapItem* parent, Node* node, QString topic):
  Layer(parent, node, topic)
{
  qRegisterMetaType<PolygonData>("PolygonData");
  qRegisterMetaType<geometry_msgs::msg::PolygonStamped>("geometry_msgs::msg::PolygonStamped");

  connect(this, &Polygon::newPolygonData, this, &Polygon::updatePolygon);

  rclcpp::QoS qos(10);
  qos.durability_volatile();

  subscription_ = node->node()->create_subscription<geometry_msgs::msg::PolygonStamped>(
      topic.toStdString(), 10,
      std::bind(&Polygon::polygonCallback, this, std::placeholders::_1));

  setStatus("[geometry_msgs/msg/PolygonStamped]");
}

Polygon::~Polygon()
{
  // [camp#213] Stop callbacks at the source FIRST. Raising the gate alone does
  // not close the window: a callback that already passed the gate check can
  // still assign a NEW worker bound to `this`, which the future captured below
  // would not cover.
  subscription_.reset();

  // Then, under the lock, close the gate and take a copy of whatever render is
  // in flight. Joining the copy OUTSIDE the lock is what makes this safe against
  // the steal-and-run case: waitForFinished() may run the worker on this very
  // thread, and that worker takes mutex_ before its pre-emit check. Mirrors
  // GridMap::~GridMap().
  QFuture<void> pending;
  {
    QMutexLocker lock(&mutex_);
    shutdown_ = true;
    pending = process_future_;
  }
  pending.waitForFinished();
}

void Polygon::polygonCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr data)
{
  // [camp#213] Check the gate and assign the future under one lock, so the
  // destructor cannot observe a half-updated process_future_ and cannot be
  // raced past between the check and the launch.
  QMutexLocker lock(&mutex_);
  if(shutdown_)
    return;
  if(!process_future_.isRunning())
  {
    process_future_ = QtConcurrent::run(this, &Polygon::processPolygon, data);
  }
}

void Polygon::processPolygon(const geometry_msgs::msg::PolygonStamped::SharedPtr data)
{
  // [#217] First statement of a QtConcurrent worker: give this pool thread an
  // alternate signal stack, without which a stack-overflow SIGSEGV here cannot
  // be reported at all. Idempotent — a thread_local pointer test — so the pool
  // pays for it once per thread, not once per task. Guarded by the
  // check_worker_alt_stacks test; see camp_crash/crash_handler.h.
  camp_crash::install_thread_alt_stack();

  PolygonData polygon_data;

  // [camp#213] The destructor joins this worker, and a destructor is implicitly
  // noexcept — an escaping exception would be std::terminate(), not a crash we
  // could diagnose. frameOriginInWebMercator() throws tf2::TransformException on
  // a TF cold start, which is routine, so it must never leave this function.
  // Matches the guards in OccupancyGrid::processGrid() and GridMap::render().
  try
  {
    polygon_data.position = frameOriginInWebMercator(data->header);
  }
  catch(const std::exception& e)
  {
    RCLCPP_WARN_STREAM(node_->node()->get_logger(),
        "Failed to transform polygon origin: " << e.what());
    return;
  }

  for(const auto& point: data->polygon.points)
  {
    polygon_data.polygon << QPointF(point.x, point.y);
  }

  // [camp#213] waitForFinished() may steal a still-queued runnable and run it on
  // the GUI thread inside ~Polygon(). The emit would then be a direct call into
  // updatePolygon(), parenting a new item to a dying layer. Re-check the gate.
  // Safe to lock here: the destructor releases mutex_ before it joins.
  {
    QMutexLocker lock(&mutex_);
    if(shutdown_)
      return;
  }

  emit newPolygonData(polygon_data);
}

void Polygon::updatePolygon(const PolygonData &data)
{
  QGraphicsPolygonItem* polygon_item = firstChildOfType<QGraphicsPolygonItem>();
  if(polygon_item)
  {
    polygon_item->setPolygon(data.polygon);
  }
  else
  {
    polygon_item = new QGraphicsPolygonItem(data.polygon, this);
    QPen pen(Qt::blue);
    pen.setWidthF(0.5);
    polygon_item->setPen(pen);
    QBrush brush(QColor(0, 0, 255, 100));
    polygon_item->setBrush(brush);
  }
  auto map_distortion = web_mercator::metersPerUnit(data.position);
  double scale = 1.0/map_distortion;
  setTransform(QTransform::fromScale(scale, scale));
  setPos(data.position);
}

} // namespace geometry
} // namespace ros
} // namespace camp
