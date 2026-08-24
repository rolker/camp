#include "polygon.h"
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
  // [camp#213] Order matters: raise the gate, stop new callbacks at the source,
  // then wait for the render already running. Resetting the subscription alone
  // does not close the window — the executor can already be inside a callback.
  // The flag is advisory (the join is what actually synchronizes), so relaxed
  // ordering is sufficient.
  shutdown_.store(true, std::memory_order_relaxed);
  subscription_.reset();
  process_future_.waitForFinished();
}

void Polygon::polygonCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr data)
{
  if(shutdown_.load(std::memory_order_relaxed))
    return;
  if(!process_future_.isRunning())
  {
    process_future_ = QtConcurrent::run(this, &Polygon::processPolygon, data);
  }
}

void Polygon::processPolygon(const geometry_msgs::msg::PolygonStamped::SharedPtr data)
{
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
  if(shutdown_.load(std::memory_order_relaxed))
    return;

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
