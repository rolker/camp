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

void Polygon::polygonCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr data)
{
  if(!process_future_.isRunning())
  {
    process_future_ = QtConcurrent::run(this, &Polygon::processPolygon, data);
  }
}

void Polygon::processPolygon(const geometry_msgs::msg::PolygonStamped::SharedPtr data)
{
  PolygonData polygon_data;
  polygon_data.position = frameOriginInWebMercator(data->header);

  for(const auto& point: data->polygon.points)
  {
    polygon_data.polygon << QPointF(point.x, point.y);
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
