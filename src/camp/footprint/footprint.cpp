#include "footprint.h"

#include <chrono>
#include <utility>

#include <QPainter>
#include <QPainterPath>
#include <QPen>

#include "ros/ros_context.h"

Footprint::Footprint(QObject* parent, QGraphicsItem* parentItem):
  camp_ros::ROSObject(parent),
  GeoGraphicsItem(parentItem)
{
}

QRectF Footprint::boundingRect() const
{
  return polygonPath().boundingRect();
}

void Footprint::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
  (void)option;
  (void)widget;
  auto path = polygonPath();
  if(path.isEmpty())
    return;

  painter->save();
  // Cosmetic pen: constant on-screen width regardless of map zoom.
  QPen pen(color_);
  pen.setCosmetic(true);
  pen.setWidthF(2.0);
  painter->setPen(pen);
  painter->setBrush(Qt::NoBrush);
  painter->drawPath(path);
  painter->restore();
}

QPainterPath Footprint::polygonPath() const
{
  QPainterPath path;
  // The footprint is anchored to the persistent scene root and projected via the
  // bg-free geoToPixel (Web-Mercator scene), so it renders with or without a
  // chart loaded. A closed outline needs at least 3 vertices.
  if(points_.size() >= 3)
  {
    bool first = true;
    for(const auto& gc: points_)
    {
      QPointF px = geoToPixel(gc);
      if(first)
      {
        path.moveTo(px);
        first = false;
      }
      else
      {
        path.lineTo(px);
      }
    }
    path.closeSubpath();
  }
  return path;
}

void Footprint::onNodeUpdated()
{
  // (Re)subscribe when the node (re)appears and a topic has been assigned.
  if(node_ && !topic_.empty())
    setTopic(topic_);
}

void Footprint::setTopic(std::string topic)
{
  topic_ = std::move(topic);
  if(!node_)
    return;

  // Use the Scene callback group so polygon conversion can't starve realtime
  // callbacks; fall back to the node default otherwise.
  rclcpp::CallbackGroup::SharedPtr cbg;
  auto ctx = camp_ros::RosContext::instance();
  if(ctx)
    cbg = ctx->group(camp_ros::RosContext::Group::Scene);

  auto node = node_;
  // QoS(1), reliable/volatile — matches the collision/marker overlays. The
  // converter transforms each vertex to "earth"->geographic, handling the
  // (rotating) publish frame without manual yaw math.
  bridge_ = std::make_unique<camp_ros::TfTopicBridge<geometry_msgs::msg::PolygonStamped, PayloadT>>(
    node_, transform_buffer_, topic_, rclcpp::QoS(1), "earth", 50, std::chrono::seconds(1), cbg,
    [node](const geometry_msgs::msg::PolygonStamped& msg, tf2_ros::Buffer& buffer)
      -> std::optional<PayloadT>
    {
      return camp_collision_monitor::convertPolygon(msg, buffer, node->get_logger());
    },
    this,
    [this](PayloadT data) { this->onPolygonPayload(std::move(data)); });
}

void Footprint::onPolygonPayload(PayloadT data)
{
  if(!data)
    return;
  prepareGeometryChange();
  points_ = std::move(data->points);
  GeoGraphicsItem::update();
}

void Footprint::updateBackground()
{
  prepareGeometryChange();
  GeoGraphicsItem::update();
}
