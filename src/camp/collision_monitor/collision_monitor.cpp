#include "collision_monitor.h"

#include <chrono>
#include <utility>

#include <QBrush>
#include <QPainter>
#include <QPainterPath>
#include <QPen>

#include "backgroundraster.h"
#include "ros/ros_context.h"

CollisionMonitor::CollisionMonitor(QWidget* parent, QGraphicsItem* parentItem):
  camp_ros::ROSWidget(parent),
  GeoGraphicsItem(parentItem)
{
  ui_.setupUi(this);
  connect(ui_.displayCheckBox, &QCheckBox::stateChanged, this, &CollisionMonitor::visibilityChanged);
  ui_.displayCheckBox->setChecked(true);
}

QRectF CollisionMonitor::boundingRect() const
{
  if(!is_visible_)
    return QRectF();
  return polygonPath().boundingRect();
}

void CollisionMonitor::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
  (void)option;
  (void)widget;
  if(!is_visible_)
    return;

  auto path = polygonPath();
  if(path.isEmpty())
    return;

  painter->save();
  // Cosmetic pen: constant on-screen line width regardless of map zoom.
  QPen pen(color_);
  pen.setCosmetic(true);
  pen.setWidthF(active_ ? 2.0 : 1.0);
  painter->setPen(pen);
  if(active_)
  {
    QColor fill = color_;
    fill.setAlpha(90);
    painter->setBrush(QBrush(fill));
  }
  else
  {
    painter->setBrush(Qt::NoBrush);
  }
  painter->drawPath(path);
  painter->restore();
}

QPainterPath CollisionMonitor::polygonPath() const
{
  QPainterPath path;
  // [#59 PR6] Chart-independent: the zone is anchored to the persistent scene
  // root and projected via the bg-free geoToPixel (Web-Mercator scene), so it
  // renders with or without a chart loaded.
  // A closed, fillable polygon needs at least 3 vertices; fewer would
  // closeSubpath() into a degenerate line. Collision-monitor zones are always
  // >=3 (4 in practice), so this is a defensive guard.
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

void CollisionMonitor::setColor(QColor color)
{
  color_ = color;
  GeoGraphicsItem::update();
}

void CollisionMonitor::setTopic(std::string topic)
{
  if(!node_)
    return;

  // Use the Scene callback group when available so polygon conversion can't
  // starve realtime callbacks; falls back to the node default otherwise.
  rclcpp::CallbackGroup::SharedPtr cbg;
  auto ctx = camp_ros::RosContext::instance();
  if(ctx)
    cbg = ctx->group(camp_ros::RosContext::Group::Scene);

  auto node = node_;
  // QoS(1), reliable/volatile — matches the markers/grid overlays. If a future
  // bridge republishes these polygons best-effort, switch to best_effort here.
  bridge_ = std::make_unique<camp_ros::TfTopicBridge<geometry_msgs::msg::PolygonStamped, PayloadT>>(
    node_, transform_buffer_, topic, rclcpp::QoS(1), "earth", 50, std::chrono::seconds(1), cbg,
    [node](const geometry_msgs::msg::PolygonStamped& msg, tf2_ros::Buffer& buffer)
      -> std::optional<PayloadT>
    {
      return camp_collision_monitor::convertPolygon(msg, buffer, node->get_logger());
    },
    this,
    [this](PayloadT data) { this->onPolygonPayload(std::move(data)); });

  ui_.topicLabel->setText(topic.c_str());
}

void CollisionMonitor::setActive(bool active)
{
  if(active_ == active)
    return;
  active_ = active;
  GeoGraphicsItem::update();
}

void CollisionMonitor::onPolygonPayload(PayloadT data)
{
  if(!data)
    return;
  prepareGeometryChange();
  points_ = std::move(data->points);
  GeoGraphicsItem::update();
}

void CollisionMonitor::visibilityChanged()
{
  prepareGeometryChange();
  is_visible_ = ui_.displayCheckBox->isChecked();
  GeoGraphicsItem::update();
}

void CollisionMonitor::updateBackground(BackgroundRaster* bg)
{
  // [#59 PR6] The zone is parented to the persistent scene anchor at creation
  // and stays there — no reparenting to the (possibly-null) BackgroundRaster.
  // Positions are absolute Web-Mercator; just trigger a repaint on chart change.
  Q_UNUSED(bg);
  prepareGeometryChange();
  GeoGraphicsItem::update();
}
