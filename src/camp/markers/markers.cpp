#include "markers.h"

#include <chrono>
#include <utility>

#include <QPainter>
#include <QTimer>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "marine_autonomy/gz4d_geo.h"
#include "backgroundraster.h"
#include "markers/markers_converter.h"
#include "ros/ros_context.h"

namespace
{

// Wrap convertMarker() to capture the node so the converter can read clock+logger.
camp_ros::TfDispatcher<visualization_msgs::msg::Marker, std::shared_ptr<Markers::MarkerData>>::Converter
makeMarkerConverter(rclcpp::Node::SharedPtr node)
{
  return [node](const visualization_msgs::msg::Marker & m, tf2_ros::Buffer & buffer)
    -> std::optional<std::shared_ptr<Markers::MarkerData>>
  {
    return camp_markers::convertMarker(m, buffer, node->get_clock()->now(), node->get_logger());
  };
}

}  // namespace

Markers::Markers(QWidget* parent, QGraphicsItem *parentItem):
  camp_ros::ROSWidget(parent),
  GeoGraphicsItem(parentItem)
{
  ui_.setupUi(this);
  connect(ui_.displayCheckBox, &QCheckBox::stateChanged, this, &Markers::visibilityChanged);
  ui_.displayCheckBox->setChecked(true);
}

QRectF Markers::boundingRect() const
{
  QRectF ret;
  BackgroundRaster* bg = findParentBackgroundRaster();
  if(!current_markers_.empty() && is_visible_)
  {
    for(auto ns: current_markers_)
      for(auto m: ns.second)
      {
        auto p = markerPath(*(m.second), bg);
        ret |= p.boundingRect();
      }
  }
  return ret;
}

void Markers::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
  (void)option;
  (void)widget;
  BackgroundRaster* bg = findParentBackgroundRaster();
  if(!current_markers_.empty() && is_visible_)
    for(auto ns: current_markers_)
      for(auto m: ns.second)
      {
        painter->save();
        QPen p;
        p.setColor(QColor(m.second->marker.color.r*255, m.second->marker.color.g*255, m.second->marker.color.b*255, m.second->marker.color.a*255));
        if(m.second->marker.type == visualization_msgs::msg::Marker::LINE_STRIP)
        {
          p.setWidthF(m.second->marker.scale.x/pixel_size_);
        }
        if(m.second->marker.type == visualization_msgs::msg::Marker::TEXT_VIEW_FACING)
          p.setCosmetic(true);
        painter->setPen(p);
        QBrush b = painter->brush();
        b.setColor(QColor(m.second->marker.color.r*255, m.second->marker.color.g*255, m.second->marker.color.b*255, m.second->marker.color.a*128));
        if(m.second->marker.type == visualization_msgs::msg::Marker::SPHERE || m.second->marker.type == visualization_msgs::msg::Marker::TEXT_VIEW_FACING)
          b.setStyle(Qt::BrushStyle::SolidPattern);
        painter->setBrush(b);
        painter->drawPath(markerPath(*m.second, bg));
        painter->restore();
      }
}

QPainterPath Markers::markerPath(const MarkerData& marker, BackgroundRaster* bg) const
{
  QPainterPath path;

  if(bg)
  {
    switch(marker.marker.type)
    {
      case visualization_msgs::msg::Marker::SPHERE:
      {
        QRectF bbox(marker.local_position, marker.local_position);
        path.addEllipse(bbox.marginsAdded(QMarginsF(marker.marker.scale.x/2.0 /pixel_size_, marker.marker.scale.y/2.0/pixel_size_, marker.marker.scale.x/2.0/pixel_size_, marker.marker.scale.y/2.0/pixel_size_)));
        break;
      }
      case visualization_msgs::msg::Marker::LINE_STRIP:
      {
        auto cosr = cos(-marker.rotation);
        auto sinr = sin(-marker.rotation);
        for(auto p1 = marker.marker.points.begin(); p1 != marker.marker.points.end(); ++p1)
        {
          auto x = p1->x*cosr + p1->y*sinr;
          auto y = p1->x*sinr - p1->y*cosr;
          x /= pixel_size_;
          y /= pixel_size_;
          if(p1 == marker.marker.points.begin())
            path.moveTo(marker.local_position.x()+x,marker.local_position.y()+y);
          else
            path.lineTo(marker.local_position.x()+x, marker.local_position.y()+y);
        }
        break;
      }
      case visualization_msgs::msg::Marker::LINE_LIST:
      {
        auto cosr = cos(-marker.rotation);
        auto sinr = sin(-marker.rotation);
        for(auto p1 = marker.marker.points.begin(); p1 != marker.marker.points.end(); ++p1)
        {
          auto p2 = p1;
          p2++;
          if(p2 == marker.marker.points.end())
            break;
          auto x1 = p1->x*cosr + p1->y*sinr;
          auto y1 = p1->x*sinr - p1->y*cosr;
          x1 /= pixel_size_;
          y1 /= pixel_size_;
          auto x2 = p2->x*cosr + p2->y*sinr;
          auto y2 = p2->x*sinr - p2->y*cosr;
          x2 /= pixel_size_;
          y2 /= pixel_size_;
          path.moveTo(marker.local_position.x()+x1,marker.local_position.y()+y1);
          path.lineTo(marker.local_position.x()+x2, marker.local_position.y()+y2);
          p1++;
          if(p1 == marker.marker.points.end())
            break;
        }
        break;
      }
      case visualization_msgs::msg::Marker::TEXT_VIEW_FACING:
      {
        QFont font;
        int font_size = std::max(5, int(marker.marker.scale.z*bg->scaledPixelSize()*10));
        font.setPixelSize(font_size);
        QFontMetrics metrics(font);
        auto bounds = metrics.boundingRect(marker.marker.text.c_str());
        path.addText(QPointF(marker.local_position.x()-bounds.width()*pixel_size_/2.0, marker.local_position.y()+bounds.height()*pixel_size_/2.0), font, marker.marker.text.c_str());
        break;
      }
      default:
        if (node_)
          RCLCPP_WARN_STREAM(node_->get_logger(), "marker type not handled: " << marker.marker.type);
    }
  }
  return path;
}

void Markers::setTopic(std::string topic, std::string type)
{
  if (!node_) return;

  // Pick the scene callback group if RosContext is available so a slow
  // marker conversion can't starve realtime callbacks. Falls back to the
  // node's default group when RosContext hasn't been set (e.g. in tests
  // that don't bring up a full NodeThread).
  rclcpp::CallbackGroup::SharedPtr cbg;
  auto ctx = camp_ros::RosContext::instance();
  if (ctx) cbg = ctx->group(camp_ros::RosContext::Group::Scene);

  {
    // Hold the lock across the entire teardown + rebuild: this is the
    // window where onMarkerArrayMessage (executor thread) could see a
    // null or half-built marker_dispatcher_. Tear down the subscriber
    // first so it stops feeding the filter, then reset the dispatcher,
    // then construct the replacement before releasing the lock.
    std::lock_guard<std::mutex> lock(marker_dispatcher_mutex_);

    marker_subscription_.unsubscribe();
    marker_array_subscription_.reset();
    marker_dispatcher_.reset();

    // Build the dispatcher (target frame "earth", buffer 50 messages, drop
    // after 1 s without TF — preserves prior behavior).
    marker_dispatcher_ = std::make_shared<
      camp_ros::TfDispatcher<visualization_msgs::msg::Marker, std::shared_ptr<MarkerData>>>(
        node_, transform_buffer_, "earth", 50, std::chrono::seconds(1),
        makeMarkerConverter(node_),
        this,
        [this](std::shared_ptr<MarkerData> data) { this->onMarkerPayload(std::move(data)); });
  }

  if (type == "visualization_msgs/msg/MarkerArray")
  {
    rclcpp::SubscriptionOptions options;
    if (cbg) options.callback_group = cbg;
    marker_array_subscription_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
      topic, 1,
      [this](const visualization_msgs::msg::MarkerArray & data) { this->onMarkerArrayMessage(data); },
      options);
  }
  else if (type == "visualization_msgs/msg/Marker")
  {
    rclcpp::SubscriptionOptions options;
    if (cbg) options.callback_group = cbg;
    marker_subscription_.subscribe(node_, topic, rclcpp::QoS(1).get_rmw_qos_profile(), options);
    std::lock_guard<std::mutex> lock(marker_dispatcher_mutex_);
    marker_dispatcher_->connectInput(marker_subscription_);
  }

  ui_.topicLabel->setText(topic.c_str());
}

void Markers::setPixelSize(double s)
{
  pixel_size_ = s;
}

void Markers::onMarkerArrayMessage(const visualization_msgs::msg::MarkerArray & data)
{
  // Runs on the ROS executor thread. setTopic on the Qt thread can replace
  // marker_dispatcher_ underneath us. Snapshot the shared_ptr under the
  // mutex, then release the lock before iterating: the local copy keeps
  // the dispatcher alive for the duration of this callback even if
  // setTopic swaps it concurrently, and setTopic isn't blocked waiting
  // for a long MarkerArray to drain.
  std::shared_ptr<DispatcherT> dispatcher;
  {
    std::lock_guard<std::mutex> lock(marker_dispatcher_mutex_);
    dispatcher = marker_dispatcher_;
  }
  if (!dispatcher) return;
  for (const auto & marker : data.markers)
  {
    dispatcher->add(marker);
  }
}

void Markers::onMarkerPayload(std::shared_ptr<MarkerData> data)
{
  if (!data) return;
  prepareGeometryChange();

  auto * bg = findParentBackgroundRaster();
  switch (data->marker.action)
  {
    case visualization_msgs::msg::Marker::ADD:
      if (bg) data->local_position = geoToPixel(data->position, bg);
      current_markers_[data->marker.ns][data->marker.id] = data;
      if (rclcpp::Duration(data->marker.lifetime).seconds() != 0)
      {
        QTimer::singleShot(
          static_cast<int>((rclcpp::Duration(data->marker.lifetime).seconds() + 1.0) * 1000),
          this, [this]() {
            prepareGeometryChange();
            this->purgeExpiredMarkers();
            GeoGraphicsItem::update();
          });
      }
      break;
    case visualization_msgs::msg::Marker::DELETE:
      {
        auto ns_it = current_markers_.find(data->marker.ns);
        if (ns_it != current_markers_.end())
          ns_it->second.erase(data->marker.id);
      }
      break;
    case visualization_msgs::msg::Marker::DELETEALL:
      // visualization_msgs/Marker DELETEALL clears every marker across every
      // namespace; the message's own ns is ignored.
      current_markers_.clear();
      break;
    default:
      if (node_)
        RCLCPP_WARN_STREAM(node_->get_logger(),
          "Unknown marker action: " << static_cast<int>(data->marker.action));
  }

  purgeExpiredMarkers();
  GeoGraphicsItem::update();
}

void Markers::purgeExpiredMarkers()
{
  if (!node_) return;
  auto now = node_->get_clock()->now();
  for (auto & ns : current_markers_)
  {
    std::vector<int32_t> expired;
    for (auto & m : ns.second)
    {
      if (!m.second) continue;
      if (rclcpp::Time(m.second->marker.header.stamp).seconds() != 0.0 &&
          rclcpp::Duration(m.second->marker.lifetime).seconds() != 0 &&
          rclcpp::Time(m.second->marker.header.stamp) +
              rclcpp::Duration(m.second->marker.lifetime) < now)
      {
        expired.push_back(m.first);
      }
    }
    for (auto id : expired)
    {
      RCLCPP_DEBUG_STREAM(node_->get_logger(), "Purging " << ns.first << ": " << id);
      ns.second.erase(id);
    }
  }
}

void Markers::visibilityChanged()
{
  prepareGeometryChange();
  is_visible_ = ui_.displayCheckBox->isChecked();
  GeoGraphicsItem::update();
}


void Markers::updateBackground(BackgroundRaster * bg)
{
  prepareGeometryChange();
  setParentItem(bg);
  if(bg)
  {
    setPixelSize(bg->pixelSize());
  }
  GeoGraphicsItem::update();
}
