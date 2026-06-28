#include "markers.h"
#include <tf2/utils.h>
#include "marker_namespace.h"
#include "../node.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace camp
{
namespace ros
{
namespace markers
{

Markers::Markers(MapItem* parent, Node* node, QString topic, QString topic_type):
  Layer(parent, node, topic), topic_(topic.toStdString())
{
  qRegisterMetaType<MarkerData>("MarkerData");

  connect(this, &Markers::newMarkerData, this, &Markers::updateMarker);


  rclcpp::QoS qos(10);
  qos.durability_volatile();

  if(topic_type == "visualization_msgs/msg/MarkerArray")
  {
    marker_array_subscription_ = node->node()->create_subscription<visualization_msgs::msg::MarkerArray>(topic_, qos, std::bind(&Markers::markerArrayCallback, this, std::placeholders::_1));
    setStatus("[visualization_msgs/msg/MarkerArray]");
  }
  else if(topic_type == "visualization_msgs/msg/Marker")
  {
    marker_subscription_ = node->node()->create_subscription<visualization_msgs::msg::Marker>(topic_, qos, std::bind(&Markers::markerCallback, this, std::placeholders::_1));
    setStatus("[visualization_msgs/msg/Marker]");

  }
}


void Markers::markerArrayCallback(const visualization_msgs::msg::MarkerArray &data)
{
  addMarkers(data.markers);
}

void Markers::markerCallback(const visualization_msgs::msg::Marker &data)
{
  std::vector<visualization_msgs::msg::Marker> markers;
  markers.push_back(data);
  addMarkers(markers);
}

void Markers::addMarkers(const std::vector<visualization_msgs::msg::Marker> &markers)
{
  // Throttle against the node's clock so log intervals track sim time when
  // use_sim_time is set, rather than a local system-time clock.
  auto clock = node_->node()->get_clock();
  auto now = clock->now();
  for(const auto& m: markers)
  {
    if(m.action == visualization_msgs::msg::Marker::ADD)
    {
      // [#70] Drop markers that are already expired on arrival. Without this
      // they'd be added, drawn once, then cleared by the 1 Hz expiry poll a
      // moment later. The non-zero-stamp guard mirrors Marker::checkExpired and
      // avoids comparing against a default-constructed (zero) stamp.
      // Ported from camp's markers_converter drop checks.
      // Construct the stamp in the node clock's time domain so the comparison
      // against `now` is valid under use_sim_time (rclcpp::Time rejects mixing
      // clock types, and the message stamp would otherwise default to ROS time).
      const rclcpp::Time stamp(m.header.stamp, clock->get_clock_type());
      if(stamp.nanoseconds() != 0 &&
         rclcpp::Duration(m.lifetime).nanoseconds() != 0 &&
         stamp + rclcpp::Duration(m.lifetime) < now)
      {
        RCLCPP_DEBUG_STREAM_THROTTLE(node_->node()->get_logger(), *clock, 5000, "Dropping already-expired marker " << m.ns << ": " << m.id);
        continue;
      }
      // [#70] Drop markers with no frame_id — the transform to earth can't be
      // resolved, so they would only throw and spam the catch below every frame.
      if(m.header.frame_id.empty())
      {
        RCLCPP_DEBUG_STREAM_THROTTLE(node_->node()->get_logger(), *clock, 1000, "Dropping marker with empty frame_id " << m.ns << ": " << m.id);
        continue;
      }
    }
    try
    {
      MarkerData marker_data;
      marker_data.marker = m;
      if(m.action == visualization_msgs::msg::Marker::ADD)
      {
        marker_data.position = transformToWebMercator(m.pose, m.header);
        marker_data.rotation = tf2::getYaw(m.pose.orientation);
      }
      emit newMarkerData(marker_data);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN_STREAM_THROTTLE(node_->node()->get_logger(), *clock, 2000, "Unable to find transform to earth for marker " << m.ns << ": " << m.id << " what: " << ex.what());
    }
  }
}

MarkerNamespace* Markers::markerNamespace(const QString& marker_namespace) const
{
  for(auto item: childItems())
  {
    auto ns = qgraphicsitem_cast<MarkerNamespace*>(item);
    if(ns && ns->objectName() == marker_namespace)
      return ns;
  }
  return nullptr;
}

void Markers::updateMarker(const MarkerData& data)
{
  const auto action = data.marker.action;

  // [#70] DELETEALL clears EVERY namespace per the visualization_msgs spec, not
  // just the message's own ns (which is conventionally empty). Fan it out to all
  // existing MarkerNamespaces; routing it to a single namespace (the previous
  // behavior) left markers in every other namespace as stale visuals on the
  // operator's map. Each MarkerNamespace::updateMarker handles DELETEALL by
  // deleting its own markers; prune the namespaces it empties.
  if(action == visualization_msgs::msg::Marker::DELETEALL)
  {
    for(auto item: childItems())
      if(auto* ns = qgraphicsitem_cast<MarkerNamespace*>(item))
        ns->updateMarker(data);
    pruneEmptyNamespaces();
    return;
  }

  // [#70] Warn on unrecognized actions instead of silently creating dead state
  // (the camp original logged this; the camp_map port dropped it).
  if(action != visualization_msgs::msg::Marker::ADD &&
     action != visualization_msgs::msg::Marker::MODIFY &&
     action != visualization_msgs::msg::Marker::DELETE)
  {
    RCLCPP_WARN_STREAM_THROTTLE(node_->node()->get_logger(), *node_->node()->get_clock(), 5000, "Unknown marker action " << static_cast<int>(action) << " for " << data.marker.ns << ": " << data.marker.id);
    return;
  }

  auto marker_namespace = markerNamespace(data.marker.ns.c_str());

  // [#70] A DELETE for a namespace we don't track is a no-op — never create a
  // namespace just to delete from it. Prune the namespace if this empties it.
  if(action == visualization_msgs::msg::Marker::DELETE)
  {
    if(marker_namespace)
    {
      marker_namespace->updateMarker(data);
      pruneEmptyNamespaces();
    }
    return;
  }

  // ADD / MODIFY
  if(!marker_namespace)
    marker_namespace = new MarkerNamespace(this, node_, data.marker.ns.c_str());
  marker_namespace->updateMarker(data);
}

void Markers::pruneEmptyNamespaces()
{
  // removeFromMap() detaches each namespace through the Map model (ADR-0003),
  // which calls setParentItem(nullptr) synchronously — so an emptied namespace
  // leaves childItems() within this loop and won't be revisited. [#70]
  for(auto item: childItems())
    if(auto* ns = qgraphicsitem_cast<MarkerNamespace*>(item))
      if(ns->isEmpty())
        ns->removeFromMap();
}

}  // namespace markers
}  // namespace ros
}  // namespace camp
