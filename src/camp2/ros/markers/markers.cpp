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
  for(auto m: markers)
  {
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
      rclcpp::Clock clock;
      RCLCPP_WARN_STREAM_THROTTLE(node_->node()->get_logger(), clock, 2000, "Unable to find transform to earth for marker " << m.ns << ": " << m.id << " what: " << ex.what());
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
  // [#70] DELETEALL clears EVERY namespace per the visualization_msgs spec, not
  // just the message's own ns (which is conventionally empty). Fan it out to all
  // existing MarkerNamespaces; routing it to a single namespace (the previous
  // behavior) left markers in every other namespace as stale visuals on the
  // operator's map. Each MarkerNamespace::updateMarker handles DELETEALL by
  // clearing its own markers.
  if(data.marker.action == visualization_msgs::msg::Marker::DELETEALL)
  {
    for(auto item: childItems())
      if(auto* ns = qgraphicsitem_cast<MarkerNamespace*>(item))
        ns->updateMarker(data);
    return;
  }

  auto marker_namespace = markerNamespace(data.marker.ns.c_str());
  if(!marker_namespace)
    marker_namespace = new MarkerNamespace(this, node_, data.marker.ns.c_str());
  marker_namespace->updateMarker(data);
}

}  // namespace markers
}  // namespace ros
}  // namespace camp
