#include "markers.h"
#include <tf2/utils.h>
#include "marker_namespace.h"

namespace camp_ros
{

Markers::Markers(MapItem* parent, NodeManager* node_manager, QString topic, QString topic_type):
  Layer(parent, node_manager, topic), topic_(topic.toStdString())
{
  qRegisterMetaType<MarkerData>("MarkerData");

  connect(this, &Markers::newMarkerData, this, &Markers::updateMarker);

  if(topic_type == "visualization_msgs/MarkerArray")
  {
    subscriber_ = ros::NodeHandle().subscribe(topic_, 10, &Markers::markerArrayCallback, this);
    setStatus("[visualization_msgs/MarkerArray]");
  }
  else if(topic_type == "visualization_msgs/Marker")
  {
    subscriber_ = ros::NodeHandle().subscribe(topic_, 10, &Markers::markerCallback, this);
    setStatus("[visualization_msgs/Marker]");

  }
}


void Markers::markerArrayCallback(const visualization_msgs::MarkerArrayConstPtr &data)
{
  addMarkers(data->markers);
}

void Markers::markerCallback(const visualization_msgs::MarkerConstPtr &data)
{
  std::vector<visualization_msgs::Marker> markers;
  markers.push_back(*data);
  addMarkers(markers);
}

void Markers::addMarkers(const std::vector<visualization_msgs::Marker> &markers)
{
  for(auto m: markers)
  {
    try
    {
      MarkerData marker_data;
      marker_data.marker = m;
      if(m.action == visualization_msgs::Marker::ADD)
      {
        marker_data.position = transformToWebMercator(m.pose, m.header);
        marker_data.rotation = tf2::getYaw(m.pose.orientation);
      }
      emit newMarkerData(marker_data);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN_STREAM_THROTTLE(2.0, "Unable to find transform to earth for marker " << m.ns << ": " << m.id << " at lookup time: " << m.header.stamp << " now: " << ros::Time::now() << " source frame: " << m.header.frame_id << " what: " << ex.what());
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
  auto marker_namespace = markerNamespace(data.marker.ns.c_str());
  if(!marker_namespace)
    marker_namespace = new MarkerNamespace(this, node_manager_, data.marker.ns.c_str());
  marker_namespace->updateMarker(data);
}

} // namespace camp_ros
