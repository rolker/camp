#include "layer.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "node.h"
#include "project11/gz4d_geo.h"
#include <tf2/utils.h>
#include "../map_view/web_mercator.h"
#include <QApplication>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace camp
{

namespace ros
{

Layer::Layer(MapItem* parent, Node* node, const QString& object_name):
  map::Layer(parent, object_name), node_(node)
{

}


QPointF Layer::transformToWebMercator(const geometry_msgs::msg::Pose &pose, const std_msgs::msg::Header &header)
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header = header;
  ps.pose = pose;
  auto transform = node_->transformBuffer()->lookupTransform("earth", ps.header.frame_id, tf2::TimePointZero);

  geometry_msgs::msg::PoseStamped ecef;
  tf2::doTransform(ps, ecef, transform);

  gz4d::GeoPointECEF ecef_point;
  ecef_point[0] = ecef.pose.position.x;
  ecef_point[1] = ecef.pose.position.y;
  ecef_point[2] = ecef.pose.position.z;
  gz4d::GeoPointLatLongDegrees ll = ecef_point;
  return  web_mercator::geoToMap(QGeoCoordinate(ll.latitude(), ll.longitude(), ll.altitude()));

  // Don't do the try/catch here, let the caller handle the exception with somthing like the following...
  // try
  // {
  // }
  // catch (tf2::TransformException &ex)
  // {
  //   ROS_WARN_STREAM_THROTTLE(2.0, "Unable to find transform to earth at lookup time: "<< header.stamp << " now: " << ros::Time::now() << " source frame: " << header.frame_id << " what: " << ex.what());
  // }
  // return {};
}

} // namespace ros
} // namespace camp

