#include "layer.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "node.h"
#include "marine_autonomy/gz4d_geo.h"
#include <tf2/utils.h>
#include "../map_view/web_mercator.h"
#include <QApplication>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace camp
{

namespace ros
{

namespace
{
// How long a cached earth<-frame transform stays usable after the live lookup
// starts failing. The earth<-map/odom transform is quasi-static, so reusing it
// across a brief TF stream hiccup (a few dropped messages) is imperceptible and
// far better than blanking the layer. Bounded so a sustained outage — or a
// genuinely moving frame — eventually drops rather than showing stale data.
constexpr std::chrono::duration<double> kTransformStalenessBudget{1.0};
}  // namespace

Layer::Layer(MapItem* parent, Node* node, const QString& object_name):
  map::Layer(parent, object_name), node_(node)
{

}


geometry_msgs::msg::TransformStamped Layer::lookupEarthTransform(const std::string& frame_id)
{
  std::lock_guard<std::mutex> lock(transform_cache_mutex_);
  return lookupEarthTransformCached(*node_->transformBuffer(), transform_cache_, frame_id,
                                    std::chrono::steady_clock::now(), kTransformStalenessBudget);
}


QPointF Layer::transformToWebMercator(const geometry_msgs::msg::Pose &pose, const std_msgs::msg::Header &header)
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header = header;
  ps.pose = pose;
  auto transform = lookupEarthTransform(ps.header.frame_id);

  geometry_msgs::msg::PoseStamped ecef;
  tf2::doTransform(ps, ecef, transform);

  gz4d::GeoPointECEF ecef_point;
  ecef_point[0] = ecef.pose.position.x;
  ecef_point[1] = ecef.pose.position.y;
  ecef_point[2] = ecef.pose.position.z;
  gz4d::GeoPointLatLongDegrees ll = ecef_point;
  return  web_mercator::geoToMap(QGeoCoordinate(ll.latitude(), ll.longitude(), ll.altitude()));
}

QPointF Layer::frameOriginInWebMercator(const std_msgs::msg::Header &header)
{
  geometry_msgs::msg::Pose origin;
  origin.orientation.w = 1.0;
  return transformToWebMercator(origin, header);
}

} // namespace ros
} // namespace camp

