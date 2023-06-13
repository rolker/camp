#include "layer.h"
#include <geometry_msgs/PoseStamped.h>
#include "node_manager.h"
#include "gz4d_geo.h"
#include <tf2/utils.h>
#include "../map_view/web_mercator.h"
#include <QApplication>


namespace camp_ros
{

Layer::Layer(MapItem* parent, NodeManager* node_manager, const QString& object_name):
  map::Layer(parent, object_name), node_manager_(node_manager)
{
  connect(QApplication::instance(), &QCoreApplication::aboutToQuit, this, &Layer::unsubscribe);
  connect(node_manager, &NodeManager::shuttingDownRos, this, &Layer::unsubscribe);
}


QPointF Layer::transformToWebMercator(const geometry_msgs::Pose &pose, const std_msgs::Header &header)
{
  geometry_msgs::PoseStamped ps;
  ps.header = header;
  ps.pose = pose;
  auto ecef = node_manager_->transformBuffer().transform(ps, "earth", ros::Duration(1.5));

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

void Layer::unsubscribe()
{
  subscriber_.shutdown();
}

} // namespace camp_ros
