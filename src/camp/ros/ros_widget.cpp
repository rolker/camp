#include "ros_widget.h"
#include "marine_autonomy/gz4d_geo.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace camp_ros
{

ROSWidget::ROSWidget(QWidget *parent)
  :ROSClient<QWidget>(parent)
{

}

QGeoCoordinate ROSWidget::getGeoCoordinate(const geometry_msgs::msg::Pose &pose, const std_msgs::msg::Header &header)
{
    if(header.frame_id.empty())
      return {};

    geometry_msgs::msg::PoseStamped ps;
    ps.header = header;
    ps.pose = pose;

    geometry_msgs::msg::PoseStamped ecef;
    try
    {
      ecef = transform_buffer_->transform(ps, "earth", tf2::durationFromSec(1.5));
    }
    catch (const tf2::TransformException &)
    {
      // A stale-stamped pose - e.g. a nav_msgs/Path overlay whose poses
      // carry old planning stamps arriving over the bridge - throws
      // (typically ExtrapolationException) and, left uncaught, aborts the
      // whole application. The map->earth transform is quasi-static, so
      // retry against the latest available transform (stamp 0 == latest)
      // rather than the pose's own stale stamp.
      try
      {
        ps.header.stamp.sec = 0;
        ps.header.stamp.nanosec = 0;
        ecef = transform_buffer_->transform(ps, "earth", tf2::durationFromSec(1.5));
      }
      catch (const tf2::TransformException &ex)
      {
        rclcpp::Clock clock;
        RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), clock, 2000,
          "ROSWidget: unable to transform pose to earth: " << ex.what()
          << " source frame: " << header.frame_id);
        return {};
      }
    }

    gz4d::GeoPointECEF ecef_point;
    ecef_point[0] = ecef.pose.position.x;
    ecef_point[1] = ecef.pose.position.y;
    ecef_point[2] = ecef.pose.position.z;
    gz4d::GeoPointLatLongDegrees ll = ecef_point;
    return QGeoCoordinate(ll.latitude(), ll.longitude(), ll.altitude());
}


} // namespace camp_ros