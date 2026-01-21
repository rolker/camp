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
    geometry_msgs::msg::PoseStamped ps;
    ps.header = header;
    ps.pose = pose;
    auto ecef = transform_buffer_->transform(ps, "earth", tf2::durationFromSec(1.5));

    gz4d::GeoPointECEF ecef_point;
    ecef_point[0] = ecef.pose.position.x;
    ecef_point[1] = ecef.pose.position.y;
    ecef_point[2] = ecef.pose.position.z;
    gz4d::GeoPointLatLongDegrees ll = ecef_point;
    return QGeoCoordinate(ll.latitude(), ll.longitude(), ll.altitude());
}


} // namespace camp_ros