#ifndef CAMP_COLLISION_MONITOR_CONVERTER_H
#define CAMP_COLLISION_MONITOR_CONVERTER_H

// PolygonStamped -> CollisionPolygonPayload converter, factored out for reuse
// and testing.
//
// The converter runs on the ROS executor thread inside a TfDispatcher. The
// dispatcher's MessageFilter has already verified that the polygon's frame is
// transformable to "earth" at the message stamp, so per-vertex lookups use the
// message stamp with no timeout. Errors here are buffer-edge races and we drop
// the polygon.
//
// Each vertex is transformed individually to "earth" -> geographic, which
// handles the base-frame heading (the polygons are published in a rotating
// base frame, e.g. base_link) without any manual yaw math. Collision-monitor
// zones have only a handful of vertices, so the per-vertex cost is negligible.

#include <memory>
#include <optional>
#include <vector>

#include <QGeoCoordinate>

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include "marine_autonomy/gz4d_geo.h"

namespace camp_collision_monitor
{

/// Polygon vertices in geographic coordinates, produced by the converter and
/// consumed by the Qt-side receiver (which maps them to scene pixels).
struct CollisionPolygonPayload
{
  std::vector<QGeoCoordinate> points;
};

/// Convert one PolygonStamped into a CollisionPolygonPayload, or std::nullopt
/// to drop (empty frame_id, or a vertex that fails to transform to "earth").
inline std::optional<std::shared_ptr<CollisionPolygonPayload>>
convertPolygon(const geometry_msgs::msg::PolygonStamped & msg,
               tf2_ros::Buffer & buffer,
               rclcpp::Logger logger)
{
  if (msg.header.frame_id.empty())
  {
    rclcpp::Clock clock;
    RCLCPP_DEBUG_STREAM_THROTTLE(logger, clock, 1000,
      "Collision-monitor polygon has empty frame_id; dropping");
    return std::nullopt;
  }

  auto payload = std::make_shared<CollisionPolygonPayload>();
  payload->points.reserve(msg.polygon.points.size());

  for (const auto & pt : msg.polygon.points)
  {
    geometry_msgs::msg::PointStamped ps;
    ps.header = msg.header;
    ps.point.x = pt.x;
    ps.point.y = pt.y;
    ps.point.z = pt.z;

    geometry_msgs::msg::PointStamped ecef;
    try
    {
      // Timeout 0: the dispatcher's MessageFilter has already verified the
      // transform is reachable at the message stamp.
      ecef = buffer.transform(ps, "earth", tf2::durationFromSec(0.0));
    }
    catch (const tf2::TransformException & ex)
    {
      rclcpp::Clock clock;
      RCLCPP_WARN_STREAM_THROTTLE(logger, clock, 1000,
        "Unable to transform collision-monitor polygon vertex ("
        << msg.header.frame_id << " -> earth): " << ex.what());
      return std::nullopt;
    }

    gz4d::GeoPointECEF ecef_point;
    ecef_point[0] = ecef.point.x;
    ecef_point[1] = ecef.point.y;
    ecef_point[2] = ecef.point.z;
    gz4d::GeoPointLatLongDegrees ll = ecef_point;
    payload->points.emplace_back(ll.latitude(), ll.longitude(), ll.altitude());
  }

  return payload;
}

} // namespace camp_collision_monitor

#endif
