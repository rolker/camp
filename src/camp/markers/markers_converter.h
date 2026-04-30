#ifndef CAMP_MARKERS_CONVERTER_H
#define CAMP_MARKERS_CONVERTER_H

// Marker → MarkerPayload converter, factored out for unit testing.
//
// The converter runs on the ROS executor thread inside a TfDispatcher. The
// dispatcher's MessageFilter has already verified that the marker's frame is
// transformable to "earth" at the message stamp, so the buffer lookup uses
// the message stamp with no timeout. Errors here are buffer-edge races and
// we drop the marker.

#include <memory>
#include <optional>

#include <QGeoCoordinate>
#include <QPointF>

#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "marine_autonomy/gz4d_geo.h"

namespace camp_markers
{

/// Per-marker payload produced by the converter and consumed by the Qt-side
/// receiver. `local_position` is filled in by the receiver after geoToPixel,
/// not by the converter.
struct MarkerPayload
{
  visualization_msgs::msg::Marker marker;
  QGeoCoordinate position;
  QPointF local_position;
  double rotation = 0.0;
};

/// Convert one Marker message into a MarkerPayload, or std::nullopt to drop.
///
/// Drops:
///   - ADD markers whose lifetime has already expired relative to `now()`.
///   - ADD markers with empty frame_id.
///   - ADD markers whose pose can't be transformed from frame_id to "earth".
///
/// DELETE / DELETEALL markers are returned with marker copied and no
/// position/rotation set; the receiver acts on `marker.action`.
inline std::optional<std::shared_ptr<MarkerPayload>>
convertMarker(const visualization_msgs::msg::Marker & m,
              tf2_ros::Buffer & buffer,
              const rclcpp::Time & now,
              rclcpp::Logger logger)
{
  auto payload = std::make_shared<MarkerPayload>();
  payload->marker = m;

  if (m.action == visualization_msgs::msg::Marker::ADD)
  {
    if (rclcpp::Duration(m.lifetime).nanoseconds() != 0 &&
        rclcpp::Time(m.header.stamp) + rclcpp::Duration(m.lifetime) < now)
    {
      rclcpp::Clock clock;
      RCLCPP_DEBUG_STREAM_THROTTLE(logger, clock, 5000,
        "Expired marker: " << m.ns << " id: " << m.id);
      return std::nullopt;
    }
    if (m.header.frame_id.empty())
    {
      rclcpp::Clock clock;
      RCLCPP_DEBUG_STREAM_THROTTLE(logger, clock, 1000,
        "Missing frame_id in marker: " << m.ns << " id: " << m.id);
      return std::nullopt;
    }

    try
    {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = m.header;
      ps.pose = m.pose;
      // Timeout 0: don't wait. The dispatcher's MessageFilter has already
      // verified the transform is reachable at the message stamp.
      auto ecef = buffer.transform(ps, "earth", tf2::durationFromSec(0.0));

      gz4d::GeoPointECEF ecef_point;
      ecef_point[0] = ecef.pose.position.x;
      ecef_point[1] = ecef.pose.position.y;
      ecef_point[2] = ecef.pose.position.z;
      gz4d::GeoPointLatLongDegrees ll = ecef_point;
      payload->position = QGeoCoordinate(ll.latitude(), ll.longitude(), ll.altitude());
      payload->rotation = tf2::getYaw(m.pose.orientation);
    }
    catch (const tf2::TransformException & ex)
    {
      rclcpp::Clock clock;
      RCLCPP_WARN_STREAM_THROTTLE(logger, clock, 1000,
        "Unable to transform marker " << m.ns << ":" << m.id
        << " (" << m.header.frame_id << " -> earth): " << ex.what());
      return std::nullopt;
    }
  }
  return payload;
}

} // namespace camp_markers

#endif
