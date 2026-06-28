#ifndef CAMP_ROS_WIDGET_H
#define CAMP_ROS_WIDGET_H

#include <QWidget>
#include <QGeoCoordinate>
#include "ros_client.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace camp_ros
{

class ROSWidget: public ROSClient<QWidget>
{
public:
  ROSWidget(QWidget *parent = nullptr);

protected:
  // timeout_sec bounds the TF wait. The default keeps the original blocking
  // behaviour for existing callers; pass 0.0 for a non-blocking lookup (e.g. on
  // the GUI thread in a per-frame loop, where a 1.5 s block per pose freezes the
  // UI — see camp#136).
  QGeoCoordinate getGeoCoordinate(const geometry_msgs::msg::Pose &pose, const std_msgs::msg::Header &header, double timeout_sec = 1.5);


};

}

#endif