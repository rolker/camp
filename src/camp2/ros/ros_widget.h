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
  QGeoCoordinate getGeoCoordinate(const geometry_msgs::msg::Pose &pose, const std_msgs::msg::Header &header);


};

}

#endif