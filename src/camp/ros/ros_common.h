#ifndef CAMP_ROS_COMMON_H
#define CAMP_ROS_COMMON_H

#include <QMetaType>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

Q_DECLARE_METATYPE(rclcpp::Node::SharedPtr)
Q_DECLARE_METATYPE(tf2_ros::Buffer::SharedPtr)

#endif