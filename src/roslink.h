#ifndef ROSLINK_H
#define ROSLINK_H

#include <QWidget>
#include "geographicsitem.h"

#include "ros/ros_common.h"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "locationposition.h"
#include <QThread>

namespace Ui
{
class ROSLink;
}

class ROSLink : public QWidget
{
  Q_OBJECT
public:
  ROSLink(QWidget* parent);

  rclcpp::Node::SharedPtr node();
  tf2_ros::Buffer::SharedPtr tfBuffer();

signals:

  void rosConnected(rclcpp::Node::SharedPtr node, tf2_ros::Buffer::SharedPtr buffer);

  void originUpdated();
  void robotNamespaceUpdated(QString robot_namespace);
  void centerMap(QGeoCoordinate location);
  void startNode();

public slots:
  void nodeStarted(rclcpp::Node::SharedPtr node, tf2_ros::Buffer::SharedPtr buffer);
  void nodeShuttingDown();

    
    // void sendLookAt(QGeoCoordinate const &targetLocation);
    // void sendLookAtMode(std::string const &mode);
    void connectROS();

    // void watchdogUpdate();
    // void showTail(bool show);

    // void rangeAndBearingUpdate(double range, rclcpp::Time const &range_timestamp, double bearing, rclcpp::Time const &bearing_timestamp);
    
private:
    // void rangeCallback(const std_msgs::msg::Float32& message);
    // void bearingCallback(const std_msgs::msg::Float32& message);
    
    Ui::ROSLink* m_ui;

    AutonomousVehicleProject * m_autonomousVehicleProject =nullptr;

    QThread node_thread_;

    rclcpp::Node::SharedPtr node_;
    tf2_ros::Buffer::SharedPtr transform_buffer_;
    
    // ros::Subscriber m_range_subscriber;
    // ros::Subscriber m_bearing_subscriber;
    
    // ros::Publisher m_look_at_publisher;
    // ros::Publisher m_look_at_mode_publisher;
    
    
    // float m_base_dimension_to_stbd; 
    // float m_base_dimension_to_port;
    // float m_base_dimension_to_bow;
    // float m_base_dimension_to_stern; 

    // QList<QGeoCoordinate> m_current_path;
    // QList<QPointF> m_local_current_path;
    
    // bool m_show_tail;

    // QTimer * m_watchdog_timer;
    
    // double m_range;
    // rclcpp::Time m_range_timestamp;
    // double m_bearing;
    // rclcpp::Time m_bearing_timestamp;
    

    // tf2_ros::Buffer::SharedPtr m_tf_buffer;
    // tf2_ros::TransformListener::SharedPtr m_tf_listener;
    // std::string m_mapFrame;
};

#endif // ROSNODE_H
