#ifndef NAV_SOURCE_H
#define NAV_SOURCE_H

#include "ros/ros_object.h"
#include "geographicsitem.h"
#include "project11_msgs/msg/nav_source.hpp"
#include "locationposition.h"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geographic_msgs/msg/geo_point_stamped.hpp"
#include "geographic_msgs/msg/geo_pose_stamped.hpp"

class NavSource: public camp_ros::ROSObject, public GeoGraphicsItem
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  NavSource(const project11_msgs::msg::NavSource& source, QObject* parent, QGraphicsItem *parentItem = nullptr);

  int type() const override {return NavSourceType;}

  QRectF boundingRect() const override;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
  QPainterPath shape() const override;

  std::pair<QPainterPath, QPainterPath> shapes() const;

  LocationPositionHeadingTime location() const;
  LocationPositionHeadingTime heading() const;

  void setColor(QColor color);

signals:
  void beforeNavUpdate();
  void sog(double sog);
  void positionUpdate(QGeoCoordinate position);

public slots:
  void updateLocation(QGeoCoordinate const &location, float heading, double time);
  void updateProjectedPoints();

  /// Set buffer duration in seconds 
  void setHistoryDuration(double duration);
  void updateSog(double sog);
  void trySubscribe();

private:
  void positionCallback(const sensor_msgs::msg::NavSatFix& message);
  void orientationCallback(const sensor_msgs::msg::Imu& message);
  void velocityCallback(const geometry_msgs::msg::TwistWithCovarianceStamped& message);
  void geoPointCallback(const geographic_msgs::msg::GeoPointStamped& message);
  void geoPoseCallback(const geographic_msgs::msg::GeoPoseStamped& message);

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr position_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orientation_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_subscription_;
  rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr geo_point_subscription_;
  rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr geo_pose_subscription_;


  std::string pending_position_topic_;
  std::string pending_orientation_topic_;
  std::string pending_velocity_topic_;

  std::map<double, LocationPositionHeadingTime> location_history_;

  /// How long data should be kept in seconds. Forever if 0.
  double buffer_duration_ = 0.0;

  /// How long to keep full rate data in seconds.
  double high_resolution_duration_ = 60.0;

  /// Low resolution period in seconds between samples
  double low_resolution_period_ = 1.0;
  
  QColor color_ = Qt::red;
  QColor dim_color_;


};

#endif