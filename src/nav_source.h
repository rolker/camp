#ifndef NAV_SOURCE_H
#define NAV_SOURCE_H

#include "geographicsitem.h"
#include "ros/ros.h"
#include "project11_msgs/NavSource.h"
#include "locationposition.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "geographic_msgs/GeoPointStamped.h"
#include "geographic_msgs/GeoPoseStamped.h"

class NavSource: public QObject, public GeoGraphicsItem
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  NavSource(const project11_msgs::NavSource& source, QObject* parent, QGraphicsItem *parentItem = nullptr);
  NavSource(std::pair<const std::string, XmlRpc::XmlRpcValue> &source, QObject* parent, QGraphicsItem *parentItem = nullptr);

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
  void positionCallback(const sensor_msgs::NavSatFix::ConstPtr& message);
  void orientationCallback(const sensor_msgs::Imu::ConstPtr& message);
  void velocityCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& message);
  void geoPointCallback(const geographic_msgs::GeoPointStamped::ConstPtr& message);
  void geoPoseCallback(const geographic_msgs::GeoPoseStamped::ConstPtr& message);

  ros::Subscriber m_position_sub;
  ros::Subscriber m_orientation_sub;
  ros::Subscriber m_velocity_sub;

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