#ifndef NAV_SOURCE_H
#define NAV_SOURCE_H

#include "geographicsitem.h"
#include "ros/ros.h"
#include "project11_msgs/NavSource.h"
#include "locationposition.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"

class NavSource: public QObject, public GeoGraphicsItem
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  NavSource(const project11_msgs::NavSource& source, QObject* parent, QGraphicsItem *parentItem = nullptr);

  int type() const override {return NavSourceType;}

  QRectF boundingRect() const override;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
  QPainterPath shape() const override;

  const LocationPosition& location() const;
  double heading() const;

signals:
  void beforeNavUpdate();

public slots:
  void updateLocation(QGeoCoordinate const &location);
  void updateHeading(double heading);
  void updateProjectedPoints();

private:
  void positionCallback(const sensor_msgs::NavSatFix::ConstPtr& message);
  void orientationCallback(const sensor_msgs::Imu::ConstPtr& message);

  ros::Subscriber m_position_sub;
  ros::Subscriber m_orientation_sub;

  LocationPosition m_location;
  double m_heading;
  std::list<LocationPosition> m_location_history;
};

#endif