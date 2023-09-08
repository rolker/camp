#include "nav_source.h"
#include <tf2/utils.h>
#include <QPainter>
#include <QTimer>
#include <QDebug>

NavSource::NavSource(const project11_msgs::NavSource& source, QObject* parent, QGraphicsItem *parentItem): QObject(parent), GeoGraphicsItem(parentItem)
{
  pending_position_topic_ = source.position_topic;
  pending_orientation_topic_ = source.orientation_topic;
  pending_velocity_topic_ = source.velocity_topic;
  setObjectName(source.name.c_str());
  trySubscribe();
}

NavSource::NavSource(std::pair<const std::string, XmlRpc::XmlRpcValue> &source, QObject* parent, QGraphicsItem *parentItem): QObject(parent), GeoGraphicsItem(parentItem)
{
  if (source.second.hasMember("position_topic"))
    pending_position_topic_ = std::string(source.second["position_topic"]);
  if (source.second.hasMember("orientation_topic"))
    pending_orientation_topic_ = std::string(source.second["orientation_topic"]);
  if (source.second.hasMember("velocity_topic"))
    pending_velocity_topic_ = std::string(source.second["velocity_topic"]);
  setObjectName(source.first.c_str());
  trySubscribe();
}

void NavSource::trySubscribe()
{
  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);
  ros::NodeHandle nh;

  if(!pending_position_topic_.empty())
  {
    for(const auto t: topic_info)
      if(t.name == pending_position_topic_)
      {
        if(t.datatype == "sensor_msgs/NavSatFix")
        {
          m_position_sub = nh.subscribe(pending_position_topic_, 1, &NavSource::positionCallback, this);
          pending_position_topic_.clear();
        }
        else if(t.datatype == "geographic_msgs/GeoPoseStamped")
        {
          m_position_sub = nh.subscribe(pending_position_topic_, 1, &NavSource::geoPoseCallback, this);
          pending_position_topic_.clear();
        }
        break;
      }
  }
  if(!pending_orientation_topic_.empty())
  {
    for(const auto t: topic_info)
      if(t.name == pending_orientation_topic_)
      {
        if(t.datatype == "sensor_msgs/Imu")
        {
          m_orientation_sub = nh.subscribe(pending_orientation_topic_, 1, &NavSource::orientationCallback, this);
          pending_orientation_topic_.clear();
        }
        break;
      }

  }
  if(!pending_velocity_topic_.empty())
  {
    for(const auto t: topic_info)
      if(t.name == pending_velocity_topic_)
      {
        if(t.datatype == "geometry_msgs/TwistWithCovarianceStamped")
        {
          m_velocity_sub = nh.subscribe(pending_velocity_topic_, 1, &NavSource::velocityCallback, this);
          pending_velocity_topic_.clear();
        }
        break;
      }
  }

  if(!pending_position_topic_.empty() || !pending_orientation_topic_.empty() || !pending_velocity_topic_.empty())
    QTimer::singleShot(1000, this, &NavSource::trySubscribe);

}


QRectF NavSource::boundingRect() const
{
  return shape().boundingRect().marginsAdded(QMargins(2,2,2,2));
}

void NavSource::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
  painter->save();

  QPen p;
  p.setCosmetic(true);
  p.setColor(m_color);
  p.setWidth(2);
  painter->setPen(p);
  painter->drawPath(shape());

  painter->restore();
}

QPainterPath NavSource::shape() const
{
  QPainterPath ret;
  if(!m_location_history.empty())
  {
    ret.moveTo(m_location_history.front().pos);
    for (auto location: m_location_history)
      ret.lineTo(location.pos);
  }
  return ret;
}

void NavSource::positionCallback(const sensor_msgs::NavSatFix::ConstPtr& message)
{
  QGeoCoordinate position(message->latitude, message->longitude, message->altitude);
  QMetaObject::invokeMethod(this,"updateLocation", Qt::QueuedConnection, Q_ARG(QGeoCoordinate, position));
}

void NavSource::geoPointCallback(const geographic_msgs::GeoPointStamped::ConstPtr& message)
{
  QGeoCoordinate position(message->position.latitude, message->position.longitude, message->position.altitude);
  QMetaObject::invokeMethod(this,"updateLocation", Qt::QueuedConnection, Q_ARG(QGeoCoordinate, position));
}

void NavSource::geoPoseCallback(const geographic_msgs::GeoPoseStamped::ConstPtr& message)
{
  QGeoCoordinate position(message->pose.position.latitude, message->pose.position.longitude, message->pose.position.altitude);
  QMetaObject::invokeMethod(this,"updateLocation", Qt::QueuedConnection, Q_ARG(QGeoCoordinate, position));
  double yaw = std::nan("");
  if(message->pose.orientation.w != 0 ||  message->pose.orientation.x != 0 || message->pose.orientation.y != 0 || message->pose.orientation.z != 0)
    yaw = tf2::getYaw(message->pose.orientation);
  double heading = 90-180*yaw/M_PI;
  QMetaObject::invokeMethod(this,"updateHeading", Qt::QueuedConnection, Q_ARG(double, heading));
}


void NavSource::orientationCallback(const sensor_msgs::Imu::ConstPtr& message)
{
  double yaw = tf2::getYaw(message->orientation);
  double heading = 90-180*yaw/M_PI;
  QMetaObject::invokeMethod(this,"updateHeading", Qt::QueuedConnection, Q_ARG(double, heading));
}

void NavSource::velocityCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& message)
{
  QMetaObject::invokeMethod(this,"updateSog", Qt::QueuedConnection, Q_ARG(double, sqrt(pow(message->twist.twist.linear.x,2) + pow(message->twist.twist.linear.y, 2))));
}


void NavSource::updateSog(double sog)
{
  emit NavSource::sog(sog);
}

void NavSource::updateLocation(QGeoCoordinate const &location)
{
  emit beforeNavUpdate();
  prepareGeometryChange();
  LocationPosition lp;
  lp.location = location;
  auto bg = findParentBackgroundRaster();
  if(bg)
    lp.pos = geoToPixel(location, bg);
  m_location_history.push_back(lp);
  m_location = lp;
  while(m_max_history > 0 && m_location_history.size() > m_max_history)
    m_location_history.pop_front();
  emit positionUpdate(location);
}

void NavSource::updateHeading(double heading)
{
  emit beforeNavUpdate();
  prepareGeometryChange();
  m_heading = heading;
}

void NavSource::updateProjectedPoints()
{
  prepareGeometryChange();
  auto bg = findParentBackgroundRaster();
  if(bg)
    for(auto& lp: m_location_history)
      lp.pos = geoToPixel(lp.location, bg);
}

const LocationPosition& NavSource::location() const
{
  return m_location;
}

double NavSource::heading() const
{
  return m_heading;
}

void NavSource::setMaxHistory(int max_history)
{
  m_max_history = max_history;
}

void NavSource::setColor(QColor color)
{
  m_color = color;
}
