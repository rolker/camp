#include "nav_source.h"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <QPainter>
#include <QTimer>
#include <QDebug>

NavSource::NavSource(const project11_msgs::msg::NavSource& source, QObject* parent, QGraphicsItem *parentItem): camp_ros::ROSObject(parent), GeoGraphicsItem(parentItem)
{
  setColor(color_);
  pending_position_topic_ = source.position_topic;
  pending_orientation_topic_ = source.orientation_topic;
  pending_velocity_topic_ = source.velocity_topic;
  setObjectName(source.name.c_str());
  trySubscribe();
}


void NavSource::trySubscribe()
{
  if(node_)
  {
    auto topics = node_->get_topic_names_and_types();
    for(auto topic: topics)
    {
      auto name = topic.first;
      
      if(!pending_position_topic_.empty() && name == pending_position_topic_)
      {
        for (auto topic_type: topic.second)
        {
          if(topic_type == "sensor_msgs/NavSatFix")
          {
            position_subscription_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(name, 1, std::bind(&NavSource::positionCallback, this, std::placeholders::_1));
            pending_position_topic_.clear();
          }
          else if(topic_type == "geographic_msgs/GeoPoseStamped")
          {
            geo_pose_subscription_ = node_->create_subscription<geographic_msgs::msg::GeoPoseStamped>(name, 1, std::bind(&NavSource::geoPoseCallback, this, std::placeholders::_1));
            pending_position_topic_.clear();
          }
        }
      }
      
      if(!pending_orientation_topic_.empty() && name == pending_orientation_topic_)
      {
        for (auto topic_type: topic.second)
        {
          if(topic_type == "sensor_msgs/Imu")
          {
            orientation_subscription_ = node_->create_subscription<sensor_msgs::msg::Imu>(name, 1, std::bind(&NavSource::orientationCallback, this, std::placeholders::_1));
            pending_orientation_topic_.clear();
          }
        }
      }

      if(!pending_velocity_topic_.empty() && name == pending_velocity_topic_)
      {
        for (auto topic_type: topic.second)
        {
          if(topic_type == "geometry_msgs/TwistWithCovarianceStamped")
          {
            velocity_subscription_ = node_->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(name, 1, std::bind(&NavSource::velocityCallback, this, std::placeholders::_1));
            pending_velocity_topic_.clear();
          }
        }
      }
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

  QPen p1;
  p1.setCosmetic(true);
  p1.setColor(color_);
  p1.setWidth(2);
  painter->setPen(p1);

  auto s = shapes();

  auto l = location();
  painter->drawPath(s.first);

  QPen p2;
  p2.setCosmetic(true);
  p2.setColor(dim_color_);
  p2.setWidth(1);
  painter->setPen(p2);

  painter->drawPath(s.second);

  painter->restore();
}

QPainterPath NavSource::shape() const
{
  QPainterPath ret;
  auto s = shapes();
  ret.addPath(s.first);
  ret.addPath(s.second);
  return ret;
}

std::pair<QPainterPath, QPainterPath> NavSource::shapes() const
{
  auto paths = std::make_pair<QPainterPath,QPainterPath>({},{});
  auto location_iterator = location_history_.rbegin();
  auto last_point = location_history_.rend();
  if(location_iterator != location_history_.rend())
  {
    double latest_time = location_iterator->first;
    bool first_point = true;
    while(location_iterator != location_history_.rend())
    {
      if(location_iterator->first < latest_time-60.0)
        break;
      if(location_iterator->second.location.isValid())
      {
        if(first_point)
        {
          paths.first.moveTo(location_iterator->second.pos);
          first_point = false;
        }
        else
        {
          paths.first.lineTo(location_iterator->second.pos);
          last_point = location_iterator;
        }
      }
      location_iterator++;
    }
    if(last_point != location_history_.rend())
    {
      paths.second.moveTo(last_point->second.pos);
      first_point = false;
    }
    else
      first_point = true;
    while(location_iterator != location_history_.rend())
    {
      if(location_iterator->second.location.isValid())
      {
        if(first_point)
        {
          paths.second.moveTo(location_iterator->second.pos);
          first_point = false;
        }
        else
        {
          paths.second.lineTo(location_iterator->second.pos);
        }
      }
      location_iterator++;
    }
  }
  return paths;
}


void NavSource::positionCallback(const sensor_msgs::msg::NavSatFix& message)
{
  if(message.status.status != sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX)
  {
    QGeoCoordinate position(message.latitude, message.longitude, message.altitude);
    QMetaObject::invokeMethod(this,"updateLocation", Qt::QueuedConnection, Q_ARG(QGeoCoordinate, position), Q_ARG(float, std::nan("")), Q_ARG(double, rclcpp::Time(message.header.stamp).seconds()));
  }
}

void NavSource::geoPointCallback(const geographic_msgs::msg::GeoPointStamped& message)
{
  QGeoCoordinate position(message.position.latitude, message.position.longitude, message.position.altitude);
  QMetaObject::invokeMethod(this,"updateLocation", Qt::QueuedConnection, Q_ARG(QGeoCoordinate, position), Q_ARG(float, std::nan("")), Q_ARG(double, rclcpp::Time(message.header.stamp).seconds()));
}

void NavSource::geoPoseCallback(const geographic_msgs::msg::GeoPoseStamped& message)
{
  QGeoCoordinate position(message.pose.position.latitude, message.pose.position.longitude, message.pose.position.altitude);
  double yaw = std::nan("");
  if(message.pose.orientation.w != 0 ||  message.pose.orientation.x != 0 || message.pose.orientation.y != 0 || message.pose.orientation.z != 0)
    yaw = tf2::getYaw(message.pose.orientation);
  double heading = 90-180*yaw/M_PI;
  if(heading < 0.0)
    heading += 360.0;
  QMetaObject::invokeMethod(this,"updateLocation", Qt::QueuedConnection, Q_ARG(QGeoCoordinate, position), Q_ARG(float, heading), Q_ARG(double, rclcpp::Time(message.header.stamp).seconds()));
}


void NavSource::orientationCallback(const sensor_msgs::msg::Imu& message)
{
  double yaw = tf2::getYaw(message.orientation);
  double heading = 90-180*yaw/M_PI;
  if(heading < 0.0)
    heading += 360.0;
  QMetaObject::invokeMethod(this,"updateLocation", Qt::QueuedConnection, Q_ARG(QGeoCoordinate, QGeoCoordinate()), Q_ARG(float, heading), Q_ARG(double, rclcpp::Time(message.header.stamp).seconds()));
}

void NavSource::velocityCallback(const geometry_msgs::msg::TwistWithCovarianceStamped& message)
{
  QMetaObject::invokeMethod(this,"updateSog", Qt::QueuedConnection, Q_ARG(double, sqrt(pow(message.twist.twist.linear.x,2) + pow(message.twist.twist.linear.y, 2))));
}


void NavSource::updateSog(double sog)
{
  emit NavSource::sog(sog);
}

void NavSource::updateLocation(QGeoCoordinate const &location, float heading, double time)
{
  emit beforeNavUpdate();
  prepareGeometryChange();

  location_history_[time].time = time;
  if(location.isValid())
  {
    location_history_[time].location = location;
    auto bg = findParentBackgroundRaster();
    if(bg)
      location_history_[time].pos = geoToPixel(location, bg);
  }
  if(!isnan(heading))
    location_history_[time].heading = heading;

  // trim samples that are older than buffer durration, if applicable.
  while(buffer_duration_ > 0.0 && !location_history_.empty() && location_history_.rbegin()->first-buffer_duration_ > location_history_.begin()->first)
    location_history_.erase(location_history_.begin()->first);

  // thin older data
  if(!location_history_.empty())
  {
    double low_res_end_time = location_history_.rbegin()->first - high_resolution_duration_;
    std::vector<double> discard_pile;
    auto location_iterator = location_history_.begin();
    double last_keep_time = location_iterator->first;
    location_iterator++;
    while(location_iterator != location_history_.end() && location_iterator->first < low_res_end_time)
    {
      if(location_iterator->first >= last_keep_time+low_resolution_period_)
      {
        last_keep_time = location_iterator->first;
      }
      else
      {
        discard_pile.push_back(location_iterator->first);
      }
      location_iterator++;
    }

    for(auto discard: discard_pile)
      location_history_.erase(discard);
  }


  if(location.isValid())
    emit positionUpdate(location);
}

void NavSource::updateProjectedPoints()
{
  prepareGeometryChange();
  auto bg = findParentBackgroundRaster();
  if(bg)
    for(auto& lp: location_history_)
      lp.second.pos = geoToPixel(lp.second.location, bg);
}

LocationPositionHeadingTime NavSource::location() const
{
  auto location_iterator = location_history_.rbegin();
  while(location_iterator != location_history_.rend())
    if(location_iterator->second.location.isValid())
      return location_iterator->second;
    else
      location_iterator++;
  return {};
}

LocationPositionHeadingTime NavSource::heading() const
{
  auto location_iterator = location_history_.rbegin();
  while(location_iterator != location_history_.rend())
    if(!isnan(location_iterator->second.heading))
      return location_iterator->second;
    else
      location_iterator++;
  return {};
}

void NavSource::setHistoryDuration(double duration)
{
  buffer_duration_ = duration;
}

void NavSource::setColor(QColor color)
{
  color_ = color;
  dim_color_ = color;
  dim_color_.setAlphaF(color.alphaF()*.5);
}
