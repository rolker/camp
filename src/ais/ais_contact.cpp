#include "ais_contact.h"
#include "backgroundraster.h"
#include <QPainter>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

AISContactDetails::AISContactDetails()
{

}

AISContactDetails::AISContactDetails(const project11_msgs::msg::Contact& message)
{
  mmsi = message.mmsi;
  name = message.name;
  dimension_to_bow = message.dimension_to_bow;
  dimension_to_port = message.dimension_to_port;
  dimension_to_stbd = message.dimension_to_stbd;
  dimension_to_stern = message.dimension_to_stern;
}

AISContactDetails::AISContactDetails(const marine_ais_msgs::msg::AISContact& message)
{
  mmsi = message.id;
  name = message.static_info.name;
  dimension_to_bow = message.static_info.reference_to_bow_distance;
  dimension_to_port = message.static_info.reference_to_port_distance;
  dimension_to_stbd = message.static_info.reference_to_starboard_distance;
  dimension_to_stern = message.static_info.reference_to_stern_distance;
}


AISContactState::AISContactState()
{

}

AISContactState::AISContactState(const project11_msgs::msg::Contact& message)
{
  timestamp = message.header.stamp;
  location.location.setLatitude(message.position.latitude);
  location.location.setLongitude(message.position.longitude);
  if(message.heading < 0)
    if(message.sog > 0.25)
      heading = message.cog*180.0/M_PI;
    else
      heading = std::nan("");
  else
    heading = message.heading*180.0/M_PI;
  cog = message.cog*180.0/M_PI;
  sog = message.sog;
}

AISContactState::AISContactState(const marine_ais_msgs::msg::AISContact& message)
{
  timestamp = message.header.stamp;
  location.location.setLatitude(message.pose.position.latitude);
  location.location.setLongitude(message.pose.position.longitude);

  tf2::Vector3 motion;
  tf2::fromMsg(message.twist.twist.linear, motion);
  sog = motion.length();
  if (motion.length() > 0.0)
  {
    cog = -motion.angle(tf2::Vector3(0.0, 1.0, 0.0))*180/M_PI;
    if (cog < 0.0)
     cog += 360.0;
  }

  tf2::Quaternion orientation_quat;
  tf2::fromMsg(message.pose.orientation, orientation_quat);
  if (orientation_quat.length2() > 0.1) // make sure it's not a null orientation
  {
    double roll,pitch,yaw;
    tf2::getEulerYPR(orientation_quat, yaw, pitch, roll);
    heading = ((M_PI/2.0)-yaw)*180.0/M_PI;
  }
  else
  {
    if(sog > 0.25)
      heading = cog;
    else
      heading = std::nan("");
  }
}

AISReport::AISReport(QObject *parent):QObject(parent)
{

}

AISReport::AISReport(const project11_msgs::msg::Contact& message, QObject *parent):
  QObject(parent),
  AISContactDetails(message),
  AISContactState(message)
{

}

AISReport::AISReport(const marine_ais_msgs::msg::AISContact& message, QObject *parent):
  QObject(parent),
  AISContactDetails(message),
  AISContactState(message)
{

}


AISContact::AISContact(QObject *parent, QGraphicsItem *parentItem)
  :camp_ros::ROSObject(parent), ShipTrack(parentItem)
{
  setAcceptHoverEvents(true);
}

AISContact::AISContact(AISReport* report, QObject *parent, QGraphicsItem *parentItem):
  camp_ros::ROSObject(parent),
  ShipTrack(parentItem),
  AISContactDetails(*report)
{
  setAcceptHoverEvents(true);
}

AISContact::~AISContact()
{

}

void AISContact::updateView()
{
  if(node_)
  {
    prepareGeometryChange();
    m_displayTime = node_->get_clock()->now();
    update();
  }
}

void AISContact::newReport(AISReport *report)
{
  mmsi = report->mmsi;
  if (!report->name.empty())
    name = report->name;

  dimension_to_bow = report->dimension_to_bow;
  dimension_to_port = report->dimension_to_port;
  dimension_to_stbd = report->dimension_to_stbd;
  dimension_to_stern = report->dimension_to_stern;
  m_states[report->timestamp] = *report;
  BackgroundRaster* bg = findParentBackgroundRaster();
  if(bg)
  {
    m_states[report->timestamp].location.pos = geoToPixel(report->location.location, bg);
    setLabelPosition(m_states[report->timestamp].location.pos);
  }
}

void AISContact::updateLabel()
{
  QString label;
  if(!name.empty())
    label = name.c_str();
  else
    label = QString::number(mmsi);

  if(!m_states.empty())
  {
    label += "\nsog: " + QString::number(int(m_states.rbegin()->second.sog*10)/10.0) + " m/s";
    label += "\ncog: " + QString::number(int(m_states.rbegin()->second.cog));
  }
  setLabel(label);
}
  


void AISContact::updateProjectedPoints()
{
  BackgroundRaster* bg = dynamic_cast<BackgroundRaster*>(parentItem());
  for (auto& s: m_states)
    s.second.location.pos = geoToPixel(s.second.location.location, bg);
  if (!m_states.empty())
    setLabelPosition(m_states.rbegin()->second.location.pos);
}

QRectF AISContact::boundingRect() const
{
  return (shape().boundingRect()|predictionShape().boundingRect()).marginsAdded(QMargins(2,2,2,2));
}

void AISContact::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
  painter->save();

  QPen p;
  p.setCosmetic(true);
  //p.setColor(Qt::blue);
  p.setColor(QColor(.2*255,.2*255,255,.7*255));
  p.setWidth(2);
  painter->setPen(p);
  painter->drawPath(shape());

  p.setColor(QColor(128, 128, 128, 128));
  
  painter->setPen(p);
  painter->drawPath(predictionShape());

  painter->restore();
}

QPainterPath AISContact::shape() const
{
  QPainterPath ret;
  // History length should be configurable and displayTime could be set
  // somwhere else to support rewinding time
  if(m_displayTime.seconds() != 0)
  {
    auto historyStartTime = m_displayTime - rclcpp::Duration::from_seconds(300);

    auto state = m_states.lower_bound(historyStartTime);
    if(state != m_states.end() && state->first <= m_displayTime)
    {
      ret.moveTo(state->second.location.pos);
      auto nextState = state;
      nextState++;
      while(nextState != m_states.end())
      {
        if(nextState->first <= m_displayTime)
        {
          state = nextState;
          nextState++;
          ret.lineTo(state->second.location.pos);
        }
        else
          break;
      }

      BackgroundRaster* bg = findParentBackgroundRaster();
      if(bg)
      {
        bool forceTriangle = false;
        if (dimension_to_bow + dimension_to_stern == 0 || dimension_to_port + dimension_to_stbd == 0)
          forceTriangle = true;
        float max_size = std::max(dimension_to_bow + dimension_to_stern, dimension_to_port + dimension_to_stbd);
        qreal pixel_size = bg->scaledPixelSize();
        if(pixel_size > max_size/10.0 || forceTriangle)
          drawTriangle(ret, bg, state->second.location.location, state->second.heading, pixel_size);
        else
          drawShipOutline(ret, bg, state->second.location.location, state->second.heading, dimension_to_bow, dimension_to_port, dimension_to_stbd, dimension_to_stern);      
      }

    }
  }
  return ret;
}

QPainterPath AISContact::predictionShape() const
{
  QPainterPath ret;
  if(m_displayTime.seconds() != 0)
  {
    auto state = m_states.rbegin();
    while(state != m_states.rend() && state->first > m_displayTime)
      state++;
    if(state != m_states.rend())
    {
      ret.moveTo(state->second.location.pos);
      QGeoCoordinate futureLocation = state->second.location.location.atDistanceAndAzimuth(state->second.sog*300, state->second.cog);
      auto timeSinceReport = m_displayTime - state->first;
      QGeoCoordinate predicatedLocation = state->second.location.location.atDistanceAndAzimuth(state->second.sog*timeSinceReport.seconds(), state->second.cog);
      BackgroundRaster* bg = findParentBackgroundRaster();
      if(bg)
      {
        ret.lineTo(geoToPixel(futureLocation, bg));

        bool forceTriangle = false;
        if (dimension_to_bow + dimension_to_stern == 0 || dimension_to_port + dimension_to_stbd == 0)
          forceTriangle = true;
        float max_size = std::max(dimension_to_bow + dimension_to_stern, dimension_to_port + dimension_to_stbd);
        qreal pixel_size = bg->scaledPixelSize();
        if(pixel_size > max_size/10.0 || forceTriangle)
          drawTriangle(ret, bg, predicatedLocation, state->second.heading, pixel_size);
        else
          drawShipOutline(ret, bg, predicatedLocation, state->second.heading, dimension_to_bow, dimension_to_port, dimension_to_stbd, dimension_to_stern);      
      }
    }
  }
  return ret;
}

void AISContact::hoverEnterEvent(QGraphicsSceneHoverEvent* event)
{
  updateLabel();
  setShowLabelFlag(true);
}

void AISContact::hoverLeaveEvent(QGraphicsSceneHoverEvent* event)
{
  setShowLabelFlag(false);
}
