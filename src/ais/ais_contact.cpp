#include "ais_contact.h"
#include "backgroundraster.h"
#include <QPainter>

AISContactDetails::AISContactDetails()
{

}

AISContactDetails::AISContactDetails(const marine_msgs::Contact::ConstPtr& message)
{
  mmsi = message->mmsi;
  name = message->name;
  dimension_to_bow = message->dimension_to_bow;
  dimension_to_port = message->dimension_to_port;
  dimension_to_stbd = message->dimension_to_stbd;
  dimension_to_stern = message->dimension_to_stern;
}

AISContactState::AISContactState()
{

}

AISContactState::AISContactState(const marine_msgs::Contact::ConstPtr& message)
{
  timestamp = message->header.stamp;
  location.location.setLatitude(message->position.latitude);
  location.location.setLongitude(message->position.longitude);
  if(message->heading < 0)
    heading = message->cog*180.0/M_PI;
  else
    heading = message->heading*180.0/M_PI;
  cog = message->cog*180.0/M_PI;
  sog = message->sog;
}

AISReport::AISReport(QObject *parent):QObject(parent)
{

}

AISReport::AISReport(const marine_msgs::Contact::ConstPtr& message, QObject *parent):
  QObject(parent),
  AISContactDetails(message),
  AISContactState(message)
{

}

AISContact::AISContact(QObject *parent, QGraphicsItem *parentItem):QObject(parent), ShipTrack(parentItem)
{
  setAcceptHoverEvents(true);
}

AISContact::AISContact(AISReport* report, QObject *parent, QGraphicsItem *parentItem):
  QObject(parent),
  ShipTrack(parentItem),
  AISContactDetails(*report)
{
  setAcceptHoverEvents(true);
}

AISContact::~AISContact()
{

}

void AISContact::newReport(AISReport *report)
{
  prepareGeometryChange();
  mmsi = report->mmsi;
  name = report->name;
  if(!name.empty())
    setLabel(name.c_str());
  else
    setLabel(QString::number(mmsi));

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
  p.setColor(Qt::blue);
  p.setWidth(2);
  painter->setPen(p);
  painter->drawPath(shape());

  p.setColor(Qt::red);
  painter->setPen(p);
  painter->drawPath(predictionShape());

  painter->restore();
}

QPainterPath AISContact::shape() const
{
  QPainterPath ret;
  // History length should be configurable and displayTime could be set
  // somwhere else to support rewinding time
  ros::Time displayTime = ros::Time::now();
  ros::Time historyStartTime = displayTime - ros::Duration(300);

  auto state = m_states.lower_bound(historyStartTime);
  if(state != m_states.end() && state->first <= displayTime)
  {
    ret.moveTo(state->second.location.pos);
    auto nextState = state;
    nextState++;
    while(nextState != m_states.end())
    {
      if(nextState->first <= displayTime)
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
  return ret;
}

QPainterPath AISContact::predictionShape() const
{
  QPainterPath ret;
  ros::Time displayTime = ros::Time::now();
  auto state = m_states.rbegin();
  while(state != m_states.rend() && state->first > displayTime)
    state++;
  if(state != m_states.rend())
  {
    ret.moveTo(state->second.location.pos);
    QGeoCoordinate futureLocation = state->second.location.location.atDistanceAndAzimuth(state->second.sog*300, state->second.cog);
    BackgroundRaster* bg = findParentBackgroundRaster();
    if(bg)
    {
      ret.lineTo(geoToPixel(futureLocation, bg));
    }
  }
  return ret;
}

void AISContact::hoverEnterEvent(QGraphicsSceneHoverEvent* event)
{
  setShowLabelFlag(true);

}

void AISContact::hoverLeaveEvent(QGraphicsSceneHoverEvent* event)
{
  setShowLabelFlag(false);
  
}