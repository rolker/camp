#ifndef CAMP_AIS_CONTACT_H
#define CAMP_AIS_CONTACT_H

#include <QObject>
#include <cmath>
#include "../ship_track.h"
#include "marine_ais_msgs/msg/ais_contact.hpp"
#include "../locationposition.h"
#include "ros/ros_common.h"
#include "ros/ros_object.h"

struct AISContactDetails
{
  AISContactDetails();
  AISContactDetails(const marine_ais_msgs::msg::AISContact& message);
  uint32_t mmsi;
  std::string name;
  float dimension_to_stbd; 
  float dimension_to_port;
  float dimension_to_bow;
  float dimension_to_stern;
};

struct AISContactState
{
  AISContactState();
  AISContactState(const marine_ais_msgs::msg::AISContact& message);
  rclcpp::Time timestamp;
  LocationPosition location;
  // Initialised to NaN, the "unavailable" value the rest of the AIS path
  // already uses. None of these is guaranteed to be assigned: a contact that
  // is not moving never computes a course, and AIS reports SOG/COG as
  // unavailable often enough (the parser writes NaN into the twist) that the
  // motion vector is frequently NaN, which fails the > 0.0 test the same way.
  // Left uninitialised, the label rendered whatever was on the stack as a
  // course over ground.
  double heading = std::nan("");
  float cog = std::nan("");
  float sog = std::nan("");
};

struct AISReport: public QObject, AISContactDetails, AISContactState
{
  Q_OBJECT
public:
  AISReport(QObject *parent = nullptr);
  AISReport(const marine_ais_msgs::msg::AISContact& message, QObject *parent = nullptr);
};


class AISContact: public camp_ros::ROSObject, public ShipTrack, AISContactDetails
{
  Q_OBJECT

public:
  AISContact(QObject* parent = nullptr, QGraphicsItem *parentItem = nullptr);
  AISContact(AISReport* report, QObject* parent = nullptr, QGraphicsItem *parentItem = nullptr);
  ~AISContact();

  int type() const override {return AISContactType;}

  QRectF boundingRect() const override;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
  QPainterPath shape() const override;
  QPainterPath predictionShape() const;

  void newReport(AISReport* report);

public slots:
  void updateProjectedPoints();
  void updateView();

protected:
  void hoverEnterEvent(QGraphicsSceneHoverEvent * event) override;
  void hoverLeaveEvent(QGraphicsSceneHoverEvent * event) override;

private:
  void updateLabel();
  
  std::map<rclcpp::Time, AISContactState> m_states;
  rclcpp::Time m_displayTime;
};

#endif