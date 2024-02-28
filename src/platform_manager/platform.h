#ifndef PLATFORM_H
#define PLATFORM_H

#include "ros/ros_widget.h"
#include "ship_track.h"
#include "project11_msgs/msg/platform_list.hpp"

namespace Ui
{
  class Platform;
}

class NavSource;
class MissionManager;
class HelmManager;

class Platform : public camp_ros::ROSWidget, public ShipTrack
{
  Q_OBJECT
public:
  explicit Platform(QWidget* parent = nullptr, QGraphicsItem *parentItem = nullptr);
  ~Platform();

  int type() const override {return PlatformType;}

  QRectF boundingRect() const override;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
  QPainterPath shape() const override;

  void update(const project11_msgs::msg::Platform &platform);

  MissionManager* missionManager() const;
  HelmManager* helmManager() const;

signals:
  void platformPosition(Platform* platform, QGeoCoordinate position);


public slots:
  void updateProjectedPoints();
  void aboutToUpdateNav();
  void updateSog(double sog);
  void updatePosition(QGeoCoordinate position);

protected:
  void hoverEnterEvent(QGraphicsSceneHoverEvent * event) override;
  void hoverLeaveEvent(QGraphicsSceneHoverEvent * event) override;

private:
  void updateLabel();
  void setColor(QColor color);

  Ui::Platform* m_ui;


  std::map<std::string, NavSource*> m_nav_sources;

  QList<qreal> m_sog_history;
  qreal m_sog;
  qreal m_sog_avg;

  float m_width = 0.0;
  float m_length = 0.0;
  float m_reference_x = 0.0;
  float m_reference_y = 0.0;

  QColor m_color = QColor(0,0,255,255);

};

#endif // PLATFORM_H
