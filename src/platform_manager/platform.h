#ifndef PLATFORM_H
#define PLATFORM_H

#include <QWidget>
#include "ship_track.h"
#include "ros/ros.h"
#include "project11_msgs/PlatformList.h"

namespace Ui
{
  class Platform;
}

class NavSource;
class MissionManager;
class HelmManager;

class Platform : public QWidget, public ShipTrack
{
  Q_OBJECT
public:
  explicit Platform(QWidget* parent = nullptr, QGraphicsItem *parentItem = nullptr);
  ~Platform();

  int type() const override {return PlatformType;}

  QRectF boundingRect() const override;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
  QPainterPath shape() const override;

  void update(project11_msgs::Platform &platform);
  void update(std::pair<const std::string, XmlRpc::XmlRpcValue> &platform);

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

  Ui::Platform* m_ui;


  std::map<std::string, NavSource*> m_nav_sources;

  ros::Subscriber m_sog_subscriber;
  QList<qreal> m_sog_history;
  qreal m_sog;
  qreal m_sog_avg;

  float m_width = 0.0;
  float m_length = 0.0;
  float m_reference_x = 0.0;
  float m_reference_y = 0.0;

};

#endif // PLATFORM_H
