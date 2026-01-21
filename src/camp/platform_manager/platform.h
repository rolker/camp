#ifndef PLATFORM_H
#define PLATFORM_H

#include "ros/ros_widget.h"
#include "ship_track.h"
#include "marine_interfaces/msg/platform_list.hpp"
#include "nav_msgs/msg/path.hpp"

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

  void update(const marine_interfaces::msg::Platform &platform);

  MissionManager* missionManager() const;
  HelmManager* helmManager() const;

  void onNodeUpdated() override;

signals:
  void platformPosition(Platform* platform, QGeoCoordinate position);
  void pathUpdated(std::vector<QPointF> path_local_points);


public slots:
  void updateProjectedPoints();
  void aboutToUpdateNav();
  void updateSog(double sog);
  void updatePosition(QGeoCoordinate position);
  void updatePath(std::vector<QPointF> path_local_points);

protected:
  void hoverEnterEvent(QGraphicsSceneHoverEvent * event) override;
  void hoverLeaveEvent(QGraphicsSceneHoverEvent * event) override;

private:
  void updateLabel();
  void setColor(QColor color);
  void subscribeToPathTopic();
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);


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

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  std::string path_topic_;
  nav_msgs::msg::Path path_;
  std::vector<QGeoCoordinate> path_geopoints_;
  std::vector<QPointF> path_local_points_;

};

#endif // PLATFORM_H
