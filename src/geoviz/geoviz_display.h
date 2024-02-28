#ifndef GEOVIZ_DISPLAY_H
#define GEOVIZ_DISPLAY_H

#include <QWidget>
#include <QColor>
#include "rclcpp/rclcpp.hpp"
#include "geographic_visualization_msgs/GeoVizItem.h"

#include "geographicsitem.h"
#include "locationposition.h"

namespace geoviz
{
  struct PointList
  {
    std::vector<LocationPosition> points;
    QColor color;
    float size;
  };

  struct Polygon
  {
    std::vector<LocationPosition> outer;
    std::vector<std::vector<LocationPosition> > inner;
    QPainterPath path;
    QColor fill_color;
    QColor edge_color;
    float edge_size;
  };

  struct Item: public QObject
  {
    Q_OBJECT
  public:
    std::string id;
    std::string label;
    LocationPosition label_position;
    std::vector<PointList> point_groups;
    std::vector<PointList> lines;
    std::vector<Polygon> polygons;
  };
}

class GeovizDisplay: public QWidget, public GeoGraphicsItem
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)

public:

  GeovizDisplay(QWidget* parent = nullptr, QGraphicsItem *parentItem = nullptr);

  int type() const override {return GeovizDisplayType;}

  QRectF boundingRect() const override;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
  QPainterPath shape() const override;
  QPainterPath shape(std::vector<LocationPosition> const &lps) const;
  
public slots:
  void updateRobotNamespace(QString robot_namespace);
  void updateDisplayItem(geoviz::Item *item);
  void updateProjectedPoints();


private:
  void geoVizDisplayCallback(const geographic_visualization_msgs::GeoVizItem::ConstPtr& message);

  ros::Subscriber m_display_subscriber;

  std::map<std::string,std::shared_ptr<geoviz::Item> > m_display_items;
};

#endif
