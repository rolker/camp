#ifndef ROSNODE_H
#define ROSNODE_H

#include "geographicsmissionitem.h"

#include "asv_msgs/BasicPositionStamped.h"
#include "asv_msgs/HeadingStamped.h"
#include "ros/ros.h"

class ROSNode : public GeoGraphicsMissionItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    ROSNode(QObject* parent, QGraphicsItem* parentItem);
    
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    QPainterPath shape() const override;
    
    void write(QJsonObject &json) const;
    void read(const QJsonObject &json);
    
    int type() const {return ROSNodeType;}
    
public slots:
    void updateLocation(QGeoCoordinate const &location);
    void updateProjectedPoints();
    
private:
    void positionCallback(const asv_msgs::BasicPositionStamped::ConstPtr& message);
    void headingCallback(const asv_msgs::HeadingStamped::ConstPtr& message);
    ros::NodeHandle m_node;
    ros::Subscriber m_position_subscriber;
    ros::Subscriber m_heading_subscriber;
    ros::AsyncSpinner m_spinner;
    QGeoCoordinate m_location;
    std::vector<QGeoCoordinate> m_location_history;
    std::vector<QPointF> m_local_location_history;
    QPointF m_local_reference_position;
    double m_heading;

};

#endif // ROSNODE_H
