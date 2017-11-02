#ifndef ROSNODE_H
#define ROSNODE_H

#include "geographicsmissionitem.h"

#include "asv_msgs/BasicPositionStamped.h"
#include "ros/ros.h"

class ROSNode : public GeoGraphicsMissionItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    ROSNode(QObject* parent, QGraphicsItem* parentItem);
    
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    
    void write(QJsonObject &json) const;
    void read(const QJsonObject &json);
    
    int type() const {return ROSNodeType;}
    
public slots:
    void updateLocation(QGeoCoordinate const &location);
    void updateProjectedPoints();
    
private:
    void positionCallback(const asv_msgs::BasicPositionStamped::ConstPtr& message);
    ros::NodeHandle node;
    ros::Subscriber subscriber;
    ros::AsyncSpinner spinner;
    QGeoCoordinate m_location;

};

#endif // ROSNODE_H
