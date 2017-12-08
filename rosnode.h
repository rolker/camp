#ifndef ROSNODE_H
#define ROSNODE_H

#include "geographicsmissionitem.h"

#include "geographic_msgs/GeoPointStamped.h"
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
    
    bool active() const;
    void setActive(bool active);
    
    std::string const &helmMode() const;
    void setHelmMode(const std::string& helmMode);
    
public slots:
    void updateLocation(QGeoCoordinate const &location);
    void updateProjectedPoints();
    void sendWaypoints(QList<QGeoCoordinate> const &waypoints);
    
private:
    void geoPointStampedCallback(const geographic_msgs::GeoPointStamped::ConstPtr& message); 
    void originCallback(const geographic_msgs::GeoPoint::ConstPtr& message);
    
    ros::NodeHandle m_node;
    ros::Subscriber m_geopoint_subscriber;
    ros::Subscriber m_origin_subscriber;
    ros::Publisher m_active_publisher;
    ros::Publisher m_helmMode_publisher;
    ros::Publisher m_wpt_updates_publisher;
    ros::AsyncSpinner m_spinner;
    QGeoCoordinate m_location;
    QGeoCoordinate m_origin;
    std::vector<QGeoCoordinate> m_location_history;
    std::vector<QPointF> m_local_location_history;
    QPointF m_local_reference_position;
    double m_heading;
    bool m_active;
    std::string m_helmMode;
};

#endif // ROSNODE_H
