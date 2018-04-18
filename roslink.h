#ifndef ROSLINK_H
#define ROSLINK_H

#include "geographicsitem.h"

#include "geographic_msgs/GeoPointStamped.h"
#include "mission_plan/NavEulerStamped.h"
#include "ros/ros.h"
#include "asv_msgs/AISContact.h"

class ROSLink : public QObject, GeoGraphicsItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    ROSLink(AutonomousVehicleProject* parent);
    
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    QPainterPath shape() const override;
    
    void write(QJsonObject &json) const;
    void read(const QJsonObject &json);
    
    int type() const {return ROSLinkType;}
    
    bool active() const;
    void setActive(bool active);
    
    std::string const &helmMode() const;
    void setHelmMode(const std::string& helmMode);

    struct Contact
    {
        std::string name;
        QGeoCoordinate location;
        QPointF location_local;
        double heading;
    };
    
signals:

    void rosConnected(bool connected);
    
public slots:
    void updateLocation(QGeoCoordinate const &location);
    void updateOriginLocation(QGeoCoordinate const &location);
    void updateHeading(double heading);
    void updateBackground(BackgroundRaster *bgr);
    void addAISContact(uint32_t mmsi, Contact c);
    void sendWaypoints(QList<QGeoCoordinate> const &waypoints);
    void sendLoiter(QGeoCoordinate const &loiterLocation);
    void connectROS();
    
    
    
private:
    void geoPointStampedCallback(const geographic_msgs::GeoPointStamped::ConstPtr& message); 
    void originCallback(const geographic_msgs::GeoPoint::ConstPtr& message);
    void headingCallback(const mission_plan::NavEulerStamped::ConstPtr& message);
    void aisCallback(const asv_msgs::AISContact::ConstPtr& message);
    
    void drawTriangle(QPainterPath &path, QPointF const &location, double heading_degrees, double scale=1.0) const;
    
    AutonomousVehicleProject *autonomousVehicleProject() const;
    
    ros::NodeHandle *m_node;
    ros::Subscriber m_geopoint_subscriber;
    ros::Subscriber m_origin_subscriber;
    ros::Subscriber m_heading_subscriber;
    ros::Subscriber m_ais_subscriber;
    ros::Publisher m_active_publisher;
    ros::Publisher m_helmMode_publisher;
    ros::Publisher m_wpt_updates_publisher;
    ros::Publisher m_loiter_updates_publisher;
    ros::AsyncSpinner *m_spinner;
    QGeoCoordinate m_location;
    QGeoCoordinate m_origin;
    std::vector<QGeoCoordinate> m_location_history;
    std::vector<QPointF> m_local_location_history;
    QPointF m_local_reference_position;
    bool m_have_local_reference;
    double m_heading;
    bool m_active;
    std::string m_helmMode;
    
    typedef std::vector<Contact> ContactList;
    typedef std::map<uint32_t,ContactList> ContactMap;
    
    ContactMap m_contacts;
    
    
};

#endif // ROSNODE_H
