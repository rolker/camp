#ifndef ROSLINK_H
#define ROSLINK_H

#include "geographicsitem.h"

#include "geographic_msgs/GeoPointStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "marine_msgs/NavEulerStamped.h"
#include "marine_msgs/Heartbeat.h"
#include "marine_msgs/RadarSectorStamped.h"
#include "ros/ros.h"
#include "marine_msgs/Contact.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/TwistStamped.h"
#include "geographic_msgs/GeoPath.h"
#include "sensor_msgs/PointCloud.h"
#include "locationposition.h"
#include "geographic_visualization_msgs/GeoVizItem.h"

Q_DECLARE_METATYPE(ros::Time);

class ROSDetails;
class RadarDisplay;

struct ROSAISContact: public QObject
{
    Q_OBJECT
public:
    ROSAISContact(QObject *parent = nullptr);
    ros::Time timestamp;
    uint32_t mmsi;
    std::string name;
    QGeoCoordinate location;
    QPointF location_local;
    double heading;
    float dimension_to_stbd; 
    float dimension_to_port;
    float dimension_to_bow;
    float dimension_to_stern;
    float cog;
    float sog;
};

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


class ROSLink : public QObject, public GeoGraphicsItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    ROSLink(AutonomousVehicleProject* parent);
    
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    QPainterPath shape() const override;
    QPainterPath shape(std::vector<LocationPosition> const &lps) const;
    QPainterPath vehicleShape() const;
    QPainterPath vehicleShapePosmv() const;
    QPainterPath baseShape() const;
    QPainterPath aisShape() const;
    QPainterPath coverageShape() const;
    QPainterPath pingsShape() const;

    void write(QJsonObject &json) const;
    void read(const QJsonObject &json);
    
    int type() const {return ROSLinkType;}
    
    std::string const &helmMode() const;
    void setHelmMode(const std::string& helmMode);
    void sendCommand(const std::string& command);
    
    void setROSDetails(ROSDetails *details);

    
signals:

    void rosConnected(bool connected);
    void originUpdated();
    
public slots:
    void updateLocation(QGeoCoordinate const &location);
    void updatePosmvLocation(QGeoCoordinate const &location);
    void updateBaseLocation(QGeoCoordinate const &location);
    void updateOriginLocation(QGeoCoordinate const &location);
    void updateHeading(double heading);
    void updatePosmvHeading(double heading);
    void updateBaseHeading(double heading);
    void updateBackground(BackgroundRaster *bgr);
    void updateCoverage(QList<QList<QGeoCoordinate> > coverage, QList<QPolygonF> local_coverage);
    void addPing(QList<QGeoCoordinate> ping, QList<QPointF> local_ping);
    void updateDisplayItem(geoviz::Item *item);

    void recalculatePositions();
    void addAISContact(ROSAISContact *c);
    
    void sendMissionPlan(QString const &plan);
    void appendMission(QString const &plan);
    void prependMission(QString const &plan);
    void updateMission(QString const &plan);
    
    void sendHover(QGeoCoordinate const &targetLocation);
    void sendGoto(QGeoCoordinate const &targetLocation);
    void sendNextItem();
    void restartMission();
    void sendLookAt(QGeoCoordinate const &targetLocation);
    void sendLookAtMode(std::string const &mode);
    void sendGotoLine(int waypoint_index);
    void sendStartLine(int waypoint_index);
    void connectROS();
    void updateHeartbeatTimes(ros::Time const &last_heartbeat_timestamp, ros::Time const &last_heartbeat_receive_time);
    void watchdogUpdate();
    void updateSog(qreal sog);
    void showRadar(bool show);
    
private:
    void geoPointStampedCallback(const geographic_msgs::GeoPointStamped::ConstPtr& message);
    void baseNavSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr& message);
    void originCallback(const geographic_msgs::GeoPoint::ConstPtr& message);
    void headingCallback(const marine_msgs::NavEulerStamped::ConstPtr& message);
    void baseHeadingCallback(const marine_msgs::NavEulerStamped::ConstPtr& message);
    void contactCallback(const marine_msgs::Contact::ConstPtr& message);
    void heartbeatCallback(const marine_msgs::Heartbeat::ConstPtr& message);
    void missionStatusCallback(const marine_msgs::Heartbeat::ConstPtr& message);
    void posmvOrientationCallback(const marine_msgs::NavEulerStamped::ConstPtr& message);
    void posmvPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& message);
    void rangeCallback(const std_msgs::Float32::ConstPtr& message);
    void bearingCallback(const std_msgs::Float32::ConstPtr& message);
    void sogCallback(const geometry_msgs::TwistStamped::ConstPtr& message);
    void coverageCallback(const geographic_msgs::GeoPath::ConstPtr& message);
    void pingCallback(const sensor_msgs::PointCloud::ConstPtr& message);
    void geoVizDisplayCallback(const geographic_visualization_msgs::GeoVizItem::ConstPtr& message);
    void radarCallback(const marine_msgs::RadarSectorStamped::ConstPtr &message, const std::string &topic);
    
    void drawTriangle(QPainterPath &path, QGeoCoordinate const &location, double heading_degrees, double scale=1.0) const;
    void drawShipOutline(QPainterPath &path, QGeoCoordinate const &location, double heading_degrees, float dimension_to_bow, float dimension_to_port, float dimension_to_stbd, float dimension_to_stern) const;
    
    QGeoCoordinate rosMapToGeo(QPointF const &location) const;
    
    AutonomousVehicleProject *autonomousVehicleProject() const;
    
    ros::NodeHandle *m_node;
    ros::Subscriber m_geopoint_subscriber;
    ros::Subscriber m_base_navsatfix_subscriber;
    ros::Subscriber m_origin_subscriber;
    ros::Subscriber m_heading_subscriber;
    ros::Subscriber m_base_heading_subscriber;
    ros::Subscriber m_ais_subscriber;
    ros::Subscriber m_heartbeat_subscriber;
    ros::Subscriber m_mission_status_subscriber;
    ros::Subscriber m_posmv_position;
    ros::Subscriber m_posmv_orientation;
    ros::Subscriber m_range_subscriber;
    ros::Subscriber m_bearing_subscriber;
    ros::Subscriber m_sog_subscriber;
    ros::Subscriber m_coverage_subscriber;
    ros::Subscriber m_ping_subscriber;
    ros::Subscriber m_display_subscriber;
    ros::Subscriber m_radar_subscriber;
    ros::Subscriber m_clock_subscriber;
    
    ros::Publisher m_send_command_publisher;
    ros::Publisher m_look_at_publisher;
    ros::Publisher m_look_at_mode_publisher;
    
    ros::AsyncSpinner *m_spinner;
    QGeoCoordinate m_location;
    QGeoCoordinate m_posmv_location;
    LocationPosition m_base_location; // location of the base operator station (ship, shore station, etc)
    QGeoCoordinate m_origin;
    std::vector<QGeoCoordinate> m_location_history;
    std::list<QPointF> m_local_location_history;
    std::vector<QGeoCoordinate> m_posmv_location_history;
    std::list<QPointF> m_local_posmv_location_history;
    std::list<LocationPosition> m_base_location_history;
    QPointF m_local_reference_position;
    bool m_have_local_reference;
    double m_heading;
    double m_posmv_heading;
    double m_base_heading;
    
    float m_base_dimension_to_stbd; 
    float m_base_dimension_to_port;
    float m_base_dimension_to_bow;
    float m_base_dimension_to_stern; 
    
    std::string m_helmMode;
    
    typedef std::list<ROSAISContact*> ContactList;
    typedef std::map<uint32_t,ContactList> ContactMap;
    
    ContactMap m_contacts;
    ROSDetails *m_details;
    
    QGeoCoordinate m_view_point;
    QPointF m_local_view_point;
    bool m_view_point_active;
    
    QList<QGeoCoordinate> m_view_seglist;
    QList<QPointF> m_local_view_seglist;
    bool m_view_seglist_active;
    
    QList<QGeoCoordinate> m_view_polygon;
    QList<QPointF> m_local_view_polygon;
    bool m_view_polygon_active;

    QList<QList<QGeoCoordinate> > m_coverage;
    QList<QPolygonF> m_local_coverage;

    QList<QList<QGeoCoordinate> > m_pings;
    QList<QList<QPointF> > m_local_pings;

    QList<QGeoCoordinate> m_current_path;
    QList<QPointF> m_local_current_path;
    
    std::map<std::string,std::shared_ptr<geoviz::Item> > m_display_items;

    std::map<std::string,RadarDisplay*> m_radar_displays;
    
    bool m_show_radar;

    ros::Time m_last_heartbeat_timestamp;
    ros::Time m_last_heartbeat_receive_time;
    
    QTimer * m_watchdog_timer;
    
    double m_range;
    ros::Time m_range_timestamp;
    double m_bearing;
    ros::Time m_bearing_timestamp;
    
    QList<qreal> m_sog_history;
    qreal m_sog;
    qreal m_sog_avg;
};

#endif // ROSNODE_H
