#ifndef ROSLINK_H
#define ROSLINK_H

#include <QWidget>
#include "geographicsitem.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/PointCloud.h"
#include "locationposition.h"
#include <tf2_ros/transform_listener.h>

namespace Ui
{
class ROSLink;
}


class RadarDisplay;




class ROSLink : public QWidget
{
    Q_OBJECT
public:
    ROSLink(QWidget* parent);
    

signals:

    void rosConnected(bool connected);
    void originUpdated();
    void robotNamespaceUpdated(QString robot_namespace);
    void centerMap(QGeoCoordinate location);
    
public slots:
    void sendLookAt(QGeoCoordinate const &targetLocation);
    void sendLookAtMode(std::string const &mode);
    void connectROS();

    void watchdogUpdate();
    void showRadar(bool show);
    void selectRadarColor();
    void showTail(bool show);

    void rangeAndBearingUpdate(double range, ros::Time const &range_timestamp, double bearing, ros::Time const &bearing_timestamp);
    
private:
    void rangeCallback(const std_msgs::Float32::ConstPtr& message);
    void bearingCallback(const std_msgs::Float32::ConstPtr& message);
    
    Ui::ROSLink* m_ui;

    AutonomousVehicleProject * m_autonomousVehicleProject =nullptr;
    
    ros::Subscriber m_origin_subscriber;
    ros::Subscriber m_range_subscriber;
    ros::Subscriber m_bearing_subscriber;
    std::map<std::string, ros::Subscriber> m_radar_subscribers;
    ros::Subscriber m_clock_subscriber;
    
    ros::Publisher m_look_at_publisher;
    ros::Publisher m_look_at_mode_publisher;
    
    ros::AsyncSpinner *m_spinner = nullptr;
    
    float m_base_dimension_to_stbd; 
    float m_base_dimension_to_port;
    float m_base_dimension_to_bow;
    float m_base_dimension_to_stern; 
    


    QList<QGeoCoordinate> m_current_path;
    QList<QPointF> m_local_current_path;
    

    std::map<std::string,RadarDisplay*> m_radar_displays;
    
    bool m_show_radar;
    bool m_show_tail;

    QTimer * m_watchdog_timer;
    
    double m_range;
    ros::Time m_range_timestamp;
    double m_bearing;
    ros::Time m_bearing_timestamp;
    

    tf2_ros::Buffer* m_tf_buffer = nullptr;
    tf2_ros::TransformListener* m_tf_listener = nullptr;
    std::string m_mapFrame;
};

#endif // ROSNODE_H
