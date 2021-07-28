#include "roslink.h"
#include "ui_roslink.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <QDebug>
#include <QPainter>
#include <QGraphicsSvgItem>
#include <QColorDialog>
#include "autonomousvehicleproject.h"
#include "backgroundraster.h"
#include <QTimer>
#include "gz4d_geo.h"
#include "radardisplay.h"
#include <tf2/utils.h>
#include "geographic_msgs/GeoPoint.h"


ROSLink::ROSLink(QWidget* parent): QWidget(parent), m_ui(new Ui::ROSLink), m_range(0.0),m_bearing(0.0),m_show_tail(true)
{
    m_ui->setupUi(this);

    m_base_dimension_to_bow = 1.0;
    m_base_dimension_to_stern = 1.0;
    m_base_dimension_to_port = 1.0;
    m_base_dimension_to_stbd = 1.0;
    
    
    qRegisterMetaType<QGeoCoordinate>();
    //connectROS();
    
    m_watchdog_timer = new QTimer(this);
    connect(m_watchdog_timer, SIGNAL(timeout()), this, SLOT(watchdogUpdate()));
}

void ROSLink::connectROS()
{    
    ros::NodeHandle nh;
    m_spinner = new ros::AsyncSpinner(0);

    m_tf_buffer = new tf2_ros::Buffer;
    m_tf_listener = new tf2_ros::TransformListener(*m_tf_buffer);

    std::string robotNamespace = ros::param::param<std::string>("robotNamespace","ben");
    m_mapFrame = robotNamespace+"/map";
    emit robotNamespaceUpdated(robotNamespace.c_str());


    m_range_subscriber = nh.subscribe("range", 10, &ROSLink::rangeCallback, this);
    m_bearing_subscriber = nh.subscribe("bearing",10, &ROSLink::bearingCallback, this);

    std::string haloA_topic = "/"+robotNamespace+"/sensors/radar/HaloA/data";
    m_radar_displays[haloA_topic] = new RadarDisplay(this);
    m_radar_displays[haloA_topic]->setTF2Buffer(m_tf_buffer);
    m_radar_displays[haloA_topic]->setMapFrame(m_mapFrame);
    m_radar_displays[haloA_topic]->subscribe(haloA_topic.c_str());

    std::string haloB_topic = "/"+robotNamespace+"/sensors/radar/HaloB/data";
    m_radar_displays[haloB_topic] = new RadarDisplay(this);
    m_radar_displays[haloB_topic]->setTF2Buffer(m_tf_buffer);
    m_radar_displays[haloB_topic]->setMapFrame(m_mapFrame);
    m_radar_displays[haloB_topic]->subscribe(haloB_topic.c_str());

    m_look_at_publisher = nh.advertise<geographic_msgs::GeoPoint>("base/camera/look_at",1);
    m_look_at_mode_publisher = nh.advertise<std_msgs::String>("base/camera/look_at_mode",1);

    nh.param("base/dimension_to_bow",m_base_dimension_to_bow,m_base_dimension_to_bow);
    nh.param("base/dimension_to_stern",m_base_dimension_to_stern,m_base_dimension_to_stern);
    nh.param("base/dimension_to_stbd",m_base_dimension_to_stbd,m_base_dimension_to_stbd);
    nh.param("base/dimension_to_port",m_base_dimension_to_port,m_base_dimension_to_port);

    //m_radar_displays["/radar/HaloA/data"]->setPos(m_base_location.pos);
    //m_radar_displays["/radar/HaloA/data"]->setRotation(m_base_heading);

    m_spinner->start();
    m_watchdog_timer->start(500);
    emit rosConnected(true);
}

void ROSLink::rangeCallback(const std_msgs::Float32::ConstPtr& message)
{
    m_range_timestamp = ros::Time::now();
    m_range = message->data;
}

void ROSLink::bearingCallback(const std_msgs::Float32::ConstPtr& message)
{
    m_bearing_timestamp = ros::Time::now();
    m_bearing = message->data;
}

void ROSLink::watchdogUpdate()
{
    rangeAndBearingUpdate(m_range,m_range_timestamp,m_bearing,m_bearing_timestamp);
}

void ROSLink::followRobot(bool follow)
{
    m_follow_robot = follow;
}




void ROSLink::sendLookAt(QGeoCoordinate const &targetLocation)
{
    geographic_msgs::GeoPoint gp;
    gp.latitude = targetLocation.latitude();
    gp.longitude = targetLocation.longitude();
    m_look_at_publisher.publish(gp);
}

void ROSLink::sendLookAtMode(std::string const &mode)
{
    std_msgs::String mode_string;
    mode_string.data = mode;
    m_look_at_mode_publisher.publish(mode_string);
}



void ROSLink::showRadar(bool show)
{
    m_show_radar = show;
    for(auto rd:m_radar_displays)
    {
        rd.second->showRadar(show);
    }
    update();
}

void ROSLink::selectRadarColor()
{
    for(auto rd:m_radar_displays)
    {
        rd.second->setColor(QColorDialog::getColor(rd.second->getColor() , nullptr, "Select Color", QColorDialog::DontUseNativeDialog));
    }
    update();   
}

void ROSLink::showTail(bool show)
{
    m_show_tail = show;
    update();
}

void ROSLink::rangeAndBearingUpdate(double range, ros::Time const & range_timestamp, double bearing, ros::Time const & bearing_timestamp)
{
    QString rblabel = "Range: " + QString::number(int(range)) + " m, Bearing: " + QString::number(int(bearing)) + " degs";
    m_ui->rangeBearingLineEdit->setText(rblabel);

    ros::Time now = ros::Time::now();
    QPalette pal = palette();
    if(now-range_timestamp < ros::Duration(5) && now-bearing_timestamp < ros::Duration(5))
    {
        pal.setColor(QPalette::Foreground, Qt::black);
        pal.setColor(QPalette::Background, Qt::white);
    }
    else
    {
        pal.setColor(QPalette::Foreground, Qt::darkGray);
        pal.setColor(QPalette::Background, Qt::yellow);
    }
    m_ui->rangeBearingLineEdit->setPalette(pal);
    
}
