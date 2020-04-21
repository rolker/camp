#include "roslink.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <QDebug>
#include <QPainter>
#include <QGraphicsSvgItem>
#include "autonomousvehicleproject.h"
#include "backgroundraster.h"
#include <QTimer>
#include "gz4d_geo.h"
#include "rosdetails.h"
//#include "boost/date_time/posix_time/posix_time.hpp"
#include "radardisplay.h"


ROSAISContact::ROSAISContact(QObject* parent): QObject(parent), mmsi(0), heading(0.0)
{

}

ROSLink::ROSLink(AutonomousVehicleProject* parent): QObject(parent), GeoGraphicsItem(),m_node(nullptr), m_spinner(nullptr),m_have_local_reference(false),m_heading(0.0),m_posmv_heading(0.0),m_base_heading(0.0), m_helmMode("standby"),m_view_point_active(false),m_view_seglist_active(false),m_view_polygon_active(false),m_range(0.0),m_bearing(0.0)
{
    m_base_dimension_to_bow = 1.0;
    m_base_dimension_to_stern = 1.0;
    m_base_dimension_to_port = 1.0;
    m_base_dimension_to_stbd = 1.0;
    
    setAcceptHoverEvents(false);
    setOpacity(1.0);
    setFlag(QGraphicsItem::ItemIsMovable, false);

    //QGraphicsSvgItem *symbol = new QGraphicsSvgItem(this);
    //symbol->setSharedRenderer(autonomousVehicleProject()->symbols());
    //symbol->setElementId("Vessel");
    //symbol->setFlag(QGraphicsItem::ItemIgnoresTransformations);
    
    qRegisterMetaType<QGeoCoordinate>();
    //connectROS();
    
    m_watchdog_timer = new QTimer(this);
    connect(m_watchdog_timer, SIGNAL(timeout()), this, SLOT(watchdogUpdate()));
}

void ROSLink::connectROS()
{    
    if(ros::master::check())
    {
        if(!m_node)
        {
            m_node = new ros::NodeHandle;
            m_spinner = new ros::AsyncSpinner(0);
            m_geopoint_subscriber = m_node->subscribe("/udp/position", 10, &ROSLink::geoPointStampedCallback, this);
            m_base_navsatfix_subscriber = m_node->subscribe("/base/position", 10, &ROSLink::baseNavSatFixCallback, this);
            m_origin_subscriber = m_node->subscribe("/udp/origin", 10, &ROSLink::originCallback, this);
            m_heading_subscriber = m_node->subscribe("/udp/heading", 10, &ROSLink::headingCallback, this);
            m_base_heading_subscriber = m_node->subscribe("/base/orientation", 10, &ROSLink::baseHeadingCallback, this);
            m_ais_subscriber = m_node->subscribe("/udp/contact", 10, &ROSLink::contactCallback, this);
            m_heartbeat_subscriber = m_node->subscribe("/udp/heartbeat", 10, &ROSLink::heartbeatCallback, this);
            m_mission_status_subscriber = m_node->subscribe("/udp/project11/mission_manager/status", 10, &ROSLink::missionStatusCallback, this);
            m_posmv_position = m_node->subscribe("/udp/posmv/position", 10, &ROSLink::posmvPositionCallback, this);
            m_posmv_orientation = m_node->subscribe("/udp/posmv/orientation", 10, &ROSLink::posmvOrientationCallback, this);
            m_range_subscriber = m_node->subscribe("/range", 10, &ROSLink::rangeCallback, this);
            m_bearing_subscriber = m_node->subscribe("/bearing",10, &ROSLink::bearingCallback, this);
            m_sog_subscriber = m_node->subscribe("/udp/sog",10, &ROSLink::sogCallback, this);
            m_coverage_subscriber = m_node->subscribe("/udp/coverage", 10, &ROSLink::coverageCallback, this);
            m_ping_subscriber = m_node->subscribe("/udp/mbes_ping", 10, &ROSLink::pingCallback, this);
            m_display_subscriber = m_node->subscribe("/udp/project11/display", 10, &ROSLink::geoVizDisplayCallback, this);
            
            m_radar_displays["/radar/HaloA/data"] = new RadarDisplay(this);
            m_radar_subscriber = m_node->subscribe<marine_msgs::RadarSectorStamped>("/radar/HaloA/data", 10, boost::bind(&ROSLink::radarCallback, this, _1, "/radar/HaloA/data"));
            
            m_send_command_publisher = m_node->advertise<std_msgs::String>("/send_command",1);
            m_look_at_publisher = m_node->advertise<geographic_msgs::GeoPoint>("/base/camera/look_at",1);
            m_look_at_mode_publisher = m_node->advertise<std_msgs::String>("/base/camera/look_at_mode",1);
            
            m_node->param("/base/dimension_to_bow",m_base_dimension_to_bow,m_base_dimension_to_bow);
            m_node->param("/base/dimension_to_stern",m_base_dimension_to_stern,m_base_dimension_to_stern);
            m_node->param("/base/dimension_to_stbd",m_base_dimension_to_stbd,m_base_dimension_to_stbd);
            m_node->param("/base/dimension_to_port",m_base_dimension_to_port,m_base_dimension_to_port);
            
            double latitude, longitude;
            if(m_node->getParam("/base/latitude",latitude) && m_node->getParam("/base/longitude",longitude))
            {
                m_base_location.location.setLatitude(latitude);
                m_base_location.location.setLongitude(longitude);
                m_base_location.pos = geoToPixel(m_base_location.location,autonomousVehicleProject());
            }
            
            m_node->param("/base/heading", m_base_heading, m_base_heading);
            
            m_radar_displays["/radar/HaloA/data"]->setPos(m_base_location.pos);
            m_radar_displays["/radar/HaloA/data"]->setRotation(m_base_heading);
            
            m_spinner->start();
            m_watchdog_timer->start(500);
            emit rosConnected(true);
        }
    }
    else
    {
        if(m_node)
        {
            delete m_node;
            m_node = nullptr;
            delete m_spinner;
            m_spinner = nullptr;
            emit rosConnected(false);
        }
    }
    QTimer::singleShot(1000,this,SLOT(connectROS()));
}

QRectF ROSLink::boundingRect() const
{
    return shape().boundingRect();
}

void ROSLink::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    painter->save();

    QPen p;
    p.setCosmetic(true);

    
    p.setColor(Qt::green);
    p.setWidth(4);
    painter->setPen(p);
    painter->setBrush(Qt::cyan);
    painter->drawPath(coverageShape());
    //painter->drawPath(pingsShape());
    painter->setBrush(Qt::NoBrush);


    
    p.setColor(Qt::blue);
    p.setWidth(2);
    painter->setPen(p);
    painter->drawPath(aisShape());

    for(auto display_item: m_display_items)
    {
        for (auto point_group: display_item.second->point_groups)
        {
            p.setColor(point_group.color);
            p.setWidth(point_group.size);
            painter->setPen(p);
            for(auto point: point_group.points)
                painter->drawPoint(point.pos);
        }
        
        for(auto line: display_item.second->lines)
        {
            p.setColor(line.color);
            p.setWidth(line.size);
            painter->setPen(p);
            painter->drawPath(shape(line.points));
        }
        
        for(auto polygon: display_item.second->polygons)
        {
            p.setColor(polygon.edge_color);
            p.setWidth(polygon.edge_size);
            painter->setPen(p);
            painter->setBrush(QBrush(polygon.fill_color));
            painter->drawPath(polygon.path);
        }
        painter->setBrush(Qt::NoBrush);
    }

    if(m_show_radar)
    {
        auto avp = autonomousVehicleProject();
        if(avp)
        {
            auto bg = avp->getBackgroundRaster();
            if(bg)
            {
                painter->save();
                painter->translate(m_base_location.pos);
                for(auto rd:m_radar_displays)
                {
                    //rd.second->paint(painter, option, widget);
                }
                painter->restore();
            }
        }
    }
    
    
    p.setColor(Qt::darkBlue);
    p.setWidth(5);
    painter->setPen(p);
    painter->drawPath(baseShape());
    p.setWidth(3);
    p.setColor(Qt::lightGray);
    painter->setPen(p);
    painter->drawPath(baseShape());

    p.setWidth(9);
    p.setColor(Qt::black);
    painter->setPen(p);
    painter->drawPath(vehicleShape());
    p.setWidth(6);
    p.setColor(Qt::yellow);
    painter->setPen(p);
    painter->drawPath(vehicleShape());
    p.setWidth(3);
    if(m_node)
        p.setColor(Qt::darkGreen);
    else
        p.setColor(Qt::darkRed);
    p.setWidth(3);
    painter->setPen(p);
    painter->drawPath(vehicleShape());

    p.setWidth(11);
    p.setColor(Qt::black);
    painter->setPen(p);
    painter->drawPath(vehicleShapePosmv());
    p.setWidth(8);
    p.setColor(Qt::yellow);
    painter->setPen(p);
    painter->drawPath(vehicleShapePosmv());
    p.setWidth(3);
    if(m_node)
        p.setColor(Qt::darkGreen);
    else
        p.setColor(Qt::darkRed);
    painter->setPen(p);
    painter->drawPath(vehicleShapePosmv());


    painter->restore();
}

QPainterPath ROSLink::shape() const
{
    QPainterPath ret;
    ret.addPath(vehicleShape());
    ret.addPath(vehicleShapePosmv());
    ret.addPath(aisShape());
    ret.addPath(baseShape());
    ret.addPath(coverageShape());
    ret.addPath(pingsShape());
    
    for(auto display_item: m_display_items)
    {
        for(auto plist: display_item.second->point_groups)
        {
            for(auto p: plist.points)
                ret.addEllipse(p.pos,plist.size,plist.size);
        }
        for(auto line: display_item.second->lines)
        {
            ret.addPath(shape(line.points));
        }
        for(auto polygon: display_item.second->polygons)
        {
            ret.addPath(polygon.path);
        }
    }

//     for(auto radar_sector: m_radar_sectors)
//     {
//         //ret.moveTo(radar_sector.second->origin.pos);
//         //ret.addPixmap(radar_sector.second->sector);
//         ret.addRect(radar_sector.second->origin.pos.x(),radar_sector.second->origin.pos.y(),radar_sector.second->sector.width(),radar_sector.second->sector.height());
//     }

    auto avp = autonomousVehicleProject();
    if(avp)
    {
        auto bg = avp->getBackgroundRaster();
        if(bg)
        {
//             double pixelSize = bg->pixelSize();
//             double s = m_radar_scale/pixelSize;
//             ret.addRect(QRectF(geoToPixel(m_location,autonomousVehicleProject())+QPointF(-512*s*1.1,-512*s*1.1),QSizeF(1024*s*1.1,1024*s*1.1)));
        }
    }
    
    return ret;
}

QPainterPath ROSLink::shape(std::vector<LocationPosition> const &lps) const
{
    QPainterPath ret;
    if(lps.size() > 1)
    {
        auto p = lps.begin();
        ret.moveTo(p->pos);
        p++;
        while(p != lps.end())
        {
            ret.lineTo(p->pos);
            p++;
        }
    }
    return ret;
}

QPainterPath ROSLink::vehicleShape() const
{
    auto bgr = autonomousVehicleProject()->getBackgroundRaster();
    QPainterPath ret;
    if (m_local_location_history.size() > 1)
    {
        auto p = m_local_location_history.begin();
        ret.moveTo(*p);
        p++;
        while(p != m_local_location_history.end())
        {
            ret.lineTo(*p);
            p++;
        }
        auto last = *(m_local_location_history.rbegin());
        
        if(bgr)
        {
            qreal pixel_size = bgr->scaledPixelSize();
            if(pixel_size > .5)
                drawTriangle(ret,m_location,m_heading,pixel_size);
            else
                drawShipOutline(ret,m_location,m_heading,3,.8,.8,1);
        }
        
        //ret.addRect(-1,-5,2,10);
        //ret.addRect(-5,-1,10,2);
    }
    return ret;
}

QPainterPath ROSLink::vehicleShapePosmv() const
{
    QPainterPath ret;
    if (m_local_posmv_location_history.size() > 1)
    {
        auto p = m_local_posmv_location_history.begin();
        ret.moveTo(*p);
        p++;
        while(p != m_local_posmv_location_history.end())
        {
            ret.lineTo(*p);
            p++;
        }
        auto last = *(m_local_posmv_location_history.rbegin());
        
        auto bgr = autonomousVehicleProject()->getBackgroundRaster();
        if(bgr)
        {
            qreal pixel_size = bgr->scaledPixelSize();
            if(pixel_size > .5)
                drawTriangle(ret,m_posmv_location,m_posmv_heading,pixel_size);
            else
                drawShipOutline(ret,m_posmv_location,m_posmv_heading,2,.8,.8,2);
        }
    }
    return ret;
}


QPainterPath ROSLink::baseShape() const
{
    QPainterPath ret;
    if (m_base_location_history.size() > 1)
    {
        auto p = m_base_location_history.begin();
        ret.moveTo(p->pos);
        p++;
        while(p != m_base_location_history.end())
        {
            ret.lineTo(p->pos);
            p++;
        }
        //auto last = *(m_base_location_history.rbegin());
    }
    if(m_base_location.location.isValid())
    {
        auto bgr = autonomousVehicleProject()->getBackgroundRaster();
        if(bgr)
        {
            qreal pixel_size = bgr->scaledPixelSize();
            if(pixel_size > 1)
                drawTriangle(ret,m_base_location.location,m_base_heading,pixel_size);
            else
                // fairweather estimates: 70m by 12.70m
                drawShipOutline(ret,m_base_location.location,m_base_heading,m_base_dimension_to_bow,m_base_dimension_to_port,m_base_dimension_to_stbd,m_base_dimension_to_stern);
        }
    }
    return ret;
}


QPainterPath ROSLink::aisShape() const
{
    QPainterPath ret;
    for(auto contactList: m_contacts)
    {
        if((ros::Time::now() - contactList.second.back()->timestamp) < ros::Duration(300))
        {
            auto p = contactList.second.begin();
            ret.moveTo((*p)->location_local);
            p++;
            while(p != contactList.second.end())
            {
                ret.lineTo((*p)->location_local);
                p++;
            }
            auto last = *(contactList.second.rbegin());

            auto bgr = autonomousVehicleProject()->getBackgroundRaster();
            if(bgr)
            {
                bool forceTriangle = false;
                if (last->dimension_to_bow + last->dimension_to_stern == 0 || last->dimension_to_port + last->dimension_to_stbd == 0)
                    forceTriangle = true;
                qreal pixel_size = bgr->scaledPixelSize();
                if(pixel_size > 1 || forceTriangle)
                    drawTriangle(ret,last->location,last->heading,pixel_size);
                else
                    drawShipOutline(ret,last->location,last->heading,last->dimension_to_bow,last->dimension_to_port,last->dimension_to_stbd,last->dimension_to_stern);
            }
        }
    }
        
    return ret;
}

QPainterPath ROSLink::coverageShape() const
{
    QPainterPath ret;
    for(auto p: m_local_coverage)
        ret.addPolygon(p);
//     QPolygonF poly;
//     if(!m_local_coverage.empty())
//     {
//         auto p = m_local_coverage.begin();
//         poly << *p;
//         //ret.moveTo(*p);
//         p++;
//         while(p != m_local_coverage.end())
//         {
//             poly << *p;
//             //ret.lineTo(*p);
//             p++;
//         }
//         ret.addPolygon(poly);
//         //ret.lineTo(m_local_coverage.front());
//     }
    
    return ret;
}

QPainterPath ROSLink::pingsShape() const
{
    QPainterPath ret;
    QPolygonF poly;
    for(auto p: m_local_pings)
        if(!p.empty())
        {
            auto pp = p.begin();
            poly << *pp;
            //ret.moveTo(*p);
            pp++;
            while(pp != p.end())
            {
                poly << *pp;
                //ret.lineTo(*p);
                pp++;
            }
            ret.addPolygon(poly);
            //ret.lineTo(m_local_coverage.front());
        }
    
    return ret;
}


void ROSLink::drawTriangle(QPainterPath& path, const QGeoCoordinate& location, double heading_degrees, double scale) const
{
    QGeoCoordinate tip = location.atDistanceAndAzimuth(15*scale,heading_degrees);
    QGeoCoordinate llcorner = location.atDistanceAndAzimuth(15*scale,heading_degrees-150);
    QGeoCoordinate lrcorner = location.atDistanceAndAzimuth(15*scale,heading_degrees+150);

    QPointF ltip = geoToPixel(tip,autonomousVehicleProject());
    QPointF lllocal = geoToPixel(llcorner,autonomousVehicleProject());
    QPointF lrlocal = geoToPixel(lrcorner,autonomousVehicleProject());

    path.moveTo(ltip);
    path.lineTo(lllocal);
    path.lineTo(lrlocal);
    path.lineTo(ltip);
}

void ROSLink::drawShipOutline(QPainterPath& path, const QGeoCoordinate& location, double heading_degrees, float dimension_to_bow, float dimension_to_port, float dimension_to_stbd, float dimension_to_stern) const
{
        float length = dimension_to_bow+dimension_to_stern;
        float width = dimension_to_port+dimension_to_stbd;
        QGeoCoordinate llcorner = location.atDistanceAndAzimuth(dimension_to_port,270+heading_degrees).atDistanceAndAzimuth(dimension_to_stern,180+heading_degrees);
        QGeoCoordinate lrcorner = llcorner.atDistanceAndAzimuth(width,90+heading_degrees);
        QGeoCoordinate urcorner = lrcorner.atDistanceAndAzimuth(length,heading_degrees);
        QGeoCoordinate ulcorner = urcorner.atDistanceAndAzimuth(width,270+heading_degrees);
        QGeoCoordinate rkink = lrcorner.atDistanceAndAzimuth(length*.8,heading_degrees);
        QGeoCoordinate lkink = llcorner.atDistanceAndAzimuth(length*.8,heading_degrees);
        QGeoCoordinate bow = ulcorner.atDistanceAndAzimuth(width/2.0,90+heading_degrees);
        QPointF lllocal = geoToPixel(llcorner,autonomousVehicleProject());
        QPointF lrlocal = geoToPixel(lrcorner,autonomousVehicleProject());
        QPointF lkinkl = geoToPixel(lkink,autonomousVehicleProject());
        QPointF rkinkl = geoToPixel(rkink,autonomousVehicleProject());
        QPointF bowl = geoToPixel(bow,autonomousVehicleProject());
        
        path.moveTo(lllocal);
        path.lineTo(lrlocal);
        path.lineTo(rkinkl);
        path.lineTo(bowl);
        path.lineTo(lkinkl);
        path.lineTo(lllocal);
}


void ROSLink::read(const QJsonObject& json)
{
}

void ROSLink::write(QJsonObject& json) const
{
}

void ROSLink::setROSDetails(ROSDetails* details)
{
    m_details = details;
}


void ROSLink::geoPointStampedCallback(const geographic_msgs::GeoPointStamped::ConstPtr& message)
{
    QGeoCoordinate position(message->position.latitude,message->position.longitude,message->position.altitude);
    QMetaObject::invokeMethod(this,"updateLocation", Qt::QueuedConnection, Q_ARG(QGeoCoordinate, position));
}

void ROSLink::posmvPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& message)
{
    QGeoCoordinate position(message->latitude, message->longitude, message->altitude);
    QMetaObject::invokeMethod(this,"updatePosmvLocation", Qt::QueuedConnection, Q_ARG(QGeoCoordinate, position));
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

void ROSLink::sogCallback(const geometry_msgs::TwistStamped::ConstPtr& message)
{
    qreal sog = sqrt(message->twist.linear.x*message->twist.linear.x+message->twist.linear.y*message->twist.linear.y);
    QMetaObject::invokeMethod(this,"updateSog", Qt::QueuedConnection, Q_ARG(qreal, sog));  
}

void ROSLink::updateSog(qreal sog)
{
    // 1852m per NM
    m_sog = sog*1.9438;
    m_sog_history.append(m_sog);
    if(m_sog_history.length() > 200)
        m_sog_history.pop_front();
    qreal sog_sum = 0;
    for(auto s: m_sog_history)
        sog_sum += s;
    m_sog_avg = sog_sum/m_sog_history.length();
    //qDebug() << m_sog << " knts, " << m_sog_avg << " knts avg";
    m_details->sogUpdate(m_sog,m_sog_avg);
}


void ROSLink::baseNavSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr& message)
{
    QGeoCoordinate position(message->latitude, message->longitude, message->altitude);
    QMetaObject::invokeMethod(this,"updateBaseLocation", Qt::QueuedConnection, Q_ARG(QGeoCoordinate, position));
}

void ROSLink::originCallback(const geographic_msgs::GeoPoint::ConstPtr& message)
{
    m_origin.setLatitude(message->latitude);
    m_origin.setLongitude(message->longitude);
    m_origin.setAltitude(message->altitude);
    QMetaObject::invokeMethod(this,"updateOriginLocation", Qt::QueuedConnection, Q_ARG(QGeoCoordinate, m_origin));
    //qDebug() << m_origin;
}

void ROSLink::headingCallback(const marine_msgs::NavEulerStamped::ConstPtr& message)
{
    QMetaObject::invokeMethod(this,"updateHeading", Qt::QueuedConnection, Q_ARG(double, message->orientation.heading));
}

void ROSLink::posmvOrientationCallback(const marine_msgs::NavEulerStamped::ConstPtr& message)
{
    QMetaObject::invokeMethod(this,"updatePosmvHeading", Qt::QueuedConnection, Q_ARG(double, message->orientation.heading));
}

void ROSLink::baseHeadingCallback(const marine_msgs::NavEulerStamped::ConstPtr& message)
{
    QMetaObject::invokeMethod(this,"updateBaseHeading", Qt::QueuedConnection, Q_ARG(double, message->orientation.heading));
}


void ROSLink::contactCallback(const marine_msgs::Contact::ConstPtr& message)
{
    //qDebug() << message->mmsi << ": " << message->name.c_str() << " heading: " << message->heading << " cog: " << message->cog << " dimensions: port: " << message->dimension_to_port << " strbd: " << message->dimension_to_stbd << " bow " << message->dimension_to_bow << " stern: " << message->dimension_to_stern;
    //qDebug() << "\t\t" << message->position.latitude << ", " << message->position.longitude;
    if(message->position.latitude > 90 || message->position.longitude > 180)
        return;
    ROSAISContact *c = new ROSAISContact();
    c->timestamp = message->header.stamp;
    c->mmsi = message->mmsi;
    c->name = message->name;
    c->location.setLatitude(message->position.latitude);
    c->location.setLongitude(message->position.longitude);
    if(message->heading < 0)
        c->heading = message->cog*180.0/M_PI;
    else
        c->heading = message->heading*180.0/M_PI;
    c->dimension_to_bow = message->dimension_to_bow;
    c->dimension_to_port = message->dimension_to_port;
    c->dimension_to_stbd = message->dimension_to_stbd;
    c->dimension_to_stern = message->dimension_to_stern;
    QMetaObject::invokeMethod(this,"addAISContact", Qt::QueuedConnection, Q_ARG(ROSAISContact*, c));
}

void ROSLink::heartbeatCallback(const marine_msgs::Heartbeat::ConstPtr& message)
{
    ros::Time last_heartbeat_receive_time = ros::Time::now();
    ros::Time last_heartbeat_timestamp = message->header.stamp;
    
    QString status_string;
    QString helm_mode;
    for(auto kv: message->values)
    {
        status_string += kv.key.c_str();
        status_string += ": ";
        status_string += kv.value.c_str();
        status_string += "\n";
        if (kv.key == "piloting_mode")
            helm_mode = kv.value.c_str();
    }
    
    QMetaObject::invokeMethod(m_details,"updateVehicleStatus", Qt::QueuedConnection, Q_ARG(QString const&, status_string));
    QMetaObject::invokeMethod(this,"updateHeartbeatTimes", Qt::QueuedConnection, Q_ARG(ros::Time const&, last_heartbeat_timestamp), Q_ARG(ros::Time const&, last_heartbeat_receive_time));
    QMetaObject::invokeMethod(m_details,"updateHelmMode", Qt::QueuedConnection, Q_ARG(QString const&, helm_mode));
}

void ROSLink::missionStatusCallback(const marine_msgs::Heartbeat::ConstPtr& message)
{
    QString status_string;
    for(auto kv: message->values)
    {
        status_string += kv.key.c_str();
        status_string += ": ";
        status_string += kv.value.c_str();
        status_string += "\n";
    }
    
    QMetaObject::invokeMethod(m_details,"updateMissionStatus", Qt::QueuedConnection, Q_ARG(QString const&, status_string));
}


void ROSLink::updateHeartbeatTimes(const ros::Time& last_heartbeat_timestamp, const ros::Time& last_heartbeat_receive_time)
{
    m_last_heartbeat_timestamp = last_heartbeat_timestamp;
    m_last_heartbeat_receive_time = last_heartbeat_receive_time;

}

void ROSLink::watchdogUpdate()
{
    if(m_node)
    {
        ros::Time now = ros::Time::now();
        ros::Duration diff = now-m_last_heartbeat_timestamp;
        //std::cerr << "timestamp: " << m_last_heartbeat_timestamp << "\tnow: " << now << "\tdiff:" << diff << std::endl;

        m_details->heartbeatDelay(diff.toSec(), m_last_heartbeat_timestamp, m_last_heartbeat_receive_time);
        m_details->rangeAndBearingUpdate(m_range,m_range_timestamp,m_bearing,m_bearing_timestamp);
    }
    else
    {
        ros::Time uninit_time;
        m_details->heartbeatDelay(1000.0, uninit_time, uninit_time);
    }
}


QGeoCoordinate ROSLink::rosMapToGeo(const QPointF& location) const
{
    gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon> gr(m_origin.latitude(),m_origin.longitude(),m_origin.altitude());
    gz4d::geo::LocalENU<> geoReference(gr);    //geoReference = gz4d::geo::LocalENU<>(gr);
    auto ecef = geoReference.toECEF(gz4d::Point<double>(location.x(),location.y(),0.0));
    gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon>ret(ecef);
    return QGeoCoordinate(ret[0],ret[1]);
}


void ROSLink::sendMissionPlan(const QString& plan)
{
    //sendCommand("mission_plan "+plan.toStdString());
    sendCommand("mission_manager replace_task mission_plan "+plan.toStdString());
//     std_msgs::String mp;
//     mp.data = plan.toStdString();
//     if(m_node)
//         m_mission_plan_publisher.publish(mp);
}

void ROSLink::appendMission(const QString& plan)
{
    sendCommand("mission_manager append_task mission_plan "+plan.toStdString());
}

void ROSLink::prependMission(const QString& plan)
{
    sendCommand("mission_manager prepend_task mission_plan "+plan.toStdString());
}

void ROSLink::updateMission(const QString& plan)
{
    sendCommand("mission_manager update_task mission_plan "+plan.toStdString());
}

void ROSLink::sendHover(const QGeoCoordinate& hoverLocation)
{
    std::stringstream updates;
    updates << std::fixed << std::setprecision(7) << "mission_manager override hover " << hoverLocation.latitude() << " " << hoverLocation.longitude();
        
    sendCommand(updates.str());
}  

void ROSLink::sendGoto(const QGeoCoordinate& gotoLocation)
{
    std::stringstream updates;
    updates << std::fixed << std::setprecision(7) << "mission_manager override goto " << gotoLocation.latitude() << " " << gotoLocation.longitude();
        
    sendCommand(updates.str());
}

void ROSLink::sendNextItem()
{
    std::stringstream updates;
    updates << "mission_manager next_task";
        
    sendCommand(updates.str());
}

void ROSLink::restartMission()
{
    std::stringstream updates;
    updates << "mission_manager restart_mission";
        
    sendCommand(updates.str());
}


void ROSLink::sendGotoLine(int waypoint_index)
{
    std::stringstream updates;
    updates << "goto_line " << waypoint_index;
        
    sendCommand(updates.str());
}

void ROSLink::sendStartLine(int waypoint_index)
{
    std::stringstream updates;
    updates << "start_line " << waypoint_index;
        
    sendCommand(updates.str());
}

void ROSLink::updateLocation(const QGeoCoordinate& location)
{
    prepareGeometryChange();
    m_location_history.push_back(location);
    m_local_location_history.push_back(geoToPixel(location,autonomousVehicleProject()));
    while (m_local_location_history.size()>500)
        m_local_location_history.pop_front();
    m_location = location;
    update();
}

void ROSLink::updatePosmvLocation(const QGeoCoordinate& location)
{
    prepareGeometryChange();
    m_posmv_location_history.push_back(location);
    m_local_posmv_location_history.push_back(geoToPixel(location,autonomousVehicleProject()));
    while (m_local_posmv_location_history.size()>1500)
        m_local_posmv_location_history.pop_front();
    m_posmv_location = location;
    update();
}


void ROSLink::updateBaseLocation(const QGeoCoordinate& location)
{
    prepareGeometryChange();
    m_base_location.location = location;
    m_base_location.pos = geoToPixel(location,autonomousVehicleProject());
    m_base_location_history.push_back(m_base_location);
    while (m_base_location_history.size()>100)
        m_base_location_history.pop_front();
    update();
}

void ROSLink::updateOriginLocation(const QGeoCoordinate& location)
{   
    auto pixel_location = geoToPixel(location,autonomousVehicleProject());
    if(!m_have_local_reference || pixel_location != m_local_reference_position)
    {
        prepareGeometryChange();
        setPos(geoToPixel(location,autonomousVehicleProject()));
        m_local_reference_position = geoToPixel(location,autonomousVehicleProject());
        m_have_local_reference = true;
        recalculatePositions();
        emit originUpdated();
    }
}

void ROSLink::updateHeading(double heading)
{
    prepareGeometryChange();
    m_heading = heading;
    update();
}

void ROSLink::updatePosmvHeading(double heading)
{
    prepareGeometryChange();
    m_posmv_heading = heading;
    update();
}

void ROSLink::updateBaseHeading(double heading)
{
    prepareGeometryChange();
    m_base_heading = heading;
    update();
}

void ROSLink::addAISContact(ROSAISContact *c)
{
    prepareGeometryChange();
    c->location_local = geoToPixel(c->location,autonomousVehicleProject());
    m_contacts[c->mmsi].push_back(c);
    while(!m_contacts[c->mmsi].empty() && (ros::Time::now() - m_contacts[c->mmsi].front()->timestamp) > ros::Duration(600))
        m_contacts[c->mmsi].pop_front();
    while(m_contacts[c->mmsi].size()>100)
        m_contacts[c->mmsi].pop_front();
    update();
}

void ROSLink::updateBackground(BackgroundRaster *bgr)
{
    setParentItem(bgr);
    recalculatePositions();
}

void ROSLink::recalculatePositions()
{
    prepareGeometryChange();
    setPos(0,0);
    m_local_location_history.clear();
    
    AutonomousVehicleProject *avp = autonomousVehicleProject();
    
    for(auto l: m_location_history)
    {
        m_local_location_history.push_back(geoToPixel(l,avp));            
    }

    m_local_posmv_location_history.clear();
    for(auto l: m_posmv_location_history)
    {
        m_local_posmv_location_history.push_back(geoToPixel(l,avp));            
    }

    
    for(std::pair<std::string, std::shared_ptr<geoviz::Item> > display_item: m_display_items)
    {
        display_item.second->label_position.pos = geoToPixel(display_item.second->label_position.location,avp);
        for(auto pl: display_item.second->point_groups)
        {
            for(auto p: pl.points)
                p.pos = geoToPixel(p.location,avp);
        }
        for(auto l: display_item.second->lines)
        {
            for(auto p: l.points)
            {
                //std::cerr << "old pos: " << p.pos.x() << ", " << p.pos.y() << std::endl;
                p.pos = geoToPixel(p.location,avp);
                //std::cerr << "new pos: " << p.pos.x() << ", " << p.pos.y() << std::endl;
            }
        }
        for(auto poly: display_item.second->polygons)
        {
            for(auto op: poly.outer)
                op.pos = geoToPixel(op.location,avp);
            for(auto ir: poly.inner)
                for(auto ip: ir)
                    ip.pos = geoToPixel(ip.location,avp);
        }
    }

    for(auto contactList: m_contacts)
    {
        for(auto contact: contactList.second)
            contact->location_local = geoToPixel(contact->location,avp);
    }

    if(m_have_local_reference)
    {
        m_local_reference_position = geoToPixel(m_origin,avp);
    }
    
    m_base_location.pos = geoToPixel(m_base_location.location,avp);
    for(LocationPosition &l: m_base_location_history)
    {
        l.pos = geoToPixel(l.location,avp);            
    }
    
    for(auto rd: m_radar_displays)
    {
        rd.second->setPos(m_base_location.pos);
        
        auto bgr = avp->getBackgroundRaster();
        if(bgr)
            rd.second->setPixelSize(bgr->pixelSize());
    }
    
    update();
}

const std::string& ROSLink::helmMode() const
{
    return m_helmMode;
}

void ROSLink::setHelmMode(const std::string& helmMode)
{
    m_helmMode = helmMode;
    sendCommand("piloting_mode "+helmMode);
//     std_msgs::String hm;
//     hm.data = helmMode;
//     m_helmMode_publisher.publish(hm);
}

void ROSLink::sendCommand(const std::string& command)
{
    std_msgs::String cmd;
    cmd.data = command;
    qDebug() << command.c_str();
    m_send_command_publisher.publish(cmd);
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

AutonomousVehicleProject * ROSLink::autonomousVehicleProject() const
{
    return qobject_cast<AutonomousVehicleProject*>(parent());
}

void ROSLink::coverageCallback(const geographic_msgs::GeoPath::ConstPtr& message)
{
    QList<QList<QGeoCoordinate> > coverage;
    QList<QPolygonF> local_coverage;
    coverage.push_back(QList<QGeoCoordinate>());
    local_coverage.push_back(QPolygonF());
    for(auto gp: message->poses)
    {
        QGeoCoordinate gc;
        gc.setLatitude(gp.pose.position.latitude);
        gc.setLongitude(gp.pose.position.longitude);
        if(gc.isValid())
        {
            coverage.back().append(gc);
            local_coverage.back().append(geoToPixel(gc,autonomousVehicleProject()));
        }
        else
        {
            coverage.push_back(QList<QGeoCoordinate>());
            local_coverage.push_back(QPolygonF());
        }
    }
    QMetaObject::invokeMethod(this,"updateCoverage", Qt::QueuedConnection, Q_ARG(QList<QList<QGeoCoordinate> >, coverage), Q_ARG(QList<QPolygonF>, local_coverage));
}

void ROSLink::geoVizDisplayCallback(const geographic_visualization_msgs::GeoVizItem::ConstPtr& message)
{
    AutonomousVehicleProject *avp = autonomousVehicleProject();
    geoviz::Item *item = new geoviz::Item();
    item->id = message->id;
    item->label = message->label;
    item->label_position.location.setLatitude(message->label_position.latitude);
    item->label_position.location.setLongitude(message->label_position.longitude);
    item->label_position.pos = geoToPixel(item->label_position.location,avp);
    for(auto pg: message->point_groups)
    {
        geoviz::PointList pl;
        pl.color.setRedF(pg.color.r);
        pl.color.setGreenF(pg.color.g);
        pl.color.setBlueF(pg.color.b);
        pl.color.setAlphaF(pg.color.a);
        pl.size = pg.size;
        
        for(auto p: pg.points)
        {
            LocationPosition lp;
            lp.location.setLatitude(p.latitude);
            lp.location.setLongitude(p.longitude);
            lp.pos = geoToPixel(lp.location,avp);
            pl.points.push_back(lp);
        }
        item->point_groups.push_back(pl);
    }
    for(auto l: message->lines)
    {
        geoviz::PointList pl;
        pl.color.setRedF(l.color.r);
        pl.color.setGreenF(l.color.g);
        pl.color.setBlueF(l.color.b);
        pl.color.setAlphaF(l.color.a);
        pl.size = l.size;
        
        for(auto p: l.points)
        {
            LocationPosition lp;
            lp.location.setLatitude(p.latitude);
            lp.location.setLongitude(p.longitude);
            lp.pos = geoToPixel(lp.location,avp);
            pl.points.push_back(lp);
        }
        item->lines.push_back(pl);
    }
    for(auto p: message->polygons)
    {
        geoviz::Polygon polygon;
        QPolygonF qPoly;
        for(auto op: p.outer.points) // outer points
        {
            LocationPosition lp;
            lp.location.setLatitude(op.latitude);
            lp.location.setLongitude(op.longitude);
            lp.pos = geoToPixel(lp.location,avp);
            qPoly << lp.pos;
            polygon.outer.push_back(lp);
        }
        polygon.path.addPolygon(qPoly);
        QPainterPath innerPath;
        for(auto ir: p.inner) //inner rings
        {
            polygon.inner.push_back(std::vector<LocationPosition>());
            QPolygonF innerPoly;
            for(auto ip: ir.points) // inner ring points
            {
                LocationPosition lp;
                lp.location.setLatitude(ip.latitude);
                lp.location.setLongitude(ip.longitude);
                lp.pos = geoToPixel(lp.location,avp);
                innerPoly << lp.pos;
                polygon.inner.back().push_back(lp);
            }
            innerPath.addPolygon(innerPoly);
        }
        polygon.path = polygon.path.subtracted(innerPath);
        polygon.fill_color.setRedF(p.fill_color.r);
        polygon.fill_color.setGreenF(p.fill_color.g);
        polygon.fill_color.setBlueF(p.fill_color.b);
        polygon.fill_color.setAlphaF(p.fill_color.a);
        polygon.edge_color.setRedF(p.edge_color.r);
        polygon.edge_color.setGreenF(p.edge_color.g);
        polygon.edge_color.setBlueF(p.edge_color.b);
        polygon.edge_color.setAlphaF(p.edge_color.a);
        polygon.edge_size = p.edge_size;
        item->polygons.push_back(polygon);
    }
    
    QMetaObject::invokeMethod(this,"updateDisplayItem", Qt::QueuedConnection, Q_ARG(geoviz::Item*, item));
}

void ROSLink::updateDisplayItem(geoviz::Item *item)
{
    prepareGeometryChange();
    m_display_items[item->id] = std::shared_ptr<geoviz::Item>(item);
    update();   
}

void ROSLink::showRadar(bool show)
{
    m_show_radar = show;
    update();
}

void ROSLink::radarCallback(const marine_msgs::RadarSectorStamped::ConstPtr &message, const std::string &topic)
{
    if (m_show_radar && !message->sector.scanlines.empty())
    {
        double angle1 = message->sector.scanlines[0].angle;
        double angle2 = message->sector.scanlines.back().angle;
        double range = message->sector.scanlines[0].range;
        int w = message->sector.scanlines[0].intensities.size();
        int h = message->sector.scanlines.size();
        QImage * sector = new QImage(w,h,QImage::Format_Grayscale8);
        sector->fill(Qt::darkGray);
        for(int i = 0; i < h; i++)
            for(int j = 0; j < w; j++)
                sector->bits()[i*w+j] = message->sector.scanlines[i].intensities[j]*16; // *16 to convert from 4 to 8 bits
        QMetaObject::invokeMethod(m_radar_displays[topic],"addSector", Qt::QueuedConnection, Q_ARG(double, angle1), Q_ARG(double, angle2), Q_ARG(double, range), Q_ARG(QImage *, sector));
    }
}

void ROSLink::pingCallback(const sensor_msgs::PointCloud::ConstPtr& message)
{
    QList<QGeoCoordinate> ping;
    QList<QPointF> local_ping;
//     for(auto gp: message->poses)
//     {
//         QGeoCoordinate gc;
//         gc.setLatitude(gp.pose.position.latitude);
//         gc.setLongitude(gp.pose.position.longitude);
//         ping.append(gc);
//         local_ping.append(geoToPixel(gc,autonomousVehicleProject())-m_local_reference_position);
//     }
//     QMetaObject::invokeMethod(this,"addPing", Qt::QueuedConnection, Q_ARG(QList<QGeoCoordinate>, ping), Q_ARG(QList<QPointF>, local_ping));
}


void ROSLink::updateCoverage(QList<QList<QGeoCoordinate> > coverage, QList<QPolygonF> local_coverage)
{
    prepareGeometryChange();
    m_coverage = coverage;
    m_local_coverage = local_coverage;
    update();
}

void ROSLink::addPing(QList<QGeoCoordinate> ping, QList<QPointF> local_ping)
{
    prepareGeometryChange();
    m_pings.append(ping);
    m_local_pings.append(local_ping);
    update();
}
