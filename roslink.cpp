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

ROSAISContact::ROSAISContact(QObject* parent): QObject(parent), mmsi(0), heading(0.0)
{

}


ROSLink::ROSLink(AutonomousVehicleProject* parent): QObject(parent), GeoGraphicsItem(),m_node(nullptr), m_spinner(nullptr),m_have_local_reference(false),m_heading(0.0),m_posmv_heading(0.0),m_base_heading(0.0), m_active(false),m_helmMode("standby"),m_view_point_active(false),m_view_seglist_active(false),m_view_polygon_active(false),m_map_scale(1.0)
{
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
            m_ais_subscriber = m_node->subscribe("/udp/ais", 10, &ROSLink::aisCallback, this);
            m_heartbeat_subscriber = m_node->subscribe("/udp/heartbeat", 10, &ROSLink::heartbeatCallback, this);
            m_view_point_subscriber = m_node->subscribe("/udp/view_point", 10, &ROSLink::viewPointCallback, this);
            m_view_polygon_subscriber = m_node->subscribe("/udp/view_polygon", 10, &ROSLink::viewPolygonCallback, this);
            m_view_seglist_subscriber = m_node->subscribe("/udp/view_seglist", 10, &ROSLink::viewSeglistCallback, this);
            m_posmv_position = m_node->subscribe("/udp/posmv/position", 10, &ROSLink::posmvPositionCallback, this);
            m_posmv_orientation = m_node->subscribe("/udp/posmv/orientation", 10, &ROSLink::posmvOrientationCallback, this);
            m_active_publisher = m_node->advertise<std_msgs::Bool>("/udp/active",1);
            m_helmMode_publisher = m_node->advertise<std_msgs::String>("/udp/helm_mode",1);
            m_wpt_updates_publisher = m_node->advertise<std_msgs::String>("/udp/wpt_updates",1);
            m_loiter_updates_publisher = m_node->advertise<std_msgs::String>("/udp/loiter_updates",1);
            m_mission_plan_publisher = m_node->advertise<std_msgs::String>("/udp/mission_plan",1);
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

    if(m_node)
        p.setColor(Qt::darkGreen);
    else
        p.setColor(Qt::darkRed);
    p.setWidth(3);
    painter->setPen(p);
    painter->drawPath(vehicleShape());

    
    p.setColor(Qt::blue);
    p.setWidth(2);
    painter->setPen(p);
    painter->drawPath(aisShape());
    
    p.setColor(Qt::darkYellow);
    p.setWidth(4);
    painter->setPen(p);
    painter->drawPath(viewShape());

    p.setColor(Qt::darkBlue);
    p.setWidth(5);
    painter->setPen(p);
    painter->drawPath(baseShape());
    p.setWidth(3);
    p.setColor(Qt::lightGray);
    painter->setPen(p);
    painter->drawPath(baseShape());
    
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
    ret.addPath(aisShape());
    ret.addPath(viewShape());
    ret.addPath(baseShape());
        
    return ret;
}

QPainterPath ROSLink::vehicleShape() const
{
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
        
        if(m_pixel_size > .5)
            drawTriangle(ret,m_location,m_heading,m_pixel_size);
        else
            drawShipOutline(ret,m_location,m_heading,3,.8,.8,1);

        
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
        
        if(m_pixel_size > .5)
            drawTriangle(ret,m_posmv_location,m_heading,m_pixel_size);
        else
            drawShipOutline(ret,m_posmv_location,m_posmv_heading,2,.8,.8,2);
    }
    return ret;
}


QPainterPath ROSLink::baseShape() const
{
    QPainterPath ret;
    if (m_local_base_location_history.size() > 1)
    {
        auto p = m_local_base_location_history.begin();
        ret.moveTo(*p);
        p++;
        while(p != m_local_base_location_history.end())
        {
            ret.lineTo(*p);
            p++;
        }
        auto last = *(m_local_base_location_history.rbegin());
        
        if(m_pixel_size > 1)
            drawTriangle(ret,m_base_location,m_base_heading,m_pixel_size);
        else
            // fairweather estimates: 70m by 12.70m
            drawShipOutline(ret,m_base_location,m_base_heading,20,6.35,6.35,49);

    }
    return ret;
}


QPainterPath ROSLink::aisShape() const
{
    QPainterPath ret;
    for(auto contactList: m_contacts)
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

        if(m_pixel_size > 1)
            drawTriangle(ret,last->location,last->heading,m_pixel_size);
        else
            drawShipOutline(ret,last->location,last->heading,last->dimension_to_bow,last->dimension_to_port,last->dimension_to_stbd,last->dimension_to_stern);
    }
        
    return ret;
}

QPainterPath ROSLink::viewShape() const

{
    QPainterPath ret;
    
    if(m_view_point_active)
        ret.addEllipse(m_local_view_point,4*m_pixel_size,4*m_pixel_size);
    
    if(m_view_seglist_active && !m_local_view_seglist.empty())
    {
        auto p = m_local_view_seglist.begin();
        ret.moveTo(*p);
        p++;
        while(p != m_local_view_seglist.end())
        {
            ret.lineTo(*p);
            p++;
        }
    }

    if(m_view_polygon_active && !m_local_view_polygon.empty())
    {
        auto p = m_local_view_polygon.begin();
        ret.moveTo(*p);
        p++;
        while(p != m_local_view_polygon.end())
        {
            ret.lineTo(*p);
            p++;
        }
        ret.lineTo(m_local_view_polygon.front());
    }

    
    return ret;
}


void ROSLink::drawTriangle(QPainterPath& path, const QGeoCoordinate& location, double heading_degrees, double scale) const
{
    QGeoCoordinate tip = location.atDistanceAndAzimuth(15*scale,heading_degrees);
    QGeoCoordinate llcorner = location.atDistanceAndAzimuth(15*scale,heading_degrees-150);
    QGeoCoordinate lrcorner = location.atDistanceAndAzimuth(15*scale,heading_degrees+150);

    QPointF ltip = geoToPixel(tip,autonomousVehicleProject())-m_local_reference_position;
    QPointF lllocal = geoToPixel(llcorner,autonomousVehicleProject())-m_local_reference_position;
    QPointF lrlocal = geoToPixel(lrcorner,autonomousVehicleProject())-m_local_reference_position;

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
        QPointF lllocal = geoToPixel(llcorner,autonomousVehicleProject())-m_local_reference_position;
        QPointF lrlocal = geoToPixel(lrcorner,autonomousVehicleProject())-m_local_reference_position;
        QPointF lkinkl = geoToPixel(lkink,autonomousVehicleProject())-m_local_reference_position;
        QPointF rkinkl = geoToPixel(rkink,autonomousVehicleProject())-m_local_reference_position;
        QPointF bowl = geoToPixel(bow,autonomousVehicleProject())-m_local_reference_position;
        
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


void ROSLink::aisCallback(const asv_msgs::AISContact::ConstPtr& message)
{
    qDebug() << message->mmsi << ": " << message->name.c_str() << " heading: " << message->heading << " cog: " << message->cog << " dimensions: port: " << message->dimension_to_port << " strbd: " << message->dimension_to_stbd << " bow " << message->dimension_to_bow << " stern: " << message->dimension_to_stern;
    ROSAISContact *c = new ROSAISContact();
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
    for(auto kv: message->values)
    {
        status_string += kv.key.c_str();
        status_string += ": ";
        status_string += kv.value.c_str();
        status_string += "\n";
    }
    
    QMetaObject::invokeMethod(m_details,"updateVehicleStatus", Qt::QueuedConnection, Q_ARG(QString const&, status_string));
    QMetaObject::invokeMethod(this,"updateHeartbeatTimes", Qt::QueuedConnection, Q_ARG(ros::Time const&, last_heartbeat_timestamp), Q_ARG(ros::Time const&, last_heartbeat_receive_time));
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

        m_details->heartbeatDelay(diff.toSec());
    }
    else
        m_details->heartbeatDelay(1000.0);
}


QGeoCoordinate ROSLink::rosMapToGeo(const QPointF& location) const
{
    gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon> gr(m_origin.latitude(),m_origin.longitude(),m_origin.altitude());
    gz4d::geo::LocalENU<> geoReference(gr);    //geoReference = gz4d::geo::LocalENU<>(gr);
    auto ecef = geoReference.toECEF(gz4d::Point<double>(location.x(),location.y(),0.0));
    gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon>ret(ecef);
    return QGeoCoordinate(ret[0],ret[1]);
}


void ROSLink::sendWaypoints(const QList<QGeoCoordinate>& waypoints)
{
    qDebug() << "origin: " << m_origin;
    gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon> gr(m_origin.latitude(),m_origin.longitude(),m_origin.altitude());
    qDebug() << "as gz4d::geo::Point: " << gr[0] << ", " << gr[1] << ", " << gr[2];
    gz4d::geo::LocalENU<> geoReference(gr);    //geoReference = gz4d::geo::LocalENU<>(gr);

    std::stringstream updates;
    updates << "points = " << std::fixed;
    for(auto wp: waypoints)
    {
        qDebug() << "wp: " << wp;
        gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon> ggp(wp.latitude(),wp.longitude(),0.0);
        qDebug() << "ggp: " << ggp[0] << ", " << ggp[1] << ", " << ggp[2];
        gz4d::geo::Point<double,gz4d::geo::WGS84::ECEF> ecef(ggp);
        qDebug() << "ecef: " << ecef[0] << ", " << ecef[1] << ", " << ecef[2];
        gz4d::Point<double> position = geoReference.toLocal(ecef);
        updates << position[0] << ", " << position[1] << ":";
    }
    
    std_msgs::String rosUpdates;
    rosUpdates.data = updates.str();
    if(m_node)
        m_wpt_updates_publisher.publish(rosUpdates);
}

void ROSLink::sendMissionPlan(const QString& plan)
{
    std_msgs::String mp;
    mp.data = plan.toStdString();
    if(m_node)
        m_mission_plan_publisher.publish(mp);
}


void ROSLink::sendLoiter(const QGeoCoordinate& loiterLocation)
{
    qDebug() << "origin: " << m_origin;
    gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon> gr(m_origin.latitude(),m_origin.longitude(),m_origin.altitude());
    qDebug() << "as gz4d::geo::Point: " << gr[0] << ", " << gr[1] << ", " << gr[2];
    gz4d::geo::LocalENU<> geoReference(gr);    //geoReference = gz4d::geo::LocalENU<>(gr);

    std::stringstream updates;
    updates << "center_assign = ";

    qDebug() << "loiterLocation: " << loiterLocation;
    gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon> ggp(loiterLocation.latitude(),loiterLocation.longitude(),0.0);
    qDebug() << "ggp: " << ggp[0] << ", " << ggp[1] << ", " << ggp[2];
    gz4d::geo::Point<double,gz4d::geo::WGS84::ECEF> ecef(ggp);
    qDebug() << "ecef: " << ecef[0] << ", " << ecef[1] << ", " << ecef[2];
    gz4d::Point<double> position = geoReference.toLocal(ecef);
    updates << position[0] << ", " << position[1] << ":";
    
    std_msgs::String rosUpdates;
    rosUpdates.data = updates.str();
    m_loiter_updates_publisher.publish(rosUpdates);
}

void ROSLink::sendGoto(const QGeoCoordinate& gotoLocation)
{
    QList<QGeoCoordinate> wps;
    wps.append(gotoLocation);
    sendWaypoints(wps);
}


void ROSLink::updateLocation(const QGeoCoordinate& location)
{
    prepareGeometryChange();
    if(m_have_local_reference)
    {
        m_location_history.push_back(location);
        m_local_location_history.push_back(geoToPixel(location,autonomousVehicleProject())-m_local_reference_position);
        while (m_local_location_history.size()>500)
            m_local_location_history.pop_front();
        m_location = location;
        update();
    }
}

void ROSLink::updatePosmvLocation(const QGeoCoordinate& location)
{
    prepareGeometryChange();
    if(m_have_local_reference)
    {
        m_posmv_location_history.push_back(location);
        m_local_posmv_location_history.push_back(geoToPixel(location,autonomousVehicleProject())-m_local_reference_position);
        while (m_local_posmv_location_history.size()>1500)
            m_local_posmv_location_history.pop_front();
        m_posmv_location = location;
        update();
    }
}


void ROSLink::updateBaseLocation(const QGeoCoordinate& location)
{
    prepareGeometryChange();
    if(m_have_local_reference)
    {
        m_base_location_history.push_back(location);
        m_local_base_location_history.push_back(geoToPixel(location,autonomousVehicleProject())-m_local_reference_position);
        while (m_local_base_location_history.size()>100)
            m_local_base_location_history.pop_front();
        m_base_location = location;
        update();
    }
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
    //c->setParent(this);
    prepareGeometryChange();
    if(m_have_local_reference)
    {
        c->location_local = geoToPixel(c->location,autonomousVehicleProject())-m_local_reference_position;
    }
    m_contacts[c->mmsi].push_back(c);
    while(m_contacts[c->mmsi].size() > 50)
        m_contacts[c->mmsi].pop_front();
    update();
}

void ROSLink::updateBackground(BackgroundRaster *bgr)
{
    m_pixel_size = m_map_scale/bgr->pixelSize();
    setParentItem(bgr);
    recalculatePositions();
}

void ROSLink::recalculatePositions()
{
    setPos(geoToPixel(m_origin,autonomousVehicleProject()));
    if(m_have_local_reference)
    {
        m_local_reference_position = geoToPixel(m_origin,autonomousVehicleProject());
        m_local_location_history.clear();
        for(auto l: m_location_history)
        {
            m_local_location_history.push_back(geoToPixel(l,autonomousVehicleProject())-m_local_reference_position);            
        }
        m_local_posmv_location_history.clear();
        for(auto l: m_posmv_location_history)
        {
            m_local_posmv_location_history.push_back(geoToPixel(l,autonomousVehicleProject())-m_local_reference_position);            
        }
        m_local_base_location_history.clear();
        for(auto l: m_base_location_history)
        {
            m_local_base_location_history.push_back(geoToPixel(l,autonomousVehicleProject())-m_local_reference_position);            
        }
        for(auto contactList: m_contacts)
        {
            for(auto contact: contactList.second)
                contact->location_local = geoToPixel(contact->location,autonomousVehicleProject())-m_local_reference_position;
        }
    }
}


bool ROSLink::active() const
{
    return m_active;
}

void ROSLink::setActive(bool active)
{
    m_active = active;
    std_msgs::Bool b;
    b.data = active;
    m_active_publisher.publish(b);
}

const std::string& ROSLink::helmMode() const
{
    return m_helmMode;
}

void ROSLink::setHelmMode(const std::string& helmMode)
{
    m_helmMode = helmMode;
    std_msgs::String hm;
    hm.data = helmMode;
    m_helmMode_publisher.publish(hm);
}

AutonomousVehicleProject * ROSLink::autonomousVehicleProject() const
{
    return qobject_cast<AutonomousVehicleProject*>(parent());
}

QMap<QString, QString> ROSLink::parseViewString(const QString& vs) const
{
    // return key-value pairs.
    // The string has ',' as a separator for the pairs, but also for elements of lists
    // that may be the value. For that reason, spliting on ',' won't always work if lists
    // as values aren't considered. Spliting on '=' first, then grabing the next key from
    // the end of the last value is the strategy used here.
    QMap<QString, QString> ret;
    QString key;
    QString value;
    QStringList parts = vs.split('=');
    if(!parts.empty())
    {
        key = parts[0];
        QStringList::iterator parts_iterator = parts.begin();
        parts_iterator++;
        while(parts_iterator != parts.end())
        {
            QString part = *parts_iterator;
            QStringList comma_separated_parts = part.split(',');
            if(comma_separated_parts.size() == 1)
            {
                value = comma_separated_parts[0];
                ret[key] = value;
            }
            else
            {
                QString next_key = comma_separated_parts.back();
                comma_separated_parts.removeLast();
                value = comma_separated_parts.join(',');
                ret[key] = value;
                key = next_key;
            }
            parts_iterator++;
        }
    }
    return ret;
}

QList<QPointF> ROSLink::parseViewPointList(const QString& pointList) const
{
    QList<QPointF> ret;
    QString trimmed = pointList.mid(1,pointList.size()-2);
    auto pairs = trimmed.split(':');
    for (auto pair: pairs)
    {
        auto xy =  pair.split(',');
        ret.append(QPointF(xy[0].toFloat(),xy[1].toFloat()));
    }
    return ret;
}


void ROSLink::viewPointCallback(const std_msgs::String::ConstPtr& message)
{
    //qDebug() << "view point" << std::string(message->data).c_str();
    auto parsed = parseViewString(QString(std::string(message->data).c_str()));
    //qDebug() << parsed;
    if(parsed.contains("active"))
        m_view_point_active = (parsed["active"] == "true");
    else
        m_view_point_active = true;
    m_view_point = rosMapToGeo(QPointF(parsed["x"].toFloat(),parsed["y"].toFloat()));
    //qDebug() << m_view_point;
    m_local_view_point = geoToPixel(m_view_point,autonomousVehicleProject())-m_local_reference_position;
    //qDebug() << m_local_view_point;
}

void ROSLink::viewPolygonCallback(const std_msgs::String::ConstPtr& message)
{
    qDebug() << "view polygon" << std::string(message->data).c_str();
    auto parsed = parseViewString(QString(std::string(message->data).c_str()));
    qDebug() << parsed;
    if(parsed.contains("active"))
        m_view_polygon_active = (parsed["active"] == "true");
    else
        m_view_polygon_active = true;
    auto points = parseViewPointList(parsed["pts"]);
    //qDebug() << points;
    m_view_polygon.clear();
    m_local_view_polygon.clear();
    for(auto p: points)
    {
        m_view_polygon.append(rosMapToGeo(p));
        m_local_view_polygon.append(geoToPixel(m_view_polygon.back(),autonomousVehicleProject())-m_local_reference_position);
    }
}

void ROSLink::viewSeglistCallback(const std_msgs::String::ConstPtr& message)
{
    //qDebug() << "view seglist" << std::string(message->data).c_str();
    auto parsed = parseViewString(QString(std::string(message->data).c_str()));
    //qDebug() << parsed;
    if(parsed.contains("active"))
        m_view_seglist_active = (parsed["active"] == "true");
    else
        m_view_seglist_active = true;
    auto points = parseViewPointList(parsed["pts"]);
    //qDebug() << points;
    m_view_seglist.clear();
    m_local_view_seglist.clear();
    for(auto p: points)
    {
        m_view_seglist.append(rosMapToGeo(p));
        m_local_view_seglist.append(geoToPixel(m_view_seglist.back(),autonomousVehicleProject())-m_local_reference_position);
    }
}

void ROSLink::updateMapScale(qreal scale)
{
    m_map_scale = scale;
    qDebug() << "scale: " << m_map_scale;
    m_pixel_size = autonomousVehicleProject()->getBackgroundRaster()->pixelSize()/m_map_scale;
    qDebug() << "scaled pixel size: " << m_pixel_size;
}

