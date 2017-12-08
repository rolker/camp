#include "rosnode.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <QDebug>
#include <QPainter>
#include <QGraphicsSvgItem>
#include "autonomousvehicleproject.h"
#include "gz4d_geo.h"

ROSNode::ROSNode(QObject* parent, QGraphicsItem* parentItem): GeoGraphicsMissionItem(parent,parentItem),m_spinner(0),m_heading(0.0),m_active(false),m_helmMode("standby")
{
    //QGraphicsSvgItem *symbol = new QGraphicsSvgItem(this);
    //symbol->setSharedRenderer(autonomousVehicleProject()->symbols());
    //symbol->setElementId("Vessel");
    //symbol->setFlag(QGraphicsItem::ItemIgnoresTransformations);

    
    qRegisterMetaType<QGeoCoordinate>();
    m_geopoint_subscriber = m_node.subscribe("/zmq/position", 10, &ROSNode::geoPointStampedCallback, this);
    m_origin_subscriber = m_node.subscribe("/zmq/origin", 10, &ROSNode::originCallback, this);
    m_active_publisher = m_node.advertise<std_msgs::Bool>("/zmq/active",1);
    m_helmMode_publisher = m_node.advertise<std_msgs::String>("/zmq/helm_mode",1);
    m_wpt_updates_publisher = m_node.advertise<std_msgs::String>("/zmq/wpt_updates",1);
    m_spinner.start();

}

QRectF ROSNode::boundingRect() const
{
    return shape().boundingRect();
}

void ROSNode::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    painter->save();

    QPen p;
    p.setColor(Qt::darkGreen);
    p.setCosmetic(true);
    p.setWidth(3);
    painter->setPen(p);
    painter->drawPath(shape());

    painter->restore();
  
    
}

QPainterPath ROSNode::shape() const
{
    if (m_local_location_history.size() > 1)
    {
        QPainterPath ret;
        auto p = m_local_location_history.begin();
        p++;
        while(p != m_local_location_history.end())
        {
            ret.lineTo(*p);
            p++;
        }
        auto last = *(m_local_location_history.rbegin());
        ret.addRoundedRect(last.x()-10,last.y()-10,20,20,8,8);
        return ret;
    }
    return QPainterPath();
}


void ROSNode::read(const QJsonObject& json)
{
}

void ROSNode::write(QJsonObject& json) const
{
}

void ROSNode::geoPointStampedCallback(const geographic_msgs::GeoPointStamped::ConstPtr& message)
{
    QGeoCoordinate position(message->position.latitude,message->position.longitude,message->position.altitude);
    QMetaObject::invokeMethod(this,"updateLocation", Qt::QueuedConnection, Q_ARG(QGeoCoordinate, position));
}

void ROSNode::originCallback(const geographic_msgs::GeoPoint::ConstPtr& message)
{
    m_origin.setLatitude(message->latitude);
    m_origin.setLongitude(message->longitude);
    m_origin.setAltitude(message->altitude);
    //qDebug() << m_origin;
}

void ROSNode::sendWaypoints(const QList<QGeoCoordinate>& waypoints)
{
    qDebug() << "origin: " << m_origin;
    gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon> gr(m_origin.latitude(),m_origin.longitude(),m_origin.altitude());
    qDebug() << "as gz4d::geo::Point: " << gr[0] << ", " << gr[1] << ", " << gr[2];
    gz4d::geo::LocalENU<> geoReference(gr);    //geoReference = gz4d::geo::LocalENU<>(gr);

    std::stringstream updates;
    updates << "points = ";
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
    m_wpt_updates_publisher.publish(rosUpdates);
}


void ROSNode::updateLocation(const QGeoCoordinate& location)
{
    if(m_location_history.empty())
    {
        setPos(geoToPixel(location,autonomousVehicleProject()));
        m_local_reference_position = geoToPixel(location,autonomousVehicleProject());
    }
    m_location_history.push_back(location);
    m_local_location_history.push_back(geoToPixel(location,autonomousVehicleProject())-m_local_reference_position);
    m_location = location;
    update();
}

void ROSNode::updateProjectedPoints()
{
    setPos(geoToPixel(m_location,autonomousVehicleProject()));
}

bool ROSNode::active() const
{
    return m_active;
}

void ROSNode::setActive(bool active)
{
    m_active = active;
    std_msgs::Bool b;
    b.data = active;
    m_active_publisher.publish(b);
}

const std::string& ROSNode::helmMode() const
{
    return m_helmMode;
}

void ROSNode::setHelmMode(const std::string& helmMode)
{
    m_helmMode = helmMode;
    std_msgs::String hm;
    hm.data = helmMode;
    m_helmMode_publisher.publish(hm);
}


