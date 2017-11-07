#include "rosnode.h"
//#include <QDebug>
#include <QPainter>
#include <QGraphicsSvgItem>
#include "autonomousvehicleproject.h"

ROSNode::ROSNode(QObject* parent, QGraphicsItem* parentItem): GeoGraphicsMissionItem(parent,parentItem),m_spinner(0),m_heading(0.0)
{
    //QGraphicsSvgItem *symbol = new QGraphicsSvgItem(this);
    //symbol->setSharedRenderer(autonomousVehicleProject()->symbols());
    //symbol->setElementId("Vessel");
    //symbol->setFlag(QGraphicsItem::ItemIgnoresTransformations);

    
    qRegisterMetaType<QGeoCoordinate>();
    m_position_subscriber = m_node.subscribe("/sensor/vehicle/position", 10, &ROSNode::positionCallback, this);
    m_heading_subscriber = m_node.subscribe("/sensor/vehicle/heading", 10, &ROSNode::headingCallback, this);
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
    p.setColor(Qt::red);
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

void ROSNode::positionCallback(const asv_msgs::BasicPositionStamped::ConstPtr& message)
{
    QGeoCoordinate position(message->basic_position.position.latitude,message->basic_position.position.longitude);
    QMetaObject::invokeMethod(this,"updateLocation", Qt::QueuedConnection, Q_ARG(QGeoCoordinate, position));
}

void ROSNode::headingCallback(const asv_msgs::HeadingStamped::ConstPtr& message)
{
    m_heading = message->heading.heading;
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
