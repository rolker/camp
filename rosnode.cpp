#include "rosnode.h"
//#include <QDebug>
#include <QPainter>

ROSNode::ROSNode(QObject* parent, QGraphicsItem* parentItem): GeoGraphicsMissionItem(parent,parentItem),spinner(0)
{
    qRegisterMetaType<QGeoCoordinate>();
    subscriber = node.subscribe("/sensor/vehicle/position", 10, &ROSNode::positionCallback, this);
    spinner.start();

}

QRectF ROSNode::boundingRect() const
{
    qreal penWidth = 1;
    return QRectF(-10 - penWidth / 2, -10 - penWidth / 2, 20 + penWidth, 20 + penWidth);

    //return shape().boundingRect();
}

void ROSNode::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    painter->save();

    QPen p;
    p.setColor(Qt::red);
    p.setCosmetic(true);
    p.setWidth(3);
    painter->setPen(p);

    painter->drawRoundedRect(-10,-10,20,20,8,8);

    painter->restore();
}


void ROSNode::read(const QJsonObject& json)
{
}

void ROSNode::write(QJsonObject& json) const
{
}

void ROSNode::positionCallback(const asv_msgs::BasicPositionStamped::ConstPtr& message)
{
    //qDebug() << "ROS: " << message->basic_position.position.latitude << ", " << message->basic_position.position.longitude;
    QGeoCoordinate position(message->basic_position.position.latitude,message->basic_position.position.longitude);
    QMetaObject::invokeMethod(this,"updateLocation", Qt::QueuedConnection, Q_ARG(QGeoCoordinate, position));
}

void ROSNode::updateLocation(const QGeoCoordinate& location)
{
    setPos(geoToPixel(location,autonomousVehicleProject()));
    m_location = location;
}

void ROSNode::updateProjectedPoints()
{
    setPos(geoToPixel(m_location,autonomousVehicleProject()));
}
