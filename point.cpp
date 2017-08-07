#include "point.h"
#include <QGraphicsSvgItem>
#include "autonomousvehicleproject.h"
#include <QSvgRenderer>

Point::Point(QObject* parent, QGraphicsItem* parentItem):GeoGraphicsItem(parent, parentItem)
{
    QGraphicsSvgItem *symbol = new QGraphicsSvgItem(this);
    symbol->setSharedRenderer(autonomousVehicleProject()->symbols());
    symbol->setElementId("Square");
    symbol->setFlag(QGraphicsItem::ItemIgnoresTransformations);

}

void Point::updateProjectedPoints()
{
   setPos(geoToPixel(m_location));
}

void Point::write(QJsonObject& json) const
{

}

void Point::read(const QJsonObject& json)
{

}

QStandardItem* Point::createItem(const QString& label)
{
    return createItemDetails<Point>(label);
}

QRectF Point::boundingRect() const
{
    return childrenBoundingRect();
}

void Point::setLocation(QGeoCoordinate const &location)
{
    setPos(geoToPixel(location));
    m_location = location;
    setLabel(location.toString());
}

void Point::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
}
