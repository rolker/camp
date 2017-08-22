#include "linestring.h"
#include <QPainter>
#include "point.h"

LineString::LineString(QObject* parent, QGraphicsItem* parentItem):Group(parent),GeoGraphicsItem(parentItem)
{

}

void LineString::updateProjectedPoints()
{
    for(auto p: points())
        p->updateProjectedPoints();
}

void LineString::write(QJsonObject& json) const
{

}

void LineString::read(const QJsonObject& json)
{

}

QStandardItem* LineString::createItem(const QString& label)
{
    return createItemDetails<LineString>(label);
}

QRectF LineString::boundingRect() const
{

}

void LineString::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{

    auto children = childItems();
    if (children.length() > 1)
    {
        painter->save();

        QPen p;
        p.setColor(Qt::red);
        p.setCosmetic(true);
        p.setWidth(3);
        painter->setPen(p);

        auto first = children.begin();
        auto second = first;
        second++;
        while(second != children.end())
        {
            painter->drawLine((*first)->pos(),(*second)->pos());
            first++;
            second++;
        }

        painter->restore();

    }

}

QPainterPath LineString::shape() const
{
    auto children = childItems();
    if (children.length() > 1)
    {
        auto i = children.begin();
        QPainterPath ret((*i)->pos());
        i++;
        while(i != children.end())
        {
            ret.lineTo((*i)->pos());
            i++;
        }
        QPainterPathStroker pps;
        pps.setWidth(5);
        return pps.createStroke(ret);

    }
    return QGraphicsItem::shape();
}


QList<Point *> LineString::points() const
{
    QList<Point *> ret;
    auto children = childItems();
    for(auto child: children)
    {
        Point *p = qgraphicsitem_cast<Point*>(child);
        if(p)
            ret.append(p);
    }
    return ret;
}

Point * LineString::addPoint(const QGeoCoordinate &location)
{
    Point *p = new Point(parent(),this);

    item()->appendRow(p->createItem("point"));
    p->setLocation(location);
    //emit trackLineUpdated();
    update();
    return p;
}

