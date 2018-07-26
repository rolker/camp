#include "linestring.h"
#include <QPainter>
#include "point.h"

LineString::LineString(MissionItem* parent):GeoGraphicsMissionItem(parent)
{

}

void LineString::updateProjectedPoints()
{
    for(auto p: m_points)
        p.pos = geoToPixel(p.location,autonomousVehicleProject());
    updateBBox();
}

void LineString::write(QJsonObject& json) const
{

}

void LineString::writeToMissionPlan(QJsonArray& navArray) const
{
}


void LineString::read(const QJsonObject& json)
{

}

QRectF LineString::boundingRect() const
{
    return m_bbox;
}

void LineString::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{

    if (m_points.length() > 1)
    {
        painter->save();

        QPen p;
        p.setColor(Qt::red);
        p.setCosmetic(true);
        p.setWidth(3);
        painter->setPen(p);

        auto first = m_points.begin();
        auto second = first;
        second++;
        while(second != m_points.end())
        {
            painter->drawLine(first->pos,second->pos);
            first++;
            second++;
        }

        painter->restore();

    }

}

QPainterPath LineString::shape() const
{
    if (m_points.length() > 1)
    {
        auto i = m_points.begin();
        QPainterPath ret(i->pos);
        i++;
        while(i != m_points.end())
        {
            ret.lineTo(i->pos);
            i++;
        }
        QPainterPathStroker pps;
        pps.setWidth(5);
        return pps.createStroke(ret);

    }
    return QGraphicsItem::shape();
}


QList<LocationPosition> const &LineString::points() const
{
    return m_points;
}

void LineString::addPoint(const QGeoCoordinate &location)
{
    LocationPosition lp;
    lp.location = location;
    lp.pos = geoToPixel(location,autonomousVehicleProject());
    m_points.append(lp);
    updateBBox();
}

void LineString::updateBBox()
{
    if(m_points.length() >0)
    {
        m_bbox = QRectF(m_points[0].pos,QSizeF());
        for(auto p:m_points)
            m_bbox = m_bbox.united(QRectF(p.pos,QSizeF()));
    }
    else
        m_bbox = QRectF();
}

