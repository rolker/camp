#include "polygon.h"
#include <QPainter>
#include <QDebug>

Polygon::Polygon(MissionItem* parent):GeoGraphicsMissionItem(parent)
{
}

QRectF Polygon::boundingRect() const
{
    return m_bbox;
}

void Polygon::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    painter->save();

    QPen p;
    p.setColor(Qt::blue);
    p.setCosmetic(true);
    p.setWidth(2);
    painter->setPen(p);
    if(m_exteriorRing.length() > 1)
        painter->drawPolygon(m_exteriorPolygon);
    
    for(auto ip:m_interiorPolygons)
        painter->drawPolygon(ip);
        
    painter->restore();
}

void Polygon::read(const QJsonObject& json)
{
}

void Polygon::write(QJsonObject& json) const
{
}

void Polygon::writeToMissionPlan(QJsonArray& navArray) const
{
}


QPainterPath Polygon::shape() const
{


    
//     QPainterPath path;
//     QPolygonF p;
//     //add outer points
//     path.addPolygon(p);
//     QPainterPath inner;
//     //add inner points
//     inner.addPolygon(p);
//     path = path.subtracted(inner);
    
    // or 
    
//     QPainterPath path;
//     QRegion clip(path);
// 
//     QPainterPath sub;
//     clip-=QRegion(sub);
//                                             
//     paint.setClipRegion(clip);
// 
//     paint.fillPath(path,brush);

    
    if (m_exteriorRing.length() > 1)
    {
        auto i = m_exteriorRing.begin();
        QPainterPath ret(i->pos);
        i++;
        while(i != m_exteriorRing.end())
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

void Polygon::updateProjectedPoints()
{
    for(auto& p: m_exteriorRing)
        p.pos = geoToPixel(p.location,autonomousVehicleProject());
    for(auto& ir: m_interiorRings)
        for(auto& p: ir)
            p.pos = geoToPixel(p.location,autonomousVehicleProject());
    updateBBox();
}

void Polygon::updateBBox()
{
    if(m_exteriorRing.length() >0)
    {
        m_exteriorPolygon.clear();
        m_bbox = QRectF(m_exteriorRing[0].pos,QSizeF());
        for(auto p:m_exteriorRing)
        {
            m_bbox = m_bbox.united(QRectF(p.pos,QSizeF()));
            m_exteriorPolygon << p.pos;
        }
    }
    else
        m_bbox = QRectF();
    m_interiorPolygons.clear();
    for(auto ip: m_interiorRings)
        if(ip.length() > 1)
        {
            QPolygonF pf;
            for(auto p: ip)
                pf << p.pos;
            m_interiorPolygons.append(pf);
        }
}

void Polygon::addExteriorPoint(const QGeoCoordinate& location)
{
    LocationPosition lp;
    lp.location = location;
    lp.pos = geoToPixel(location,autonomousVehicleProject());
    m_exteriorRing.append(lp);
    //updateBBox();
}

void Polygon::addInteriorPoint(const QGeoCoordinate& location)
{
    LocationPosition lp;
    lp.location = location;
    lp.pos = geoToPixel(location,autonomousVehicleProject());
    m_interiorRings.rbegin()->append(lp);
}

void Polygon::addInteriorRing()
{
    m_interiorRings.append(QList<LocationPosition>());
}

