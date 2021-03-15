#include "geographicsitem.h"
#include "backgroundraster.h"
#include "autonomousvehicleproject.h"
#include "missionitem.h"
#include <QGraphicsSimpleTextItem>
#include <QFont>
#include <QBrush>
#include <QPen>
#include <QDebug>

GeoGraphicsItem::GeoGraphicsItem(QGraphicsItem *parentItem): QGraphicsItem(parentItem), m_showLabelFlag(false)
{
    m_label = new QGraphicsSimpleTextItem(this);
    m_label->setFlag(GraphicsItemFlag::ItemIgnoresTransformations);
    auto font = m_label->font();
    font.setPointSize(20);
    font.setBold(true);
    m_label->setFont(font);
    m_label->setBrush(QBrush(QColor("black")));
    QPen p(QColor("white"));
    p.setWidth(2);
    m_label->setPen(p);
    //m_label->setFlag(QGraphicsItem::ItemIsMovable); this caused other elements to move while trying to move the label!
}

QPointF GeoGraphicsItem::geoToPixel(const QGeoCoordinate &point, AutonomousVehicleProject *p) const
{
    if(p)
    {
        BackgroundRaster *bg = p->getBackgroundRaster();
        if(bg)
        {
            QPointF ret = bg->geoToPixel(point);
            QGraphicsItem *pi = parentItem();
            if(pi)
            {
                return ret - pi->scenePos();
            }
            return ret;
        }
    }
    return QPointF();
}

void GeoGraphicsItem::prepareGeometryChange()
{
    QGraphicsItem::prepareGeometryChange();
}

void GeoGraphicsItem::setLabel(const QString &label)
{
    m_labelText = label;
    if(m_showLabelFlag)
        m_label->setText(m_labelText);
}

void GeoGraphicsItem::setLabelPosition(QPointF pos)
{
    m_label->setPos(pos);
}

bool GeoGraphicsItem::showLabelFlag() const
{
    return m_showLabelFlag;
}

void GeoGraphicsItem::setShowLabelFlag(bool show)
{
    m_showLabelFlag = show;
    if(show)
        m_label->setText(m_labelText);
    else
        m_label->setText("");
}



