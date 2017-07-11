#include "geographicsitem.h"
#include "backgroundraster.h"
#include "autonomousvehicleproject.h"
#include <QGraphicsTextItem>
#include <QFont>

GeoGraphicsItem::GeoGraphicsItem(QObject *parent, QGraphicsItem *parentItem):MissionItem(parent), QGraphicsItem(parentItem)
{
    m_label = new QGraphicsTextItem(this);
    m_label->setFlag(GraphicsItemFlag::ItemIgnoresTransformations);
    auto font = m_label->font();
    font.setPointSize(20);
    m_label->setFont(font);
    m_label->setDefaultTextColor(QColor("blue"));
}

QPointF GeoGraphicsItem::geoToPixel(const QGeoCoordinate &point) const
{
    AutonomousVehicleProject *p = project();
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

AutonomousVehicleProject *GeoGraphicsItem::project() const
{
    AutonomousVehicleProject *ret = qobject_cast<AutonomousVehicleProject*>(parent());
    return ret;
}

void GeoGraphicsItem::setItem(QStandardItem *item)
{
    m_item = item;
}


QStandardItem * GeoGraphicsItem::item() const
{
    return m_item;
}
