#include "geographicsmissionitem.h"

#include "backgroundraster.h"
#include <QDebug>

GeoGraphicsMissionItem::GeoGraphicsMissionItem(QObject* parent, QGraphicsItem* parentItem):MissionItem(parent), GeoGraphicsItem(parentItem)
{
    setAcceptHoverEvents(true);
    setOpacity(.5);
}

void GeoGraphicsMissionItem::updateBackground(BackgroundRaster* bg)
{
    setParentItem(bg);
    updateProjectedPoints();
}

void GeoGraphicsMissionItem::hoverEnterEvent(QGraphicsSceneHoverEvent* event)
{
    //qDebug() << "Enter item!";
    setOpacity(1.0);
}

void GeoGraphicsMissionItem::hoverLeaveEvent(QGraphicsSceneHoverEvent* event)
{
    //qDebug() << "Leave item!";
    setOpacity(.5);
}
