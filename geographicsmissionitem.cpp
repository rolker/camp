#include "geographicsmissionitem.h"

#include "backgroundraster.h"
#include <QDebug>

GeoGraphicsMissionItem::GeoGraphicsMissionItem(MissionItem* parent):MissionItem(parent)
{
    QGraphicsItem *parentItem = parent->findParentGraphicsItem();
    setParentItem(parentItem);
    setAcceptHoverEvents(true);
    setOpacity(.5);
    setFlag(QGraphicsItem::ItemIsMovable);
    setFlag(QGraphicsItem::ItemIsSelectable);
    setFlag(QGraphicsItem::ItemSendsGeometryChanges);
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

QGraphicsItem * GeoGraphicsMissionItem::findParentGraphicsItem()
{
    return this;
}
