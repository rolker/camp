#include "geographicsmissionitem.h"

#include "backgroundraster.h"

GeoGraphicsMissionItem::GeoGraphicsMissionItem(QObject* parent, QGraphicsItem* parentItem):MissionItem(parent), GeoGraphicsItem(parentItem)
{
}

void GeoGraphicsMissionItem::updateBackground(BackgroundRaster* bg)
{
    setParentItem(bg);
    updateProjectedPoints();
}
