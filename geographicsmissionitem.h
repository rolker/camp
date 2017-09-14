#ifndef GEOGRAPHICSMISSIONITEM_H
#define GEOGRAPHICSMISSIONITEM_H

#include "missionitem.h"
#include "geographicsitem.h"

class BackgroundRaster;

class GeoGraphicsMissionItem : public MissionItem, public GeoGraphicsItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    explicit GeoGraphicsMissionItem(QObject *parent = 0, QGraphicsItem *parentItem =0);
    
public slots:
    void updateBackground(BackgroundRaster * bg);
    
};

#endif // GEOGRAPHICSMISSIONITEM_H
