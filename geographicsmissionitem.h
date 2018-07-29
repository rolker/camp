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
    explicit GeoGraphicsMissionItem(MissionItem *parent = 0);

    QGraphicsItem * findParentGraphicsItem() override;
    
    bool locked() const;
    QList<GeoGraphicsMissionItem*> childrenGeoGraphicsMissionItems() const;
    
public slots:
    void updateBackground(BackgroundRaster * bg);
    void lock();
    void unlock();
    
protected:
    void hoverEnterEvent(QGraphicsSceneHoverEvent * event) override;
    void hoverLeaveEvent(QGraphicsSceneHoverEvent * event) override;
    
    QColor m_lockedColor;
    QColor m_unlockedColor;
    
private:
    bool m_locked;
};

#endif // GEOGRAPHICSMISSIONITEM_H
