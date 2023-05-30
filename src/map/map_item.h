#ifndef MAP_MAP_ITEM_H
#define MAP_MAP_ITEM_H

#include "item_types.h"

class QMimeData;

namespace map
{

class Map;

// Base class for map components.
class MapItem: public QGraphicsObject
{
  Q_OBJECT
public:
  MapItem(MapItem* parent_item, const QString& object_name);
  virtual ~MapItem() = default;

  enum { Type = MapItemType };

  int type() const override
  {
    // Enable the use of qgraphicsitem_cast with this item.
    return Type;
  }

  // QGraphicsItem overrides
  QRectF boundingRect() const override;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;


  // Returns a pointer to the parent item if it is a MapItem.
  // If the parent is not a MapItem or is null, return nullptr.
  MapItem* parentMapItem() const;

  // Return the child items that are MapItems.
  virtual QList<MapItem*> childMapItems() const;
  QList<const MapItem*> childConstMapItems() const;

  static constexpr char MimeType[] = "application/camp.map_item.pointer";

  virtual bool canDropMimeData(const QMimeData* data, Qt::DropAction action, int row, int col) const;

public slots:
  void setOpacity(qreal opacity);

protected:
  // Allows item to modify Model/View flags.
  virtual void updateFlags(Qt::ItemFlags& flags) const;

private:
  // Make sure Map can create a top level item without a parent.
  friend class Map;
  MapItem(const QString& object_name);

};

} // namespace map

#endif
