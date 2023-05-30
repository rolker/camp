#include "map_item.h"

#include <cassert>

namespace map
{

const char MapItem::MimeType[];

MapItem::MapItem(MapItem* parent_item, const QString& object_name):
  QGraphicsObject(parent_item)
{
  assert(parent_item!=nullptr);
  setObjectName(object_name);
}

MapItem::MapItem(const QString& object_name)
{
  setObjectName(object_name);
}

QRectF MapItem::boundingRect() const
{
  return QRectF();
}


void MapItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
}


MapItem* MapItem::parentMapItem() const
{
  return dynamic_cast<MapItem*>(parentItem());
}


QList<MapItem*> MapItem::childMapItems() const
{
  QList<MapItem*> map_items;
  for(auto item: childItems())
  {
    auto map_item = dynamic_cast<MapItem*>(item);
    if(map_item)
      map_items.append(map_item);
  }
  return map_items;
}

QList<const MapItem*> MapItem::childConstMapItems() const
{
  QList<const MapItem*> map_items;
  for(auto item: this->childMapItems())
    map_items.append(item);
  return map_items;
}


void MapItem::updateFlags(Qt::ItemFlags& flags) const
{

}

bool MapItem::canDropMimeData(const QMimeData* data, Qt::DropAction action, int row, int col) const
{
  return false;
}

void MapItem::setOpacity(qreal opacity)
{
  QGraphicsItem::setOpacity(opacity);
}

} // namepsace map
