#include "map_item.h"
#include "map.h"
#include <QApplication>
#include <QTimer>

#include <cassert>

namespace map
{

const char MapItem::MimeType[];

MapItem::MapItem(MapItem* parent_item, const QString& object_name)
{
  assert(parent_item!=nullptr);
  setObjectName(object_name);

  Map * parent_map = parent_item->parentMap();
  if(parent_map)
    parent_map->setMapItemParent(this, parent_item);
  else
    setParentItem(parent_item);

  connect(QApplication::instance(), &QCoreApplication::aboutToQuit, this, &MapItem::applicationQuitting);

  QTimer::singleShot( 0, this, &MapItem::itemConstructed); 
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


void MapItem::setObjectName(const QString& name)
{
  QGraphicsObject::setObjectName(name);

  auto map = parentMap();
  if(map)
    map->updateDisplay(this);
}


Map* MapItem::parentMap() const
{
  auto parent_scene = scene();
  if(parent_scene)
    return qobject_cast<Map*>(parent_scene->parent());
  return nullptr;
}

void MapItem::setStatus(const QString& status)
{
  if(status != status_)
  {
    status_ = status;

    auto map = parentMap();
    if(map)
      map->updateDisplay(this);
  }
}


const QString& MapItem::status() const
{
  return status_;
}


void MapItem::contextMenu(QMenu* menu)
{
}

void MapItem::itemConstructed()
{
  readSettings();
}

void MapItem::applicationQuitting()
{
  writeSettings();
}

void MapItem::readSettings()
{
}

void MapItem::writeSettings()
{
}


} // namepsace map
