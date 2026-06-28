#include "map_item.h"
#include "map.h"
#include "../map_view/web_mercator.h"
#include <QApplication>
#include <QTimer>

#include <cassert>
#include <QDebug>

namespace camp
{
namespace map
{

const char MapItem::MimeType[];

MapItem::MapItem(MapItem* parent_item, const QString& object_name)
{
  assert(parent_item!=nullptr);
  setObjectName(object_name);

  setParentMapItem(parent_item);

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

void MapItem::setWebMercatorPositionAndScale(const QPointF& position, double unit_size_in_meters)
{
  assert(unit_size_in_meters > 0.0);
  auto map_distortion = web_mercator::metersPerUnit(position);
  double scale = unit_size_in_meters/map_distortion;
  setTransform(QTransform::fromScale(scale, scale));
  setPos(position);
}

QString MapItem::itemID() const
{
  QString id;
  auto parent = parentMapItem();
  if(parent)
    id = parent->itemID() + "/";
  id += objectName();
  return id;
}

void MapItem::setParentMapItem(MapItem* parent_item)
{
  Map * existing_parent_map = parentMap();
  Map * new_parent_map = nullptr;
  if(parent_item)
  {
    new_parent_map = parent_item->parentMap();
  }

  if(existing_parent_map && existing_parent_map != new_parent_map)
    existing_parent_map->setMapItemParent(this, nullptr);

  if(new_parent_map && existing_parent_map != new_parent_map)
    new_parent_map->setMapItemParent(this, parent_item);
  else
    QGraphicsItem::setParentItem(parent_item);
}

MapItem* MapItem::parentMapItem() const
{
  return dynamic_cast<MapItem*>(parentItem());
}


QList<MapItem*> MapItem::childMapItems(bool recursive) const
{
  QList<MapItem*> map_items;
  for(auto item: childItems())
  {
    auto map_item = dynamic_cast<MapItem*>(item);
    if(map_item)
    {
      map_items.append(map_item);
      if(recursive)
      {
        auto child_map_items = map_item->childMapItems(true);
        map_items.append(child_map_items);
      }
    }
  }
  return map_items;
}


QList<const MapItem*> MapItem::childConstMapItems(bool recursive) const
{
  QList<const MapItem*> map_items;
  for(auto item: this->childMapItems(recursive))
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

void MapItem::contextMenuForItem(MapItem* item, QMenu* menu)
{
  qDebug() << "MapItem::contextMenuForItem" << item << "is asking" << this << "for a context menu";
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


} // namespace map
} // namespace camp
