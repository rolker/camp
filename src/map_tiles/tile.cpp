#include "tile.h"
#include <cmath>
#include "map_tiles.h"
#include <QStyleOptionGraphicsItem>

namespace map_tiles
{

const QRectF Tile::bounding_rect_;

Tile::Tile(QGraphicsItem *parentItem, TileAddress address):
  QGraphicsObject(parentItem), address_(address)
{
  new QGraphicsPixmapItem(this);
}

QRectF Tile::boundingRect() const
{
  return bounding_rect_;
}

void Tile::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,QWidget *widget)
{
  auto lod = QStyleOptionGraphicsItem::levelOfDetailFromTransform(painter->worldTransform());

  auto map_tiles = parentMapTiles();
  if(!map_tiles)
    return;

  if(!pixmap_load_request_sent_ && address_.zoomLevel() >= map_tiles->minimumZoomLevel())
  {
    map_tiles->loadTile(address_);
    pixmap_load_request_sent_ = true;
  }

  if(lod > 2.0 || address_.zoomLevel() < map_tiles->minimumZoomLevel())
  {
    if(address_.zoomLevel() < std::min(TileAddress::max_zoom_level, map_tiles->maximumZoomLevel()) && childTiles().empty())
    {
      for(int x = 0; x < 2; x++)
        for(int y = 0; y < 2; y++ )
        {
          auto child_tile = new Tile(this, address_.child(x,y));
          child_tile->setScale(0.5);
        }
      updateLayout(map_tiles->flipY());
    }

    pixmapItem()->setVisible(lod < 4.0);
  }
  else
  {
    // hide pixmap if we are zoomed out too much
    if(lod < 1.0 && address_.zoomLevel() > map_tiles->minimumZoomLevel())
      pixmapItem()->setVisible(false);
    else
      pixmapItem()->setVisible(true);
  }
}

void Tile::updateLayout(bool flip_y)
{
  for(auto tile: childTiles())
  {
    int x = tile->address_.index().x()%2;
    int y = tile->address_.index().y()%2;
    if(flip_y)
      y = 1-y;
    tile->setPos(x*web_mercator::tile_size/2.0, y*web_mercator::tile_size/2.0);
    tile->updateLayout(flip_y);
  }
}

MapTiles * Tile::parentMapTiles() const
{
  auto parent = parentItem();
  if (parent)
  {
    auto map_tiles = qgraphicsitem_cast<MapTiles*>(parent);
    if(map_tiles)
      return map_tiles;
    auto parent_tile = qgraphicsitem_cast<Tile*>(parent);
    if(parent_tile)
      return parent_tile->parentMapTiles();
  }
  return nullptr;
}

QList<Tile*> Tile::childTiles() const
{
  QList<Tile*> ret;
  for(auto child: childItems())
  {
    auto tile = qgraphicsitem_cast<Tile*>(child);
    if(tile)
      ret.append(tile);
  }
  return ret;
}

QGraphicsPixmapItem* Tile::pixmapItem() const
{
  for(auto child: childItems())
  {
    auto pixmap_item = qgraphicsitem_cast<QGraphicsPixmapItem*>(child);
    if(pixmap_item)
      return pixmap_item;
  }
  return nullptr;
}

void Tile::updatePixmap(QPixmap pixmap)
{
  auto pixmap_item = pixmapItem();
  if(pixmap_item)
  {
    pixmap_item->setPixmap(pixmap);
    // Scale if our pixmap is not standard tile size.
    if(pixmap.width() != web_mercator::tile_size || pixmap.height() != web_mercator::tile_size)
    {
      if(pixmap.width() > 0 && pixmap.height() > 0)
        pixmap_item->setTransform(QTransform::fromScale(web_mercator::tile_size/double(pixmap.width()),web_mercator::tile_size/double(pixmap.height())));
    }
  }
}

const TileAddress& Tile::address() const
{
  return address_;
}

Tile* Tile::find(const TileAddress& address)
{
  if(address == address_)
    return this;
  if(address.descendentOf(address_))
    for(auto child: childTiles())
    {
      auto found = child->find(address);
      if(found)
        return found;
    }
  return nullptr;
}

} // namepsace map_tiles
