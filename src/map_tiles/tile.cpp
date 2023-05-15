#include "tile.h"
#include <cmath>
#include "map_tiles.h"
#include <QtConcurrent>

namespace map_tiles
{

const QRectF Tile::bounding_rect_;

Tile::Tile(const ViewContext& view_context, QGraphicsItem *parentItem, TileAddress address):
  QGraphicsObject(parentItem), address_(address), view_context_(view_context)
{
  setZValue(address.zoomLevel());
}

QRectF Tile::boundingRect() const
{
  return bounding_rect_;
}

void Tile::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,QWidget *widget)
{
}

void Tile::updateView()
{
  if(mapToScene(bounding_rect_).boundingRect().intersects(view_context_.viewport.map_extents))
  {
    if(address_.zoomLevel() <= view_context_.current_zoom_level)
    {
      setVisible(true);
      if(!pixmap_load_request_sent_)
      {
        auto map_tiles = parentMapTiles();
        if(map_tiles)
        {
          map_tiles->loadTile(address_);
          pixmap_load_request_sent_ = true;
        }
      }

      if(childTiles().empty())
      {
        for(int x = 0; x < 2; x++)
          for(int y = 0; y < 2; y++ )
          {
            auto child_tile = new Tile(view_context_, this, address_.child(x,y));
            child_tile->setScale(0.5);
          }
        updateLayout();
      }
      for(auto tile: childTiles())
        tile->updateView();

      // if we are not the bottom level tile, hide our pixmap unless
      // one of child tiles is not ready to render.
      auto pixmap_item = pixmapItem();
      if(pixmap_item)
        if(address_.zoomLevel() < view_context_.current_zoom_level)
        {
          bool all_child_tiles_ready = true;
          for(auto child_tile: childTiles())
            if(child_tile->isVisible())
            {
              auto child_pixmap_item = child_tile->pixmapItem();
              if(!child_pixmap_item)
                all_child_tiles_ready = false;
            }
            pixmap_item->setVisible(!all_child_tiles_ready);
        }
        else
          pixmap_item->setVisible(true);
    }
    else
    {
      // hide ourselves if we are at too high a zoom level
      setVisible(false);
    }
  }
  else
    // hide ourselves since we are outside of the viewport
    setVisible(false);
}

void Tile::updateLayout()
{
  for(auto tile: childTiles())
  {
    int x = tile->address_.index().x()%2;
    int y = tile->address_.index().y()%2;
    if(view_context_.flip_y)
      y = 1-y;
    tile->setPos(x*web_mercator::tile_size/2.0, y*web_mercator::tile_size/2.0);
    tile->updateLayout();
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
    pixmap_item->setPixmap(pixmap);
  else
  {
    pixmap_item = new QGraphicsPixmapItem(pixmap, this);
    pixmap_item->setZValue(address_.zoomLevel());
  }
  // do we need to scale if not default tile size?
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
