#include "tile.h"
#include <cmath>
#include "map_tiles.h"
#include <QStyleOptionGraphicsItem>

namespace camp
{

namespace map_tiles
{

//const QRectF Tile::bounding_rect_;

Tile::Tile(TileAddress address, QGraphicsItem *parentItem):
  QGraphicsObject(parentItem), address_(address)
{
  bounding_rect_ = QRectF(0, 0, address.tileLayout().zoom_levels[address.zoomLevel()].tile_width, address.tileLayout().zoom_levels[address.zoomLevel()].tile_height);

  setTransform(QTransform::fromScale(address.scale(), -address.scale()));
  
  setPos(address.topLeftCorner());

  new QGraphicsPixmapItem(this);
}

QRectF Tile::boundingRect() const
{
  return bounding_rect_;
}

void Tile::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,QWidget *widget)
{
  pixmapItem()->setVisible(true);
  return;
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
    const auto& layout = address_.tileLayout().zoom_levels[address_.zoomLevel()];
    if(pixmap.width() != layout.tile_width || pixmap.height() != layout.tile_height)
    {
      if(pixmap.width() > 0 && pixmap.height() > 0)
        pixmap_item->setTransform(QTransform::fromScale(layout.tile_width/double(pixmap.width()), layout.tile_height/double(pixmap.height())));
    }
  }
}

const TileAddress& Tile::address() const
{
  return address_;
}

} // namepsace map_tiles

} // namespace camp
