#ifndef MAP_TILES_TILE_H
#define MAP_TILES_TILE_H

#include <QGraphicsObject>
#include "../map_view/map_view.h"
#include "../geographicsitem.h"
#include "../map_view/web_mercator.h"
#include <QNetworkReply>
#include "tile_address.h"
#include "../map/item_types.h"

namespace map_tiles
{

class MapTiles;

// Graphics item that draws a map tile at a given
// TileAddress.
class Tile: public QGraphicsObject
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  Tile(QGraphicsItem *parentItem = nullptr, TileAddress address = TileAddress());

  enum { Type =  map::TileType };

  int type() const override
  {
    // Enable the use of qgraphicsitem_cast with this item.
    return Type;
  }

  QRectF boundingRect() const override;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

  const TileAddress& address() const;

  // Find a tile by its address.
  // Returns a pointer to the found tile if it's this tile
  // or an existing descendant of this tile.
  // Returns nullptr if a tile with that address is not found.
  Tile* find(const TileAddress& address);

public slots:
  // Called when Y direction changes
  void updateLayout(bool flip_y);
  void updatePixmap(QPixmap pixmap);

private:
  static constexpr QRectF bounding_rect_  = {0.0, 0.0, web_mercator::tile_size, web_mercator::tile_size};

  TileAddress address_;

  bool pixmap_load_request_sent_ = false;

  MapTiles* parentMapTiles() const;
  QList<Tile*> childTiles() const;
  QGraphicsPixmapItem* pixmapItem() const;
};

} // namespace map_tiles

#endif