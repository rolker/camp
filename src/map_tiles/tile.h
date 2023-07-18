#ifndef MAP_TILES_TILE_H
#define MAP_TILES_TILE_H

#include <QGraphicsObject>
#include "../map_view/map_view.h"
#include "../geographicsitem.h"
#include "../map_view/web_mercator.h"
#include <QNetworkReply>
#include "tile_address.h"
#include "../map/item_types.h"

namespace camp
{

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
  Tile(TileAddress address, QGraphicsItem *parentItem = nullptr);

  enum { Type =  map::TileType };

  int type() const override
  {
    // Enable the use of qgraphicsitem_cast with this item.
    return Type;
  }

  QRectF boundingRect() const override;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

  const TileAddress& address() const;

public slots:
  void updatePixmap(QPixmap pixmap);

private:
  QRectF bounding_rect_;

  TileAddress address_;

  bool pixmap_load_request_sent_ = false;

  MapTiles* parentMapTiles() const;
  QGraphicsPixmapItem* pixmapItem() const;
};

} // namespace map_tiles

} // namespace camp

#endif