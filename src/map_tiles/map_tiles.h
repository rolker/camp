#ifndef MAP_TILES_MAP_TILES_H
#define MAP_TILES_MAP_TILES_H

#include <QWidget>
#include <QGraphicsObject>
#include "../map_view/map_view.h"
#include "../geographicsitem.h"
#include "tile_address.h"

namespace map_tiles
{

class Tile;
class CachedTileLoader;

// Contains the info needed for a tile to determine
// if it should display itself.
struct ViewContext
{
  MapView::Viewport viewport;
  uint8_t current_zoom_level = 0;
  bool flip_y = false;
};

// Displays a hiearchy of map tiles from local disk or network sources.
// The tiles are layed out in the OpenStreetMap Slippy map scheme.
class MapTiles: public QGraphicsObject
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  MapTiles(QGraphicsItem *parentItem =0);

  enum { Type = GeoGraphicsItem::MapTilesType};
  int type() const override
  {
    return GeoGraphicsItem::MapTilesType;
  }


  QRectF boundingRect() const override;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

  void loadTile(TileAddress tile_address);

  void setLabel(QString label);
  void setBaseUrl(QString base_url);

public slots:
  void updateViewport(MapView::Viewport viewport);
  void updateMinimumZoomLevel(int level);
  void setYDirection(bool flipped);
  void labelEditingFinished();
  void baseUrlEditingFinished();
  void updateViewScale(double view_scale);

private:
  Tile* top_tile_ = nullptr;

  ViewContext view_context_;

  CachedTileLoader* tile_loader_;

private slots:
  void tileLoaded(QPixmap pixmap, TileAddress tile);
};

} // namespace map_tiles

#endif
