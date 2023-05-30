#ifndef MAP_TILES_MAP_TILES_H
#define MAP_TILES_MAP_TILES_H

#include "../map/layer.h"
#include "tile_address.h"

namespace map_tiles
{

class Tile;
class CachedTileLoader;

// Displays a hierarchy of map tiles from local disk or network sources.
// The tiles are layed out in the OpenStreetMap Slippy map scheme.
class MapTiles: public map::Layer
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  MapTiles(map::MapItem* parentItem);

  enum { Type = map::MapTilesType};
  int type() const override
  {
    return Type;
  }


  QRectF boundingRect() const override;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

  void loadTile(TileAddress tile_address);

  void setLabel(QString label);
  void setBaseUrl(QString base_url);

  // Returns true if tiles are addressed from south to north.
  bool flipY() const;

  uint8_t minimumZoomLevel() const;
  uint8_t maximumZoomLevel() const;

public slots:
  void setMinimumZoomLevel(uint8_t level);
  void setMaximumZoomLevel(uint8_t level);
  void setFlipY(bool flipped);

  void labelEditingFinished();
  void baseUrlEditingFinished();
  void updateViewScale(double view_scale);

private:
  Tile* top_tile_ = nullptr;

  // Flag to indicate if tiles are ordered from south to north.
  // By default, tiles are ordered from north to south.
  bool flip_y_ = false;

  // Zoom level of the top level tile(s).
  uint8_t minimum_zoom_level_ = 0;

  // Zoom level of the highest detailed tiles.
  // Defaults to 19, which is Open Street Map's max zoom level.
  uint8_t maximum_zoom_level_ = 19;

  CachedTileLoader* tile_loader_;

private slots:
  void tileLoaded(QPixmap pixmap, TileAddress tile);
};

} // namespace map_tiles

#endif
