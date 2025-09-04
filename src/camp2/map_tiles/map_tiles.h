#ifndef MAP_TILES_MAP_TILES_H
#define MAP_TILES_MAP_TILES_H

#include "../map/layer.h"
#include "tile_address.h"

namespace camp
{

namespace wmts
{
  class Capabilities;
}

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
  MapTiles(map::MapItem* parentItem, const QString& label, const TileLayout& tile_layout = {});

  enum { Type = map::MapTilesType};
  int type() const override
  {
    return Type;
  }

  QRectF boundingRect() const override;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

  void setLayout(const TileLayout& tile_layout);
  void setLayoutFromWMTS(const wmts::Capabilities &capabilities, QString layer_id = {}, QString tile_matrix_set = {});

  void loadTile(TileAddress tile_address);

  //void setBaseUrl(QString base_url);

public slots:
  void updateViewScale(double view_scale);
  void wmtsCapabilitiesReady();

private:
  TileLayout tile_layout_;
  std::map<TileAddress, Tile*> tiles_;

  CachedTileLoader* tile_loader_;

  const wmts::Capabilities* wmts_capabilites_ = nullptr;
  QString wmts_layer_id_;
  QString wmts_tile_matrix_set_;
private slots:
  void tileLoaded(QPixmap pixmap, TileAddress tile);
};

} // namespace map_tiles

} // namespace camp

#endif
