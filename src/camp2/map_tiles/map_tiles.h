#ifndef MAP_TILES_MAP_TILES_H
#define MAP_TILES_MAP_TILES_H

#include "../map/layer.h"
#include "tile_address.h"

class QTimer;

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

  // [#99 Phase 2] Periodically refresh the layer's tiles. On each interval the
  // disk cache is invalidated and the layout is reset, forcing a fresh network
  // fetch — needed for time-varying overlays such as weather radar. msec <= 0
  // disables refresh (the default; existing static layers never call this).
  void setRefreshInterval(int msec);

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

  // [#99 Phase 2] Owned (parent=this), null until setRefreshInterval enables it.
  QTimer* refresh_timer_ = nullptr;
private slots:
  void tileLoaded(QPixmap pixmap, TileAddress tile);

  // [#99 Phase 2] Refresh slot — invoked by refresh_timer_ on timeout, and
  // directly (via QMetaObject::invokeMethod) by test_map_tiles_refresh for a
  // deterministic single-fire check. Invalidates the disk cache then resets the
  // layout, deleting all current Tile* children and re-fetching from the network.
  void onRefreshTimer();
};

} // namespace map_tiles

} // namespace camp

#endif
