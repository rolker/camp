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

protected:
  // [#111] When a refreshing layer (radar) becomes visible, drop its disk cache
  // and advance the cache-buster so the FIRST paint after the operator enables it
  // fetches a fresh frame instead of serving a tile cached in a previous session
  // (e.g. yesterday's radar). Static layers (cache-busting off) are untouched, so
  // their disk cache is never dropped on show. The brief gap shows blank, never a
  // stale frame.
  QVariant itemChange(GraphicsItemChange change, const QVariant& value) override;

private:
  TileLayout tile_layout_;
  std::map<TileAddress, Tile*> tiles_;

  // [#98] LRU eviction bookkeeping that bounds tiles_ against the unbounded
  // pan/zoom growth behind the #98 OOM. The OSM/WMTS basemap never refreshes
  // (only the radar overlay calls setRefreshInterval -> setLayout), so without a
  // cap tiles_ grows for every newly-visited tile area until the process is
  // OOM-killed. paint_generation_ is a monotonically increasing paint-call
  // counter; tile_last_visible_gen_ records, per tile, the most recent paint
  // generation in which it was visible — the LRU key used to pick eviction
  // victims (oldest first). tile_last_visible_gen_ is keyed by TileAddress and
  // therefore shares tiles_' ordering: TileAddress::operator< ignores the
  // refresh epoch (tile_address.cpp), which is exactly what we want so the two
  // maps stay in lock-step. eviction_pending_ debounces the deferred eviction so
  // paint() schedules at most one evictIfNeeded() at a time.
  quint64 paint_generation_ = 0;
  std::map<TileAddress, quint64> tile_last_visible_gen_;
  bool eviction_pending_ = false;

  CachedTileLoader* tile_loader_;

  const wmts::Capabilities* wmts_capabilites_ = nullptr;
  QString wmts_layer_id_;
  QString wmts_tile_matrix_set_;

  // [#99 Phase 2] Owned (parent=this), null until setRefreshInterval enables it.
  QTimer* refresh_timer_ = nullptr;

  // [#99] Per-refresh layout generation. Bumped on every setLayout() so that
  // TileAddresses minted after a refresh compare unequal (operator==) to ones
  // minted before it. Without this, onRefreshTimer re-applies the SAME tile_layout_
  // object, leaving the layout pointer unchanged across a refresh — an in-flight
  // pre-refresh pixmap would then satisfy the tileLoaded guard on the rebuilt
  // same-position tile and paint a stale radar frame for up to one cycle.
  quint64 layout_epoch_ = 0;
private slots:
  void tileLoaded(QPixmap pixmap, TileAddress tile);

  // [#98] Deferred LRU eviction of off-screen tiles. paint() only SCHEDULES this
  // (via a queued invocation) — it never deletes Tile* itself, because deleting a
  // scene child mid-paint is a use-after-free risk. Running in the event loop, this
  // slot recomputes the cap from the CURRENT visible count, then deletes the
  // least-recently-visible non-visible tiles until tiles_.size() <= cap. A tile
  // visible at eviction time is never evicted, even if it was off-screen when the
  // eviction was scheduled.
  void evictIfNeeded();

  // [#99 Phase 2] Refresh slot — invoked by refresh_timer_ on timeout, and
  // directly (via QMetaObject::invokeMethod) by test_map_tiles_refresh for a
  // deterministic single-fire check. Invalidates the disk cache then resets the
  // layout, deleting all current Tile* children and re-fetching from the network.
  void onRefreshTimer();
};

} // namespace map_tiles

} // namespace camp

#endif
