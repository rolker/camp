#include "map_tiles.h"
#include <QPainter>
#include "tile.h"
#include <cmath>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include "cached_tile_loader.h"
#include <QDir>
#include <QSettings>
#include <QStyleOptionGraphicsItem>
#include <QTimer>
#include <QVariant>
#include <algorithm>
#include <set>
#include <utility>
#include <vector>
#include "background/tile_layer_presets.h"
#include "wmts/capabilities.h"

namespace camp
{

namespace map_tiles
{

namespace
{

// [#98] Eviction cap for MapTiles::tiles_, computed as
// max(kTileEvictionMinCap, kTileEvictionMultiplier * visible_tile_count).
//
// OSM tiles decode to ~256x256 ARGB (~256 KB each), so the 256-tile floor caps
// the basemap tile buffer at roughly 64 MB — a generous pan buffer sized for the
// salmon operator workstation, large enough that normal pan/zoom never blanks a
// tile that's about to be revisited, yet bounded so an all-afternoon survey can't
// OOM-kill CAMP (issue #98). The multiplier keeps the cap comfortably above the
// working set at high zoom, where the viewport can show many small tiles at once.
constexpr int kTileEvictionMinCap = 256;
constexpr int kTileEvictionMultiplier = 4;

} // namespace

MapTiles::MapTiles(map::MapItem* parentItem, const QString& label, const TileLayout& tile_layout):
  map::Layer(parentItem, label), tile_layout_(tile_layout)
{
  tile_loader_ = new CachedTileLoader(this);

  connect(tile_loader_, &CachedTileLoader::pixmapLoaded, this, &MapTiles::tileLoaded);

  auto dir = QDir::home().filePath(".CCOMAutonomousMissionPlanner/map_tiles/"+label);
  tile_loader_->setCachePath(dir);
  setLayout(tile_layout);
}

QRectF MapTiles::boundingRect() const
{
  return childrenBoundingRect();
}

void MapTiles::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
  auto lod = QStyleOptionGraphicsItem::levelOfDetailFromTransform(painter->worldTransform());

  // scale up view
  lod /= 2.0;

  auto wt = painter->worldTransform();
  auto window = painter->window();
  QPointF top_left((window.x()-wt.m31())/wt.m11(), (window.y()-wt.m32())/wt.m22());
  QPointF bottom_right(top_left.x()+window.width()/wt.m11(), top_left.y()+window.height()/wt.m22());

  int level_number = 0;
  int render_level = tile_layout_.zoom_levels.size()-1;

  std::set<TileAddress> visible_tiles;

  for(const auto& level: tile_layout_.zoom_levels)
  {
    if(level.scale*lod < 2.0)
    {
      render_level = level_number;
      break;
    }
    level_number++;
  }

  if(render_level >= 0)
  {
    const auto& level = tile_layout_.zoom_levels[render_level];
    auto start_x_index = std::max(0,std::min(level.matrix_width,int(floor( (top_left.x() - level.top_left_corner.x())/(level.tile_width*level.scale)))));
    auto end_x_index = std::max(0,std::min(level.matrix_width,int(ceil(( bottom_right.x() - level.top_left_corner.x())/(level.tile_width*level.scale)))));
    auto start_y_index = std::max(0,std::min(level.matrix_height,int(floor( (level.top_left_corner.y()-top_left.y()))/(level.tile_height*level.scale))));
    auto end_y_index = std::max(0,std::min(level.matrix_height,int(ceil( (level.top_left_corner.y()-bottom_right.y())/(level.tile_height*level.scale)))));

    for(int row = start_y_index; row <= end_y_index && row < level.matrix_height; row++)
      for(int col = start_x_index; col <= end_x_index && col < level.matrix_width; col++)
      {
        TileAddress address(&tile_layout_, render_level, QPoint(col, row), layout_epoch_);
        if(tiles_.find(address) == tiles_.end() || tiles_[address] == nullptr)
        {
          tiles_[address] = new Tile(address, this);
          tile_loader_->load(address);
        }
        tiles_[address]->setVisible(true);
        visible_tiles.insert(address);
      }
  }

  for(auto tile: tiles_)
    if(visible_tiles.find(tile.first) == visible_tiles.end())
      tile.second->setVisible(false);

  // [#98] Record the paint generation in which each visible tile was seen, then
  // advance the generation. This is the LRU bookkeeping consumed by
  // evictIfNeeded(): tiles not visited recently have the smallest generation and
  // are evicted first.
  for(const auto& address: visible_tiles)
    tile_last_visible_gen_[address] = paint_generation_;
  ++paint_generation_;

  // [#98] If tiles_ has outgrown the cap, schedule a DEFERRED eviction. Eviction
  // must not happen here: deleting a Tile (a QGraphicsObject scene child) inside
  // paint() removes a scene item mid-paint, a use-after-free risk. Instead queue
  // evictIfNeeded() onto the event loop, where deleting scene items is safe. The
  // eviction_pending_ flag debounces this so we don't post a fresh invocation on
  // every paint while one is already queued.
  const int cap = std::max(kTileEvictionMinCap,
                           kTileEvictionMultiplier * int(visible_tiles.size()));
  if(int(tiles_.size()) > cap && !eviction_pending_)
  {
    eviction_pending_ = true;
    QMetaObject::invokeMethod(this, "evictIfNeeded", Qt::QueuedConnection);
  }
}

void MapTiles::evictIfNeeded()
{
  eviction_pending_ = false;

  // Recompute the cap from the CURRENT visible set and collect eviction
  // candidates: tiles that are not visible right now. A tile that became visible
  // again between scheduling and now must never be evicted, so we guard on the
  // live isVisible() rather than the visible set captured at schedule time.
  int visible_count = 0;
  std::vector<std::pair<quint64, TileAddress>> candidates;
  candidates.reserve(tiles_.size());
  for(const auto& entry: tiles_)
  {
    if(entry.second && entry.second->isVisible())
    {
      ++visible_count;
      continue;
    }
    auto gen_it = tile_last_visible_gen_.find(entry.first);
    quint64 gen = (gen_it != tile_last_visible_gen_.end()) ? gen_it->second : 0;
    candidates.emplace_back(gen, entry.first);
  }

  const int cap = std::max(kTileEvictionMinCap,
                           kTileEvictionMultiplier * visible_count);
  if(int(tiles_.size()) <= cap)
    return;

  // Oldest last-visible generation first (a missing entry sorts as 0 == oldest).
  // cap >= visible_count, so there are always enough non-visible candidates to
  // bring tiles_ down to the cap without touching a visible tile.
  std::sort(candidates.begin(), candidates.end(),
            [](const std::pair<quint64, TileAddress>& a,
               const std::pair<quint64, TileAddress>& b)
            { return a.first < b.first; });

  size_t to_evict = tiles_.size() - cap;
  for(const auto& candidate: candidates)
  {
    if(to_evict == 0)
      break;
    auto tile_it = tiles_.find(candidate.second);
    if(tile_it != tiles_.end())
    {
      delete tile_it->second;
      tiles_.erase(tile_it);
    }
    tile_last_visible_gen_.erase(candidate.second);
    --to_evict;
  }
}

void MapTiles::setLayout(const TileLayout& tile_layout)
{
  for(auto tile: tiles_)
    if(tile.second)
      delete tile.second;
  tiles_.clear();
  // [#98] Drop the LRU eviction bookkeeping for the now-deleted tile set, and
  // cancel any queued eviction (its candidate addresses are gone). Note
  // paint_generation_ is deliberately NOT reset — it stays monotonic; entries for
  // the freshly seeded tiles simply start accumulating again from the current
  // generation on the next paint.
  tile_last_visible_gen_.clear();
  eviction_pending_ = false;
  // [#99] Bump the layout generation so any in-flight pixmap requested under the
  // previous layout (same tile_layout_ pointer across a refresh) is rejected by
  // tileLoaded once it lands on the rebuilt tile set.
  ++layout_epoch_;
  tile_layout_ = tile_layout;
  if(!tile_layout_.zoom_levels.empty())
  {
    const auto &top_level = tile_layout_.zoom_levels.front();
    for(int row = 0; row < top_level.matrix_height; row++)
      for(int col = 0; col < top_level.matrix_width; col++)
      {
        TileAddress address(&tile_layout_, 0, QPoint(col, row), layout_epoch_);
        tiles_[address] = new Tile(address, this);
        tile_loader_->load(address);
      }
  }
}

void MapTiles::setLayoutFromWMTS(const wmts::Capabilities &capabilites, QString layer_id, QString tile_matrix_set)
{
  wmts_capabilites_ = &capabilites;
  wmts_layer_id_ = layer_id;
  wmts_tile_matrix_set_ = tile_matrix_set;
  connect(wmts_capabilites_, &wmts::Capabilities::ready, this, &MapTiles::wmtsCapabilitiesReady);
}

void MapTiles::wmtsCapabilitiesReady()
{
  setLayout(wmts_capabilites_->getLayout(wmts_layer_id_, wmts_tile_matrix_set_));
}

void MapTiles::setRefreshInterval(int msec)
{
  if(msec <= 0)
  {
    // Disable: stop and tear down the timer if one was running.
    if(refresh_timer_)
      refresh_timer_->stop();
    return;
  }

  if(!refresh_timer_)
  {
    refresh_timer_ = new QTimer(this);
    refresh_timer_->setSingleShot(false);
    connect(refresh_timer_, &QTimer::timeout, this, &MapTiles::onRefreshTimer);
  }
  refresh_timer_->start(msec);

  // [#111] A refreshing layer (radar) is exactly where per-refresh URL cache-
  // busting belongs: it re-fetches the same z/x/y URLs every cycle, so without a
  // changing query token a CDN/proxy can re-serve a stale frame. Tying this to
  // setRefreshInterval (and nothing else calls it for static layers) keeps the
  // OSM/WMTS basemap URLs untouched.
  if(tile_loader_)
    tile_loader_->enableCacheBusting();
}

void MapTiles::refreshTiles()
{
  // Force a genuinely fresh re-fetch of every tile. Three steps, all required:
  //   1. bumpCacheBust() advances the per-refresh URL token, so the network GET
  //      is a URL a *CDN/proxy* between CAMP and the origin cannot answer from
  //      its own edge cache. Disk invalidation alone (#99) does NOT defeat that
  //      intermediary cache — that gap is exactly the #111 stale-radar bug.
  //   2. invalidateCache() drops this layer's *local* disk PNGs, so the next
  //      load() goes to the network instead of re-serving the prior frame for
  //      the same z/x/y.
  //   3. setLayout() deletes every current Tile* (each holds a QGraphicsPixmapItem
  //      child) and rebuilds the zoom-0 tiles, which is what actually RE-ISSUES
  //      the load() requests — without it, steps 1-2 only affect tiles fetched
  //      *later*, leaving the already-built (stale) Tile objects on screen.
  // setLayout() also resets memory to the seed set AT each refresh boundary;
  // within a cycle, paint() growth is bounded separately by the [#98] LRU
  // eviction (see evictIfNeeded), so this reset and the eviction cap are
  // complementary — the refresh exists for radar freshness, not the memory bound.
  //
  // The IEM "nexrad-n0q" product resolves to the latest mosaic (no timestamp
  // pinning), so once both caches are bypassed the re-fetch is genuinely current.
  // The cache-buster is a defensive fix: it is correct regardless of whether CDN
  // caching is the sole mechanism (a no-op query param if no intermediary caches).
  // IEM is an interim stopgap — the intended end state is NOAA nowCOAST via WMS
  // (camp#118), at which point the provider's freshness contract is revisited.
  if(tile_loader_)
  {
    tile_loader_->bumpCacheBust();
    tile_loader_->invalidateCache();
  }
  setLayout(tile_layout_);
  update();
}

void MapTiles::onRefreshTimer()
{
  // The 5-minute cadence (#99 Phase 2) for time-varying overlays (radar): each
  // tick re-fetches a fresh frame via the shared refresh path.
  refreshTiles();
}

QVariant MapTiles::itemChange(GraphicsItemChange change, const QVariant& value)
{
  // [#111] Make the FIRST paint after the operator turns a refreshing layer on
  // fetch fresh, not a frame cached in a previous session. The 5-minute timer is
  // the only OTHER thing that refreshes, so enabling radar shortly after launch
  // (or toggling it off→on) would otherwise re-show the previously-built tiles —
  // yesterday's on-disk frame — until the next tick. On the visible transition we
  // run the same full refresh (bump + invalidate + rebuild), so the ensuing paint
  // re-fetches with a CDN-distinct URL. Gated on cache-busting being enabled,
  // which is true only for refreshing (radar) layers — static OSM/WMTS basemaps
  // keep their cache (and built tiles) on show.
  //
  // Tradeoff: rapidly toggling radar off→on re-downloads the layer each time (the
  // "always fresh on show" guarantee is deliberate — never show a stale frame).
  // A freshness guard (skip the reload if the last fetch is recent) would need
  // per-fetch timestamps, which is the broader staleness-tracking work in camp#119;
  // deferred there rather than added here.
  if(change == ItemVisibleHasChanged && value.toBool() && tile_loader_ &&
     tile_loader_->cacheBustToken() != 0)
  {
    refreshTiles();
  }
  return Layer::itemChange(change, value);
}

void MapTiles::updateViewScale(double view_scale)
{
  //updateViewport(view_context_.viewport);
}

void MapTiles::loadTile(TileAddress tile_address)
{
  tile_loader_->load(tile_address);
}

void MapTiles::tileLoaded(QPixmap pixmap, TileAddress tile_address)
{
  if(tiles_.find(tile_address) != tiles_.end() && tiles_[tile_address] != nullptr)
    // The < operator used for map does not consider layout, but the == operator does.
    // This is to make sure an old pixmap loading before a setLayout call doesn't make
    // it to a new layout's tile.
    if(tiles_[tile_address]->address() == tile_address)
      tiles_[tile_address]->updatePixmap(pixmap);
}

void MapTiles::onRemovedFromMap()
{
  // [camp#117] Drop this layer from the BackgroundManager restore list so a
  // user-removed tile layer stays gone next session. Only fires for layers that
  // were persisted (name present in the ids list) — the ids removal gates the
  // group removal, so a non-persisted MapTiles never touches settings.
  QSettings settings;
  QStringList ids = settings.value(background::tileLayerIdsKey()).toStringList();
  if(ids.removeAll(objectName()) > 0)
  {
    settings.setValue(background::tileLayerIdsKey(), ids);
    settings.beginGroup(background::tileLayerRootGroup());
    settings.remove(background::tileLayerGroupKey(objectName()));
    settings.endGroup();
  }
}

} // namespace map_tiles

} // namespace camp
