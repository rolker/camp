#ifndef MAP_TILES_CACHED_TILE_LOADER_H
#define MAP_TILES_CACHED_TILE_LOADER_H

#include <QObject>
#include "tile_address.h"
#include <QPixmap>
#include <QDir>
#include <string>

namespace camp
{

class CachedFileClient;

namespace map_tiles
{

// Loads map tile images from a drive or
// from the network via http.
// can cache http tiles locally for performance 
// and to comply with usage policies of public
// map tile sources such as OpenStreetMap.
class CachedTileLoader: public QObject
{
  Q_OBJECT
public:
  CachedTileLoader(QObject* parent=nullptr);

  // Requests that a tile gets loaded.
  // A pixmapLoaded signal is sent once the tile image is ready.
  void load(TileAddress tile);

  QDir cachePath() const;

  // [#99 Phase 2] Remove this layer's disk-cached tiles so the next load()
  // re-fetches from the network instead of serving stale PNGs. Scoped to this
  // loader's own per-layer cache subdir; bails safely if the path is empty or
  // not the expected map_tiles subdir (never touches the global cache root).
  void invalidateCache();

  // [#111] Per-refresh URL cache-busting. invalidateCache() clears only the
  // *local* disk cache; a CDN/proxy between CAMP and the tile origin can still
  // re-serve a stale tile for the same cache-buster-less z/x/y URL. Appending a
  // distinct query token to the request URL each refresh makes the GET a URL the
  // intermediary cannot satisfy from cache. Opt-in (off until enabled), so static
  // OSM/WMTS layers are unaffected; only refreshing layers (radar) enable it.
  //
  // enableCacheBusting() seeds the token from the wall clock so freshness holds
  // across sessions too (the on-disk cache survives a restart). bumpCacheBust()
  // advances it with a strictly-monotonic guard so back-to-back refreshes always
  // produce a distinct token — the testable invariant. The busted token is
  // applied to the network URL only; the on-disk path stays z/x/y.png.
  void enableCacheBusting();
  void bumpCacheBust();
  quint64 cacheBustToken() const { return cache_bust_; }

  // Append a "?t=<token>" (or "&t=<token>" if the URL already has a query) cache-
  // buster to a tile URL. Pure/static so it is unit-testable without a network.
  static std::string withCacheBust(const std::string& url, quint64 token);

signals:
  void pixmapLoaded(QPixmap pixmap, TileAddress tile_address);

public slots:
  void setCachePath(QString cache_path);

private:
  // Base relative file location where map tiles are stored locally
  QString local_cache_path_;

  // [#111] Per-refresh URL cache-buster token. 0 == disabled (the default, so
  // non-refreshing layers append nothing). Non-zero on a refreshing layer once
  // enableCacheBusting() has run; advanced by bumpCacheBust() each cycle.
  quint64 cache_bust_ = 0;

private slots:
  void dataLoaded(QByteArray &data, CachedFileClient* client);

};
 
} // namespace map_tiles

} // namespace camp

#endif
