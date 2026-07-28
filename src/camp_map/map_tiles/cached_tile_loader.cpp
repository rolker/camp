#include "cached_tile_loader.h"

#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QJsonObject>
#include <QJsonDocument>
#include <QDateTime>
//#include "main/main_window.h"
#include "util/cached_file_loader.h"

#include <QDebug>

#include <algorithm>

namespace camp
{

namespace map_tiles
{

CachedTileLoader::CachedTileLoader(QObject* parent):
  QObject(parent)
{
}

void CachedTileLoader::setCachePath(QString cache_path)
{
  local_cache_path_ = cache_path;
}

void CachedTileLoader::load(TileAddress address)
{
  if(local_cache_path_.isEmpty())
  {
    qDebug() << "CachedTileLoader cache path not set";
    return;
  }
  QString url_str = address.url().c_str();

  // [#111] Defeat intermediary (CDN/proxy) HTTP caching of the cache-buster-less
  // tile URL: when enabled, append a per-refresh token to the *network* URL only.
  // The disk path below is built from `address` (z/x/y.png), so the buster never
  // touches the local cache layout — it just forces the GET to be a URL the CDN
  // cannot answer from its own cache. 0 == disabled (static layers append nothing).
  // Safe degradation: a tile server that 4xx'd on the unknown ?t= param would just
  // yield a blank tile (CachedFileLoader drops a failed reply without setting the
  // pixmap) — never a stale frame. IEM tolerates ?t= today (verified, HTTP 200).
  if(cache_bust_ != 0)
    url_str = QString::fromStdString(withCacheBust(url_str.toStdString(), cache_bust_));

  QFileInfo file_path(local_cache_path_, address);

  // [#177] expects_image opts tile loads into the loader's image gate.
  CachedFileClient* client = new CachedFileClient(this, true);
  connect(client, &CachedFileClient::dataLoaded, this, &CachedTileLoader::dataLoaded);

  QVariant address_variant;
  address_variant.setValue(address);
  client->setProperty("address", address_variant);

  CachedFileLoader::instance()->load(url_str, file_path.filePath(), client);
}

void CachedTileLoader::invalidateCache()
{
  // Safety guard before a recursive removal: only ever delete this loader's own
  // per-layer cache subdir (set in MapTiles to
  // ~/.CCOMAutonomousMissionPlanner/map_tiles/<label>). An empty or misconfigured
  // path could otherwise target an unexpected root, so bail unless the path is
  // non-empty AND is a verified descendant of the resolved cache root's
  // "map_tiles" directory. A *prefix* check (not a free substring) is used so a
  // path that merely contains "/map_tiles/" somewhere — or resolves elsewhere via
  // symlinks/relative segments — cannot pass. This must never hit the global cache
  // root (~/.CCOMAutonomousMissionPlanner/) or any path outside it.
  if(local_cache_path_.isEmpty())
  {
    qDebug() << "CachedTileLoader::invalidateCache skipped: cache path not set";
    return;
  }

  // Resolve the cache root the same way MapTiles builds the per-layer path
  // (QDir::home() + ".CCOMAutonomousMissionPlanner/map_tiles"), then require the
  // configured path to live strictly beneath it.
  const QString allowed_prefix =
      QDir(QDir::home().filePath(".CCOMAutonomousMissionPlanner/map_tiles")).absolutePath() + "/";
  const QString normalized = QDir(local_cache_path_).absolutePath();
  if(!normalized.startsWith(allowed_prefix))
  {
    qDebug() << "CachedTileLoader::invalidateCache refused: path is not under the "
                "resolved per-layer cache root" << allowed_prefix << ":" << normalized;
    return;
  }

  QDir cache_dir(normalized);
  if(cache_dir.exists())
    cache_dir.removeRecursively();
  // Recreate the (now empty) subdir so the subsequent load() can write into it.
  QDir::root().mkpath(normalized);
}

void CachedTileLoader::enableCacheBusting()
{
  // Idempotent: if busting is already on, leave the token alone rather than
  // re-seeding to the current clock — a fresh seed could regress BELOW a token
  // already advanced by bumpCacheBust() (which can sit above wall-clock via its
  // +1 guard), briefly re-exposing a CDN-cached URL. Unreachable today (one
  // setRefreshInterval call per layer) but cheap to make robust.
  if(cache_bust_ != 0)
    return;
  // Seed from the wall clock so the token is distinct across sessions too (the
  // disk cache survives a restart, so a session-local counter starting at 0 each
  // launch could collide with a previously-cached busted URL). std::max with 1
  // keeps it non-zero (0 means disabled) even in the impossible clock-at-epoch case.
  cache_bust_ = std::max<quint64>(1, static_cast<quint64>(QDateTime::currentMSecsSinceEpoch()));
}

void CachedTileLoader::bumpCacheBust()
{
  if(cache_bust_ == 0)  // not a refreshing layer — nothing to bump.
    return;
  // Advance to the current wall clock, but never to an equal-or-lower value than
  // last time: the +1 guard guarantees a STRICTLY greater token even on two
  // refreshes within the same millisecond. Strict monotonicity is the invariant
  // the regression test asserts (and it means each cycle's URL is genuinely new).
  cache_bust_ = std::max<quint64>(cache_bust_ + 1,
                                  static_cast<quint64>(QDateTime::currentMSecsSinceEpoch()));
}

std::string CachedTileLoader::withCacheBust(const std::string& url, quint64 token)
{
  // Join with '?' if the URL has no query yet, else '&'. The XYZ tile URLs used
  // by refreshing layers (e.g. IEM radar) carry no query, so this is the '?' path;
  // the '&' branch keeps the helper correct for any URL that already has one.
  const char separator = (url.find('?') == std::string::npos) ? '?' : '&';
  return url + separator + "t=" + std::to_string(token);
}

void CachedTileLoader::dataLoaded(QByteArray &data, CachedFileClient* client)
{
  auto address = client->property("address").value<TileAddress>();
  QPixmap pixmap;
  pixmap.loadFromData(data, "png");
  emit pixmapLoaded(pixmap, address);
}

} // namespace map_tiles

} // namespace camp
