#include "cached_tile_loader.h"

#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QJsonObject>
#include <QJsonDocument>
//#include "main/main_window.h"
#include "util/cached_file_loader.h"

#include <QDebug>

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

  QFileInfo file_path(local_cache_path_, address);

  CachedFileClient* client = new CachedFileClient(this);
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
  // ~/.CCOMAutonomousMissionPlanner/map_tiles/<label>). An empty or
  // misconfigured path could otherwise target an unexpected root, so bail unless
  // the path is non-empty AND lives under a "map_tiles" segment. This must never
  // hit the global cache root (~/.CCOMAutonomousMissionPlanner/).
  if(local_cache_path_.isEmpty())
  {
    qDebug() << "CachedTileLoader::invalidateCache skipped: cache path not set";
    return;
  }

  const QString normalized = QDir(local_cache_path_).absolutePath();
  if(!normalized.contains("/map_tiles/"))
  {
    qDebug() << "CachedTileLoader::invalidateCache refused: path is not a "
                "per-layer map_tiles subdir:" << normalized;
    return;
  }

  QDir cache_dir(normalized);
  if(cache_dir.exists())
    cache_dir.removeRecursively();
  // Recreate the (now empty) subdir so the subsequent load() can write into it.
  QDir::root().mkpath(normalized);
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
