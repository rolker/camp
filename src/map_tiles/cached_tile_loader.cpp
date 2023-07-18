#include "cached_tile_loader.h"

#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QJsonObject>
#include <QJsonDocument>
#include "main/camp_main_window.h"
#include "main/cached_file_loader.h"

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

  CachedFileLoader::get()->load(url_str, file_path.filePath(), client);
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
