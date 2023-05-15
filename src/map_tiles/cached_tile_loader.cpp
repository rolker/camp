#include "cached_tile_loader.h"

#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QJsonObject>
#include <QJsonDocument>

#include <QDebug>

namespace map_tiles
{

CachedTileLoader::CachedTileLoader(QObject* parent):
  QObject(parent)
{
  network_access_manager_ = new QNetworkAccessManager(this);
  connect(network_access_manager_, &QNetworkAccessManager::finished, this, &CachedTileLoader::downloadFinished);
}

void CachedTileLoader::setCachePath(QString cache_path)
{
  cache_path_ = cache_path;
  QDir cache_dir(cache_path);
  if(!cache_dir.exists())
  {
    qDebug() << "map tile cache directory does not exist: " << cache_path_;
    if(!QDir::root().mkpath(cache_dir.absolutePath()))
      qDebug() << "failed to create cache directory";
  }
}

void CachedTileLoader::setBaseUrl(QString base_url)
{
  base_url_ = base_url;
}

void CachedTileLoader::load(TileAddress address)
{
  if(cache_path_.isEmpty())
  {
    qDebug() << "CachedTileLoader cache path not set";
    return;
  }
  auto url_str = base_url_+address;

  QFileInfo file_path(cache_path_, address);
  if(file_path.exists())
    url_str = "file://"+file_path.filePath();

  QNetworkRequest request(url_str);
  request.setRawHeader("User-Agent", "CCOMAutonomousMissionPlanner/1.0");

  // In order to associate the tile address with the request, it is
  // set as a property of a QObject as a variant. The QObject is then
  // attached to the request.
  QObject* address_holder = new QObject(this);
  QVariant address_variant;
  address_variant.setValue(address);
  address_holder->setProperty("address", address_variant);
  request.setOriginatingObject(address_holder);

  network_access_manager_->get(request);
}

void CachedTileLoader::downloadFinished(QNetworkReply* reply)
{
  if(reply->error() == QNetworkReply::NoError)
  {
    if(reply->request().originatingObject())
    {
      QVariant address_variant = reply->request().originatingObject()->property("address");
      auto address = address_variant.value<TileAddress>();
      auto data = reply->readAll();

      if(!reply->request().url().isLocalFile())
      {
        // Save the data to a local cache location if not a local source
        QFileInfo file_path(cache_path_, address);
        if(!file_path.exists())
        {
          QDir cache_path(cache_path_);
          if(!cache_path.mkpath(file_path.path()))
            qDebug() << "Failed to create directory: " << file_path.path();
        }
        QFile file(file_path.filePath());
        file.open(QIODevice::WriteOnly);
        file.write(data);
        file.close();

        QJsonObject meta;
        meta["url"] = reply->request().url().toString();
        QJsonObject header;
        for(auto pair: reply->rawHeaderPairs())
          header[pair.first] = QString(pair.second);
        meta["reply-header"] = header;
        
        QFile reply_file(file_path.filePath()+".json");
        reply_file.open(QIODevice::WriteOnly);
        reply_file.write(QJsonDocument(meta).toJson());
        reply_file.close();
      }

      QPixmap pixmap;
      pixmap.loadFromData(data, "png");
      emit pixmapLoaded(pixmap, address);
    }

  }
  else
    qDebug() << "Error " << reply->error() << " when getting " << reply->request().url();

}


} // namespace map_tiles
