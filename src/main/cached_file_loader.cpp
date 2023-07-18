#include "cached_file_loader.h"

#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QJsonObject>
#include <QJsonDocument>

#include <QDebug>

namespace camp
{

CachedFileLoader* CachedFileLoader::instance = nullptr;

CachedFileLoader::CachedFileLoader(QObject* parent):
  QObject(parent)
{
  network_access_manager_ = new QNetworkAccessManager(this);
  connect(network_access_manager_, &QNetworkAccessManager::finished, this, &CachedFileLoader::downloadFinished);
  setCachePath(QDir::home().filePath(".CCOMAutonomousMissionPlanner/"));
}

CachedFileLoader* CachedFileLoader::get()
{
  return instance;
}

void CachedFileLoader::construct()
{
  instance = new CachedFileLoader();
}

void CachedFileLoader::destruct()
{
  delete instance;
  instance = nullptr;
}

void CachedFileLoader::setCachePath(QString cache_path)
{
  cache_path_ = cache_path;
  QDir cache_dir(cache_path);
  if(!cache_dir.exists())
  {
    qDebug() << "cache directory does not exist: " << cache_path_;
    if(!QDir::root().mkpath(cache_dir.absolutePath()))
      qDebug() << "failed to create cache directory";
  }
}

void CachedFileLoader::load(QString url, QString cache_local_path, CachedFileClient* client)
{
  QUrl request_url(url);
  if(!cache_path_.isEmpty() && !cache_local_path.isEmpty())
  {
    QFileInfo file_path(cache_path_, cache_local_path);
    if(file_path.exists())
      request_url.setUrl("file://"+file_path.filePath());
  }

  QNetworkRequest request(request_url);
  request.setRawHeader("User-Agent", "CCOMAutonomousMissionPlanner/1.0");

  QVariant local_path_variant;
  local_path_variant.setValue(cache_local_path);
  client->setProperty("cache_local_path", local_path_variant);
  request.setOriginatingObject(client);

  network_access_manager_->get(request);
}

void CachedFileLoader::downloadFinished(QNetworkReply* reply)
{
  if(reply->error() == QNetworkReply::NoError)
  {
    auto* client = qobject_cast<CachedFileClient*>(reply->request().originatingObject());
    if(client)
    {
      QVariant local_path_variant = client->property("cache_local_path");
      auto cache_local_path = local_path_variant.value<QString>();
      auto data = reply->readAll();

      if(!reply->request().url().isLocalFile())
      {
        // Save the data to a local cache location if not a local source
        QFileInfo file_path(cache_path_, cache_local_path);
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

      emit client->dataLoaded(data, client);
    }

  }
  else
    qDebug() << "Error " << reply->error() << " when getting " << reply->request().url();
  reply->deleteLater();
}



CachedFileClient::CachedFileClient(QObject* parent):
  QObject(parent)
{
}

} // namespace camp
