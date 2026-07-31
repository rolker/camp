#include "cached_file_loader.h"

#include <QApplication>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QJsonObject>
#include <QJsonDocument>
#include <QImage>

#include <QDebug>


namespace camp
{

CachedFileLoader* CachedFileLoader::instance_ = nullptr;

CachedFileLoader::CachedFileLoader(QObject* parent):
  QObject(parent)
{
  network_access_manager_ = new QNetworkAccessManager(this);
  connect(network_access_manager_, &QNetworkAccessManager::finished, this, &CachedFileLoader::downloadFinished);
  setCachePath(QDir::home().filePath(".CCOMAutonomousMissionPlanner/"));
}

CachedFileLoader::~CachedFileLoader()
{
  instance_ = nullptr;
}

CachedFileLoader* CachedFileLoader::instance()
{
  if(!instance_)
    instance_ = new CachedFileLoader(QApplication::instance());
  return instance_;
}

QDir CachedFileLoader::cachePath() const
{
  return QDir(cache_path_);
}

bool CachedFileLoader::isAcceptableImageBody(const QByteArray& data, const QString& content_type)
{
  // [#177] Fast reject: declared text or XML bodies (a WMS error report is
  // HTTP 200 + text/xml or application/vnd.ogc.se_xml) never decode as tiles.
  // Other non-image declarations (e.g. application/octet-stream) may still be
  // valid images — the decode below decides those.
  const auto type = content_type.toLower();
  if(type.startsWith("text/") || type.contains("xml"))
    return false;
  QImage image;
  return image.loadFromData(data);
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
  // [#177] Stash the original network URL so a poisoned-cache rejection in
  // downloadFinished can re-issue the fetch against the network (the request
  // URL itself may have been rewritten to file:// above).
  client->setProperty("network_url", url);
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

      if(client->expectsImage())
      {
        // [#177] WMS servers report errors as HTTP 200 + XML; caching such a
        // body would poison the tile cache until LRU eviction (#98) for
        // layers with no refresh interval. Only image-expecting clients are
        // gated — WMTS capabilities XML loads through this same loader.
        auto content_type = reply->header(QNetworkRequest::ContentTypeHeader).toString();
        if(!isAcceptableImageBody(data, content_type))
        {
          qDebug() << "Rejecting non-image body (Content-Type " << content_type << ") from " << reply->request().url();

          if(reply->request().url().isLocalFile())
          {
            // [#177] Self-heal a poisoned disk cache entry. A pre-fix cache file
            // holding a WMS error body would otherwise be re-read from disk
            // forever (blank tile, no refetch) for layers with no refresh
            // interval — exactly the field symptom #177 was filed for. Delete
            // the poisoned file and its .json sidecar, then re-issue the fetch
            // against the original network URL.
            QFileInfo poisoned(cache_path_, cache_local_path);
            const QString poisoned_path = poisoned.filePath();
            const bool removed = QFile::remove(poisoned_path);
            QFile::remove(poisoned_path+".json");
            // [#177] Only re-issue once the poisoned file is actually gone. If
            // removal fails and the file remains, load() would re-select the
            // file:// path (cpp:75), re-read it, reject it, and re-issue — an
            // unbounded async loop via the QNAM finished signal. The retry is a
            // network request only when removal succeeds; guard on that so a
            // second rejection falls through to the non-local return below (no
            // retry loop). If removal failed, drop the reply and leave the tile
            // blank rather than spin.
            if(removed || !QFileInfo::exists(poisoned_path))
            {
              auto network_url = client->property("network_url").value<QString>();
              reply->deleteLater();
              load(network_url, cache_local_path, client);
              return;
            }
            qDebug() << "Failed to remove poisoned cache file, not re-issuing: " << poisoned_path;
          }
          reply->deleteLater();
          return;
        }
      }

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
    // [#99] Intentional graceful degradation: on any network error we log and
    // drop the reply WITHOUT emitting dataLoaded. The tile's pixmap is never
    // set, so it simply stays blank — no crash, no UI block. This is the desired
    // field-ops behavior when a tile server (e.g. the IEM NEXRAD radar tiles) is
    // unreachable. Do not "fix" this into an emit; a missing tile must be silent.
    qDebug() << "Error " << reply->error() << " when getting " << reply->request().url();
  reply->deleteLater();
}



CachedFileClient::CachedFileClient(QObject* parent, bool expects_image):
  QObject(parent),
  expects_image_(expects_image)
{
}

bool CachedFileClient::expectsImage() const
{
  return expects_image_;
}

} // namespace camp
