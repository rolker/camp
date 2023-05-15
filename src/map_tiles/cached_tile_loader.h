#ifndef MAP_TILES_CACHED_TILE_LOADER_H
#define MAP_TILES_CACHED_TILE_LOADER_H

#include <QObject>
#include "tile_address.h"
#include <QPixmap>
#include <QDir>

class QNetworkAccessManager;
class QNetworkReply;

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
  std::string baseUrl() const;

signals:
  void pixmapLoaded(QPixmap pixmap, TileAddress tile_address);

public slots:
  void setCachePath(QString cache_path);
  void setBaseUrl(QString base_url);

private:
  QNetworkAccessManager* network_access_manager_;

  // Base file location where map tiles are stored locally
  QString cache_path_;
  // Base URL from where map tiles can be downloaded
  QString base_url_;

private slots:
  void downloadFinished(QNetworkReply* reply);

};

} // namespace map_tiles

#endif
