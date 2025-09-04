#ifndef MAP_TILES_CACHED_TILE_LOADER_H
#define MAP_TILES_CACHED_TILE_LOADER_H

#include <QObject>
#include "tile_address.h"
#include <QPixmap>
#include <QDir>

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

signals:
  void pixmapLoaded(QPixmap pixmap, TileAddress tile_address);

public slots:
  void setCachePath(QString cache_path);

private:
  // Base relative file location where map tiles are stored locally
  QString local_cache_path_;

private slots:
  void dataLoaded(QByteArray &data, CachedFileClient* client);

};
 
} // namespace map_tiles

} // namespace camp

#endif
