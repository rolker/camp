#include "gggs_store_layer.h"

#include "gggs_tile_layer.h"

#include <QDir>
#include <QDirIterator>
#include <QFileInfo>
#include <QSettings>

namespace camp
{
namespace raster
{

namespace
{

bool dirHasTifs(const QString& path)
{
  return !QDir(path).entryList(QStringList() << "*.tif" << "*.tiff",
                               QDir::Files).isEmpty();
}

bool subtreeHasTifs(const QString& path)
{
  QDirIterator it(path, QStringList() << "*.tif" << "*.tiff", QDir::Files,
                  QDirIterator::Subdirectories);
  return it.hasNext();
}

}  // namespace

GggsStoreLayer::GggsStoreLayer(map::MapItem* parentItem, const QString& directory):
  map::Layer(parentItem, QFileInfo(directory).fileName()),
  directory_(directory)
{
  build(directory);
}

void GggsStoreLayer::onRemovedByUser()
{
  // [camp#90] Drop this store root from the BackgroundManager restore list so a
  // user-removed store stays gone next session.
  QSettings settings;
  QStringList roots = settings.value("GggsStores/roots").toStringList();
  if(roots.removeAll(directory_) > 0)
    settings.setValue("GggsStores/roots", roots);
}

void GggsStoreLayer::build(const QString& directory)
{
  QDir dir(directory);

  // Tiles directly in this directory -> a tile-set leaf (handles a root that is
  // itself a single tile-set).
  if(dirHasTifs(directory))
    new GggsTileLayer(this, directory);

  // Each subdirectory: a leaf if it holds tiles directly, a grouping node if it
  // only has tiles deeper down. Empty / tile-free subtrees are skipped.
  const QStringList subs = dir.entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
  for(const QString& sub : subs)
  {
    const QString subpath = dir.filePath(sub);
    if(dirHasTifs(subpath))
      new GggsTileLayer(this, subpath);
    else if(subtreeHasTifs(subpath))
      new GggsStoreLayer(this, subpath);
  }
}

}  // namespace raster
}  // namespace camp
