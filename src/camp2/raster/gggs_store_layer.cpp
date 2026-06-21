#include "gggs_store_layer.h"

#include "gggs_tile_layer.h"

#include <QDir>
#include <QDirIterator>
#include <QFileInfo>
#include <QFileSystemWatcher>
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
  // [camp#102] Watch the store subtree so tiles/epochs that land after open show
  // up without a restart. Created before build() so scan() can register a watch
  // per visited directory.
  watcher_ = new QFileSystemWatcher(this);
  connect(watcher_, &QFileSystemWatcher::directoryChanged, this,
          &GggsStoreLayer::onDirectoryChanged);
  build(directory);
}

void GggsStoreLayer::onRemovedFromMap()
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
  scan(directory);
}

void GggsStoreLayer::scan(const QString& directory)
{
  QDir dir(directory);

  // [camp#102] Watch this directory for added/removed entries (non-recursive).
  if(watcher_ && !watcher_->directories().contains(directory) && dir.exists())
    watcher_->addPath(directory);

  // Tiles directly in this directory -> a tile-set leaf (handles a root that is
  // itself a single tile-set). Skip if already added (incremental re-scan).
  if(dirHasTifs(directory) && !tileset_children_.contains(directory))
  {
    new GggsTileLayer(this, directory);   // default-off via its readSettings()
    tileset_children_.insert(directory);
  }

  // Each subdirectory: a leaf if it holds tiles directly, a grouping node if it
  // only has tiles deeper down. Empty / tile-free subtrees are skipped.
  // NoSymLinks guards against a directory-symlink loop recursing unbounded.
  const QStringList subs = dir.entryList(
    QDir::Dirs | QDir::NoDotAndDotDot | QDir::NoSymLinks, QDir::Name);
  for(const QString& sub : subs)
  {
    const QString subpath = dir.filePath(sub);
    if(dirHasTifs(subpath))
    {
      if(!tileset_children_.contains(subpath))
      {
        new GggsTileLayer(this, subpath);   // default-off
        tileset_children_.insert(subpath);
      }
    }
    else if(subtreeHasTifs(subpath))
    {
      if(!group_children_.contains(subpath))
      {
        new GggsStoreLayer(this, subpath);   // grouping node (NOT forced off)
        group_children_.insert(subpath);
      }
    }
  }
}

void GggsStoreLayer::onDirectoryChanged(const QString& path)
{
  // [camp#102] Incremental: discover newly-landed tile-sets/epochs (added
  // default-off in scan()) and, for a changed tile-set leaf, ask it to re-scan
  // for new tiles. Never rebuilds — existing children and operator on/off choices
  // are preserved.
  //
  // Re-scan from `path` to pick up new direct-tile leaves / grouping subtrees.
  scan(path);

  // If `path` is itself a known tile-set leaf, the change may be a tile landing
  // inside it — tell that leaf to re-scan its directory. Match the child by its
  // directory().
  if(tileset_children_.contains(path))
    for(map::MapItem* child : childMapItems())
      if(auto* leaf = dynamic_cast<GggsTileLayer*>(child))
        if(leaf->directory() == path)
        {
          leaf->rescan();
          break;
        }
}

}  // namespace raster
}  // namespace camp
