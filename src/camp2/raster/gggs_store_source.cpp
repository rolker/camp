#include "gggs_store_source.h"

#include "gggs_tile_layer.h"
#include "gggs_tile_util.h"
#include "../catalog/catalog_item.h"
#include "../map/layer_list.h"

#include <QDir>
#include <QFileInfo>
#include <QSettings>

namespace camp
{
namespace raster
{

namespace
{

// A directory holds a tile-set if it contains a base value GeoTIFF tile
// directly. Salvaged from the retired GggsStoreLayer's anonymous-namespace
// helper. [camp#112] The `*.tif` glob also matches `_time`/`_source` companion
// tiles, so a directory of companions alone is NOT a tile-set — filter to base
// value tiles (isValueTile) before deciding.
bool dirHasTifs(const QString& path)
{
  const QStringList tifs = QDir(path).entryList(
    QStringList() << "*.tif" << "*.tiff", QDir::Files);
  for(const QString& name : tifs)
    if(isValueTile(name))
      return true;
  return false;
}

// Build a CatalogItem subtree for `path`: a leaf if it holds tiles directly, a
// group of discovered children otherwise. Returns nullptr if `path` (and its
// subtree) contains no tiles — which prunes empty / tile-free branches without a
// separate subtree scan (the recursion's nullptr return subsumes the old
// GggsStoreLayer::subtreeHasTifs prune).
std::unique_ptr<catalog::CatalogItem> buildNode(const QString& path,
                                                const QString& source_id)
{
  const QString name = QFileInfo(path).fileName();

  // A directory with tiles directly is a tile-set leaf (handles a root that is
  // itself a single tile-set). Its `*.tif` children are the tiles, not nested
  // tile-sets, so we do not descend further.
  if(dirHasTifs(path))
    return std::make_unique<catalog::CatalogItem>(name, source_id, path);

  // Otherwise a grouping node: recurse into subdirectories. NoSymLinks guards
  // against a directory-symlink loop recursing unbounded.
  auto group = std::make_unique<catalog::CatalogItem>(name);
  const QStringList subs = QDir(path).entryList(
    QDir::Dirs | QDir::NoDotAndDotDot | QDir::NoSymLinks, QDir::Name);
  for(const QString& sub : subs)
    if(auto child = buildNode(QDir(path).filePath(sub), source_id))
      group->addChild(std::move(child));

  if(group->childCount() == 0)
    return nullptr;   // no tiles anywhere under `path`
  return group;
}

}  // namespace

QString GggsStoreSource::seedLabel() const
{
  return QObject::tr("Open tile store");
}

std::unique_ptr<catalog::CatalogItem> GggsStoreSource::discover(const QString& root)
{
  if(root.isEmpty() || !QDir(root).exists())
    return nullptr;
  return buildNode(root, sourceId());
}

map::Layer* GggsStoreSource::instantiate(map::LayerList* layers, const QString& key)
{
  if(!layers || key.isEmpty())
    return nullptr;

  // Dedup-on-select: a flat GggsTileLayer's onRemovedFromMap() de-persists by
  // directory(), so two layers on the same directory would let removing one
  // orphan the other's persistence. If this tile-set is already displayed,
  // return the existing layer instead of spawning a duplicate.
  for(map::MapItem* child : layers->childMapItems())
    if(auto* existing = dynamic_cast<GggsTileLayer*>(child))
      if(existing->directory() == key)
        return existing;

  auto* layer = new GggsTileLayer(layers, key);

  // Persist the selected tile-set so createDefaultLayers() restores it next
  // session (dir-unique; GggsTileLayer::onRemovedFromMap drops it on removal).
  QSettings settings;
  QStringList dirs = settings.value("GggsTileLayers/dirs").toStringList();
  if(!dirs.contains(key))
  {
    dirs.append(key);
    settings.setValue("GggsTileLayers/dirs", dirs);
  }
  return layer;
}

}  // namespace raster
}  // namespace camp
