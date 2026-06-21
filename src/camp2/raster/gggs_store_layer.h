#ifndef RASTER_GGGS_STORE_LAYER_H
#define RASTER_GGGS_STORE_LAYER_H

#include "../map/layer.h"

#include <QSet>
#include <QString>

class QFileSystemWatcher;

namespace camp
{
namespace raster
{

class GggsTileLayer;

/// [camp#90 / I4] Container layer for a GGGS tile *store*. Given a root
/// directory it recursively builds the layer tree: any directory that directly
/// contains `*.tif` becomes a renderable GggsTileLayer leaf; directories above
/// those become grouping GggsStoreLayer nodes (e.g. a store root with
/// `bathymetry/`, `sidescan/` and `backscatter/` modality dirs, each holding
/// `draft/`/`processed/` maturity subdirs). The node itself draws nothing — it
/// only groups children in the Layers tree.
class GggsStoreLayer: public map::Layer
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  GggsStoreLayer(map::MapItem* parentItem, const QString& directory);

  enum { Type = map::GggsStoreLayerType };
  int type() const override { return Type; }

  const QString& directory() const { return directory_; }

protected:
  /// [camp#90] Drop this store root from the persisted list on user removal.
  void onRemovedFromMap() override;

private slots:
  /// [camp#102] A watched directory changed: incrementally add newly-landed
  /// tile-sets/epochs (default-off) and re-scan a changed tile-set leaf. Never
  /// rebuilds — already-loaded tiles and operator on/off choices are preserved.
  void onDirectoryChanged(const QString& path);

private:
  void build(const QString& directory);
  /// [camp#102] Scan @p directory for direct-tile leaves / grouping subtrees,
  /// creating any child not already known. Registers a watch on each visited
  /// directory. Idempotent — existing children are left untouched.
  void scan(const QString& directory);

  QString directory_;

  // [camp#102] Watch the store subtree so tiles/epochs that land after open show
  // up without a restart. Qt watches are non-recursive, so we add one watch per
  // discovered directory (incremental add — see onDirectoryChanged). The inotify
  // watch-count ceiling over a very deep multi-epoch store is DEFERRED (camp#102
  // review finding #4): the Massabesic-scale store this lands against is small;
  // a root+active-epoch cap is premature until stores grow.
  QFileSystemWatcher* watcher_ = nullptr;
  QSet<QString> tileset_children_;   // leaf tile-set dirs already added
  QSet<QString> group_children_;     // grouping subtree dirs already added
};

}  // namespace raster
}  // namespace camp

#endif
