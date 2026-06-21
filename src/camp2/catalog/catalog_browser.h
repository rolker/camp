#ifndef CATALOG_CATALOG_BROWSER_H
#define CATALOG_CATALOG_BROWSER_H

#include <QWidget>
#include <memory>
#include <vector>

#include "catalog_source.h"

class QPushButton;
class QTreeView;
class QItemSelection;
class QPoint;

namespace camp
{
namespace map
{
  class Layer;
  class LayerList;
}

namespace catalog
{

class CatalogModel;

/// [camp#104] Embeddable catalog browser: a tree view over a CatalogModel plus
/// a seed ("Open store…") affordance and an "Add to map" button. It is shown as
/// a tab in the Layers panel next to the layer tree (browse here, compose in the
/// layer tree). Generic over CatalogSource: the seed action discovers a source's
/// hierarchy into the model, and "Add to map" routes the selected leaf back to
/// its source's instantiate() to spawn a flat top-level layer in the target
/// LayerList. Piece 3a registers exactly one source (GggsStoreSource).
class CatalogBrowser: public QWidget
{
  Q_OBJECT
public:
  explicit CatalogBrowser(QWidget* parent = nullptr);
  ~CatalogBrowser() override;

  /// The LayerList that "Add to map" spawns selected layers into (the Map's
  /// top-level layers).
  void setTarget(map::LayerList* layers) { target_ = layers; }

  /// Register a source, taking ownership. The first registered source seeds the
  /// "Open store…" action (3a ships one).
  void addSource(std::unique_ptr<CatalogSource> source);

  /// [camp#104] Discover @p root through @p source and, if it yields a node, add
  /// it as a top-level model row and record + persist the seed. Dedups: an
  /// already-browsed (source, root) is re-selected instead of re-added (mirrors
  /// the flat-layer dedup-on-select). Returns true if a node is present for
  /// (source, root) afterwards. The non-dialog core of openStore(), exposed so
  /// the operator's "Open store…" choice and persisted-seed restore share one
  /// path (and so it is unit-testable without a file dialog).
  bool seedRoot(CatalogSource* source, const QString& root);

  /// [camp#104] Re-seed any persisted browse roots (QSettings
  /// CatalogBrowser/seeds). Call once after every source is registered (seeds
  /// reference sources by id). Skips a seed whose source is unknown or whose root
  /// no longer exists on disk (skip-missing, like the flat-layer restore), dedups
  /// against already-present seeds, and rewrites the persisted list once if any
  /// stale/duplicate entry was dropped (a clean restore leaves the key untouched).
  void restoreSeeds();

  /// [camp#104] Remove the browsed root at top-level @p row: drop the model row,
  /// erase the matching seed, and re-persist. Keeps seeds_ aligned with the
  /// model's top-level rows. No-op for an out-of-range row. Driven by the tree's
  /// "Remove from browser" context action (and exercised directly by tests).
  void removeSeed(int row);

  /// [camp#104] Number of browsed roots currently shown (top-level model rows).
  int topLevelCount() const;

signals:
  /// Emitted after a selected leaf spawns (or resolves to) a flat layer.
  void layerAdded(map::Layer* layer);

private slots:
  void openStore();
  void addSelected();
  void onSelectionChanged(const QItemSelection& selected, const QItemSelection& deselected);
  void onContextMenu(const QPoint& pos);

private:
  CatalogSource* sourceForId(const QString& id) const;

  /// A browsed store root: the source that produced it plus its root path. Kept
  /// strictly parallel to the model's top-level rows — addTopLevel appends
  /// exactly one top-level node per seed, in order, so seeds_[i] describes the
  /// i-th top-level row.
  struct Seed { QString sourceId; QString root; };

  /// Select, expand, and scroll to the top-level row at @p seed_index.
  void selectTopLevel(int seed_index);

  /// Write seeds_ to QSettings CatalogBrowser/seeds as a tab-delimited
  /// "sourceId\troot" QStringList (paths may contain commas but not tabs). An
  /// empty list removes the key. Called on every change so disk mirrors seeds_.
  void persistSeeds() const;

  CatalogModel* model_;
  QTreeView* view_;
  QPushButton* open_button_;
  QPushButton* add_button_;
  map::LayerList* target_ = nullptr;
  std::vector<std::unique_ptr<CatalogSource>> sources_;
  std::vector<Seed> seeds_;   // parallel to model_'s top-level rows
};

}  // namespace catalog
}  // namespace camp

#endif
