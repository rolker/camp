#ifndef CATALOG_CATALOG_BROWSER_H
#define CATALOG_CATALOG_BROWSER_H

#include <QWidget>
#include <memory>
#include <vector>

#include "catalog_source.h"

class QPushButton;
class QTreeView;
class QItemSelection;

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

signals:
  /// Emitted after a selected leaf spawns (or resolves to) a flat layer.
  void layerAdded(map::Layer* layer);

private slots:
  void openStore();
  void addSelected();
  void onSelectionChanged(const QItemSelection& selected, const QItemSelection& deselected);

private:
  CatalogSource* sourceForId(const QString& id) const;

  CatalogModel* model_;
  QTreeView* view_;
  QPushButton* open_button_;
  QPushButton* add_button_;
  map::LayerList* target_ = nullptr;
  std::vector<std::unique_ptr<CatalogSource>> sources_;
};

}  // namespace catalog
}  // namespace camp

#endif
