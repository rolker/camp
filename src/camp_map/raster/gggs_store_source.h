#ifndef RASTER_GGGS_STORE_SOURCE_H
#define RASTER_GGGS_STORE_SOURCE_H

#include "../catalog/catalog_source.h"

namespace camp
{
namespace raster
{

/// [camp#104] The first (and, in piece 3a, only) concrete CatalogSource: browses
/// a GGGS tile *store* by folder scan and spawns a flat top-level GggsTileLayer
/// for a selected tile-set. This replaces the retired nested GggsStoreLayer —
/// the same modality → maturity → tile-set folder scan now builds CatalogItem
/// nodes instead of in-tree layer nodes, and a selection becomes an independent
/// display layer rather than a buried leaf.
class GggsStoreSource: public catalog::CatalogSource
{
public:
  /// Leaves are stamped "gggs"; the browser routes them back here.
  QString sourceId() const override { return QStringLiteral("gggs"); }
  QString seedLabel() const override;

  /// Scan @p root (a store-root directory): a directory holding `*.tif`
  /// directly becomes a selectable tile-set leaf (key = its path); a directory
  /// with tiles only deeper becomes a group; tile-free subtrees are skipped.
  /// Returns the root node, or nullptr if @p root contains no tiles anywhere.
  std::unique_ptr<catalog::CatalogItem> discover(const QString& root) override;

  /// Spawn a flat GggsTileLayer for tile-set directory @p key under @p layers,
  /// deduping against an already-displayed layer on the same directory, and
  /// persist the directory under `GggsTileLayers/dirs` so it restores next
  /// session. Returns the new (or existing) layer.
  map::Layer* instantiate(map::LayerList* layers, const QString& key) override;
};

}  // namespace raster
}  // namespace camp

#endif
