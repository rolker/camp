#ifndef CATALOG_CATALOG_SOURCE_H
#define CATALOG_CATALOG_SOURCE_H

#include <QString>
#include <memory>

namespace camp
{
namespace map
{
  class Layer;
  class LayerList;
}

namespace catalog
{

class CatalogItem;

/// [camp#104] The reuse seam between a browsable catalog and the Layers tree.
/// A concrete source knows how to (a) `discover()` its hierarchy into a
/// CatalogItem tree the generic CatalogModel can present, and (b) `instantiate()`
/// the flat display layer for a selected leaf's opaque key. GGGS stores are the
/// first and only consumer in piece 3a; future layer-manager items (cf. topic
/// discovery, camp #44/#68/#69) implement this same interface to reuse the
/// browser without re-deriving one.
class CatalogSource
{
public:
  virtual ~CatalogSource() = default;

  /// Stable identifier stamped onto this source's leaves (CatalogItem::sourceId).
  /// The browser routes a selected leaf back to the source that produced it.
  virtual QString sourceId() const = 0;

  /// Human-readable label for the browser's "open/seed" affordance (e.g.
  /// "Open tile store…").
  virtual QString seedLabel() const = 0;

  /// Build a catalog subtree for @p seed (for GGGS, a store-root directory) and
  /// return its root node, or nullptr if @p seed yields nothing browsable.
  /// Ownership transfers to the caller (the CatalogModel).
  virtual std::unique_ptr<CatalogItem> discover(const QString& seed) = 0;

  /// Spawn the flat top-level display layer for the selected leaf @p key under
  /// @p layers and return it (or the existing layer if @p key is already
  /// displayed — dedup-on-select). Returns nullptr on failure. A source that
  /// persists its selected layers does so here.
  virtual map::Layer* instantiate(map::LayerList* layers, const QString& key) = 0;
};

}  // namespace catalog
}  // namespace camp

#endif
