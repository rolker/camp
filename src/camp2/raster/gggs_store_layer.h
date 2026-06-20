#ifndef RASTER_GGGS_STORE_LAYER_H
#define RASTER_GGGS_STORE_LAYER_H

#include "../map/layer.h"

namespace camp
{
namespace raster
{

/// [camp#90 / I4] Container layer for a GGGS tile *store*. Given a root
/// directory it recursively builds the layer tree: any directory that directly
/// contains `*.tif` becomes a renderable GggsTileLayer leaf; directories above
/// those become grouping GggsStoreLayer nodes (e.g. a store root with
/// `bathymetry/` and `sidescan_backscatter/<epoch>/` products). The node itself
/// draws nothing — it only groups children in the Layers tree.
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

private:
  void build(const QString& directory);

  QString directory_;
};

}  // namespace raster
}  // namespace camp

#endif
