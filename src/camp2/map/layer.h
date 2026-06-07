#ifndef MAP_LAYER_H
#define MAP_LAYER_H

#include "map_item.h"

namespace camp
{
namespace map
{

// Base class for map layers.
class Layer: public MapItem
{
public:
  Layer(MapItem* parent, const QString& object_name);

  enum { Type = LayerType };

  int type() const override
  {
    // Enable the use of qgraphicsitem_cast with this item.
    return Type;
  }

protected:
  /// [#59 ADR-0003] Adds a "Remove" entry (all layers are removable from the
  /// Layers tab). Detaches through the Map model — owners that track specific
  /// layers (e.g. the project's chart/depth bookkeeping) react via the model's
  /// rowsAboutToBeRemoved — then deletes the layer.
  void contextMenu(QMenu* menu) override;
  void updateFlags(Qt::ItemFlags& flags) const override;
  void readSettings() override;
  void writeSettings() override;

};

} // namespace map
} // namespace camp

#endif
