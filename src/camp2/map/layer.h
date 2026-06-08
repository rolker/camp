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

  /// [#59 PR5] Whether this layer offers a Layers-tab "Remove" action. True by
  /// default (user-loaded charts, discovered ROS layers). System-owned overlay
  /// containers whose owner keeps a raw pointer to them (e.g. the AIS / Collision
  /// Monitor managers' anchor layer) set this false so the user can't delete the
  /// layer out from under the owner.
  void setRemovable(bool removable) { removable_ = removable; }
  bool isRemovable() const { return removable_; }

protected:
  /// [#59 ADR-0003] Adds a "Remove" entry (unless setRemovable(false)). Detaches
  /// through the Map model — owners that track specific layers (e.g. the
  /// project's chart/depth bookkeeping) react via the model's
  /// rowsAboutToBeRemoved — then deletes the layer.
  void contextMenu(QMenu* menu) override;
  void updateFlags(Qt::ItemFlags& flags) const override;
  void readSettings() override;
  void writeSettings() override;

private:
  bool removable_ = true;

};

} // namespace map
} // namespace camp

#endif
