#ifndef MAP_LAYER_H
#define MAP_LAYER_H

#include "map_item.h"

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
  void updateFlags(Qt::ItemFlags& flags) const override;
  void readSettings() override;
  void writeSettings() override;

};

} // namespace map

#endif
