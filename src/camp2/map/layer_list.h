#ifndef MAP_LAYER_LIST_H
#define MAP_LAYER_LIST_H

#include "map_item.h"

namespace map
{

// Layer used by map to contain top level layers.
class LayerList: public MapItem
{
public:
  LayerList(MapItem* item);

  enum { Type = LayerListType };

  int type() const override
  {
    // Enable the use of qgraphicsitem_cast with this item.
    return Type;
  }

  bool canDropMimeData(const QMimeData* data, Qt::DropAction action, int row, int col) const override;

protected:
  void updateFlags(Qt::ItemFlags& flags) const override;
};

} // namespace map

#endif
