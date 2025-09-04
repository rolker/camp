#include "layer_list.h"
#include "map_item_mime_data.h"
#include "layer.h"

namespace map
{

LayerList::LayerList(MapItem* item):
  MapItem(item, "Layers")
{
}

void LayerList::updateFlags(Qt::ItemFlags& flags) const
{
  flags |= Qt::ItemIsDropEnabled;
}

bool LayerList::canDropMimeData(const QMimeData* data, Qt::DropAction action, int row, int col) const
{
  if(action == Qt::MoveAction && col == 0)
  {
    auto map_item_data = qobject_cast<const MapItemMimeData*>(data);
    if(map_item_data)
    {
      auto layer = dynamic_cast<const Layer*>(map_item_data->mapItem());
      if(layer)
        return true;
    }
  }
  return false;
}

} // namepsace map
