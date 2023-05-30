#include "layer.h"
#include "layer_list.h"
#include <QGraphicsScene>

namespace map
{

Layer::Layer(MapItem* parent, const QString& object_name):
  MapItem(parent, object_name)
{
}

void Layer::updateFlags(Qt::ItemFlags& flags) const
{
  MapItem::updateFlags(flags);
  flags |= Qt::ItemIsEditable;
  flags |= Qt::ItemIsUserCheckable;
  flags |= Qt::ItemIsDragEnabled;
  flags |= Qt::ItemIsDropEnabled;
}


} // namespace map
