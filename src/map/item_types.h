#ifndef MAP_ITEM_TYPES_H
#define MAP_ITEM_TYPES_H

#include <QGraphicsItem>

namespace map
{

// Used to enable qgraphicsitem_cast on classes derived from QGraphicsItem.
// In a derived class' header, in the public section, add something like:
//
// enum { Type =  map::FooType };
//
// int type() const override
// {
//   // Enable the use of qgraphicsitem_cast with this item.
//   return Type;
// }
//
enum ItemType
{
  MapItemType = QGraphicsItem::UserType+1,

  // Descendants of map::MapItem
  LayerType,
  TopLevelItemType,
  LayerListType,
  ToolsManagerType,
  LayerManagerType,
  BackgroundManagerType,
  MapTilesType,
  RasterLayerType,
  NodeManagerType,
  RosLayerType,
  MarkerNamespaceType,
  MarkerType,

  // other QGraphicsItem descendants
  TileType,
};

} // namespace map

#endif
