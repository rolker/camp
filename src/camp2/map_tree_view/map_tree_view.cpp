#include "map_tree_view.h"
#include "map_item_delegate.h"
#include "../map/map.h"
#include <QContextMenuEvent>
#include <QMenu>

namespace map_tree_view
{

MapTreeView::MapTreeView(QWidget *parent):
  QTreeView(parent)
{
  setHeaderHidden(true);
  setDragEnabled(true);
  viewport()->setAcceptDrops(true);
  setDropIndicatorShown(true);
  setItemDelegate( new MapItemDelegate(this));
}

void MapTreeView::setMap(map::Map* map)
{
  setModel(map);
  expandToDepth(1);
}

void MapTreeView::contextMenuEvent(QContextMenuEvent* event)
{
  auto map_item = indexAt(event->pos());
  QMenu menu(this);
  auto map = qobject_cast<map::Map*>(model());
  if(map)
    map->contextMenuFor(&menu, map_item);
  menu.exec(event->globalPos());
}

} // namespace map_tree_view
