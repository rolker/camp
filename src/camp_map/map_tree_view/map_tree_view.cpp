#include "map_tree_view.h"
#include "map_item_delegate.h"
#include "../map/map.h"
#include <QContextMenuEvent>
#include <QMenu>

namespace camp
{
namespace map_tree_view
{

MapTreeView::MapTreeView(QWidget *parent):
  QTreeView(parent)
{
  setHeaderHidden(true);
  // InternalMove (not the individual drag/accept flags) is required for layer
  // reordering. In plain DragDrop mode the view runs its own clearOrRemove() on
  // the source rows after a MoveAction drop, on top of Map::dropMimeData() having
  // already moved the item — a double-handling that makes the reorder fail to
  // take. InternalMove configures dragEnabled + acceptDrops AND makes the view
  // trust the model's dropMimeData() to perform the whole move (issue #81).
  setDragDropMode(QAbstractItemView::InternalMove);
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
} // namespace camp
