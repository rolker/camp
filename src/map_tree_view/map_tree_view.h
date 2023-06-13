#ifndef MAP_TREE_VIEW_MAP_TREE_VIEW_H
#define MAP_TREE_VIEW_MAP_TREE_VIEW_H

#include <QTreeView>

namespace map
{
  class Map;
}

namespace map_tree_view
{

class MapTreeView: public QTreeView
{
public:
  MapTreeView(QWidget *parent = nullptr);

  void setMap(map::Map* map);

private:
  void contextMenuEvent(QContextMenuEvent* event) override;

};

} // namespace map_tree_view

#endif
